//
// Created by NW_W0 on 8/11/2021.
//

// /// For Demo (speed concern)
// void DoTasksForDemo2(const int& last_job_id, const int& cur_job_id, const int& task_group_id);


void yf::sys::nw_sys::DoTasksForDemo2(const int& last_job_id, const int &cur_job_id, const int& task_group_id)
{
    /// 0. Initialization --- flow control
    //
    nw_status_ptr_->cur_job_success_flag = false;
    bool ugv_mission_success_flag = false;
    bool arm_mission_success_flag = false;

    /// 1. Initialization --- model info
    //
    //  1.1 Get cur_model_config_id From DB
    //  1.2 Get related variables
    cur_model_config_id_    = sql_ptr_->GetModelConfigId(cur_job_id);
    cur_mission_num_        = sql_ptr_->GetUgvMissionConfigNum(cur_model_config_id_);
    cur_model_type_         = tm5.GetModelType(cur_model_config_id_);
    cur_task_mode_          = tm5.GetTaskMode(cur_model_config_id_);

    //  1.3 Get Valid Order For Arm Configs
    //  e.g. {0,0,0,1,1,1}
    cur_valid_result_queue_ = sql_ptr_->GetArmConfigValidResultQueue(cur_model_config_id_);
    cur_first_valid_order_  = sql_ptr_->GetFirstValidOrder(cur_model_config_id_);
    cur_last_valid_order_   = sql_ptr_->GetLastValidOrder(cur_model_config_id_);
    //  e.g {3,4,5}
    cur_valid_indexes_ = sql_ptr_->GetValidIndexes(cur_model_config_id_);

    int cur_order = 0;
    int pre_order = 0;

    /// 2. Assign Ugv Mission

    //  2.1. Ugv: post a new mission via REST
    if(cur_model_config_id_ ==  8072 || 8073 || 8074)
    {
        switch(cur_model_config_id_)
        {
            case 8072:
            {
                cur_mission_guid_ = "a0334511-36f2-11ec-a88e-00012978eb45";
                break;
            }
            case 8073:
            {
                cur_mission_guid_ = "1d274a73-3571-11ec-a88e-00012978eb45";
                break;
            }
            case 8074:
            {
                cur_mission_guid_ = "5a0ebea1-3571-11ec-a88e-00012978eb45";
                break;
            }
        }
    }
    else
    {
        mir100_ptr_->PostMission(cur_model_config_id_);
        //  2.2. Ugv: post actions via REST
        mir100_ptr_->PostActions(cur_model_config_id_);
        //  2.3  Ugv: get current mission_guid
        cur_mission_guid_ = mir100_ptr_->GetCurMissionGUID();
    }

    /// 3. Initial Status Checking
    //
    bool arm_init_status_check_success_flag = tm5.InitialStatusCheckForMission(2);
    bool ugv_init_status_check_success_flag = mir100_ptr_->InitialStatusCheckForMission(2);

    /// 3.1 Start executing a mission, which is a model_config (n ugv_mission_configs and n arm_configs)
    if (arm_init_status_check_success_flag == true &&
        ugv_init_status_check_success_flag == true)
    {
        // kick off mir missions.
        mir100_ptr_->PostMissionQueue(cur_mission_guid_);
        mir100_ptr_->Play();

        /// \brief
        ///
        /// while ugv mission has not finished, keep assigning arm mission config and executing arm mission.
        /// for ugv mission finish info, please refer to PLC Register Assignment.
        bool mission_continue_flag = true;

        while (mission_continue_flag)
        {
            /// while mir's mission has not finished
            // if plc4 ==2 || plc4 == 3 --> break
            // if plc4 !=2 && plc4 != 3 ===> continue
            while(  mir100_ptr_->GetPLCRegisterIntValue(4) != 2 &&
                    mir100_ptr_->GetPLCRegisterIntValue(4) != 3)
            {
                bool arm_sub_mission_success_flag = true;

                ///TIME
                sleep.ms(200);

                /// b.1 Initialization
                bool ugv_mission_continue_flag = false;

                bool arm_mission_continue_flag = false;
                bool arm_wait_plc003_success_flag = false;
                bool arm_wait_plc001_success_flag = false;

                /// b.2 ugv_mission_status_check
                // keep checking
                // if plc004 == 1 and then we know mir mission is executing ;
                // if plc004 == 3 and then we know mir mission is error;
                ugv_mission_continue_flag = mir100_ptr_->MissionStatusCheck(2);

                if(ugv_mission_continue_flag)
                {
                    /// step 0: check arm config is valid or not
                    /// step 1: if it is valid, configuring arm mission. if it is invalid, configure nothing
                    /// step 2: execute arm mission

                    bool arm_config_stage_success_flag = false;
                    int last_valid_order;

                    arm_wait_plc003_success_flag = this->WaitForUgvPLCRegisterInt(3,1,5);

                    ///TIME
//                    sleep.ms(200);

                    if(arm_wait_plc003_success_flag)
                    {
                        /// get current order

                        while(cur_order == pre_order || cur_order == 0)
                        {
                            cur_order = mir100_ptr_->GetPLCRegisterIntValue(2);

                            ///TIME
//                            sleep.ms(200);
                        }

                        pre_order = cur_order;

                        /// check arm config is valid or not

//                        sleep.ms(1000);

                        int  cur_arm_config_id = sql_ptr_->GetArmConfigId(cur_model_config_id_, cur_order);
                        int  is_valid = sql_ptr_->GetArmConfigIsValid(cur_arm_config_id);
                        LOG(INFO) << "current arm config is_valid:" << is_valid;

                        switch(is_valid)
                        {
                            case 0: // invalid
                            {
                                /// 1. Do arm configuration here: assign all arm_mission_configs
                                // ...
                                /// 2. Notify mir100 that arm configuration stage has finished
                                mir100_ptr_->SetPLCRegisterIntValue(3,0);

                                /// 3. Wait for flag for executing all arm_mission_configs
                                arm_wait_plc001_success_flag = this->WaitForUgvPLCRegisterInt(1,1,5);

                                /// 4. Check the flag result
                                if(arm_wait_plc001_success_flag)
                                {
                                    /// 4.1 True

                                    /// 4.1.1 Loop all arm_mission_configs here
                                    // ...
                                    sleep.ms(200);

                                    /// 4.1.2 Check the execution result of all arm_mission_configs
                                    ///    a. if anything wrong, record and then break the loop
                                    ///    b. if everything okay, record and then break the loop

                                    // situation b.
                                    // b.1 Notify mir100 that all arm_mission_configs has finished, you're ready to go :)
                                    mir100_ptr_->SetPLCRegisterIntValue(1,0);

                                    // b.2 Update current arm_config status as Finished.
                                    sql_ptr_->UpdateEachTaskStatus(task_group_id, cur_order, 3);
                                }
                                else
                                {
                                    /// 4.2 False

                                    LOG(INFO) << "Mission failed: ugv failed to set plc001=1 ? Arm needs plc001==1 to execute all arm_mission_configs";

                                    arm_mission_success_flag = false;
                                    mission_continue_flag = false;

                                    /// 4.2.1 Update current arm_config status as Error.
                                    sql_ptr_->UpdateEachTaskStatus(task_group_id, cur_order, 5);

                                    break;
                                }

                                /// 5. Check if this is the Last order, if true, break the loop.
                                if(cur_order == cur_mission_num_)
                                {
                                    sleep.ms(1000);
                                    /// ipc1 loop
                                    // set arm_mission_success_flag

                                    arm_mission_success_flag = true;

                                    // set ugv_mission_success_flag
                                    if(mir100_ptr_->GetPLCRegisterIntValue(4) == 2)
                                    {
                                        ugv_mission_success_flag    = true;
                                        mission_continue_flag       = false;
                                    }
                                }

                                break;
                            }
                            case 1: // valid
                            {
                                /// 1. Do arm configuration here: assign all arm_mission_configs

                                LOG(INFO) << "configure arm mission   [Start!]";
                                auto arm_mission_configs = tm5.ConfigureArmMission(cur_model_config_id_, cur_order);
                                LOG(INFO) << "configure arm mission   [Finished!]";

                                cur_operation_area_ =  arm_mission_configs[0].operation_area;

                                // 1. find last valid order
                                // input: cur_order, which is the cur_valid_order!
                                // 2. get cur_valid_order & last_valid_order
                                std::vector<int>::iterator cur_valid_index_index;

                                if(cur_order == cur_first_valid_order_)
                                {
                                    last_valid_order = cur_order;
                                }
                                else
                                {
                                    cur_valid_index_index = std::find(cur_valid_indexes_.begin(), cur_valid_indexes_.end(), cur_order-1);

                                    auto last_valid_order_index = cur_valid_index_index - cur_valid_indexes_.begin() - 1;

                                    last_valid_order = cur_valid_indexes_[last_valid_order_index] + 1;
                                }

                                // get last operation area
                                int  last_arm_config_id = sql_ptr_->GetArmConfigId(cur_model_config_id_, last_valid_order);
                                auto last_operation_area = tm5.GetOperationArea(last_arm_config_id);

                                /// 2. Notify mir100 that arm configuration stage has finished

                                mir100_ptr_->SetPLCRegisterIntValue(3,0);

                                arm_config_stage_success_flag = true; // no use now?

                                /// 3. Wait for flag for executing all arm_mission_configs
                                arm_wait_plc001_success_flag = this->WaitForUgvPLCRegisterInt(1,1,30);

                                /// 4. Check the flag result
                                if(arm_wait_plc001_success_flag)
                                {
                                    /// 4.1 True
                                    /// 4.1.1 Loop all arm_mission_configs here

                                    ///\workflow Start executing arm mission

                                    ///\ (1) First order, pick the tool?
                                    if(cur_order == cur_first_valid_order_)
                                    {
                                        switch (cur_task_mode_)
                                        {
                                            case data::arm::TaskMode::UVCScanning:
                                            {
                                                if(cur_job_id == q_job_uvc_scanning_ids.front())
                                                {
                                                    // if there is any tool attaching on the arm, place it first
                                                    this->ArmPlaceToolSafety();
                                                    sleep.ms(200);

                                                    this->ArmPickTool(cur_task_mode_);

                                                    tm5.set_remove_tool_flag(false);
                                                }

                                                if(cur_job_id == q_job_uvc_scanning_ids.back())
                                                {
                                                    tm5.set_remove_tool_flag(true);
                                                }

                                                break;
                                            }
                                            case data::arm::TaskMode::Mopping:
                                            {
                                                cur_tool_angle_ = data::arm::ToolAngle::Zero;

                                                /// check if it's the first mopping job
                                                if(cur_job_id == q_job_mopping_ids.front())
                                                {
                                                    // if there is any tool attaching on the arm, place it first
                                                    this->ArmPlaceToolSafety();
//                                                    sleep.ms(200);

                                                    this->ArmPickTool(cur_task_mode_);
//                                                    sleep.ms(200);

                                                    this->ArmPickPad(cur_job_id);
//                                                    sleep.ms(200);

                                                    this->ArmUpdatePadNo();
//                                                    sleep.ms(200);

                                                    // for remove_tool
                                                    tm5.set_remove_tool_flag(false);

                                                    // for change_pad: start the timer
                                                    tm5.set_pad_start_timer();
                                                } else
                                                {
                                                    auto change_pad_flag = tm5.CheckChangePadFlag(sql_ptr_->GetModelConfigId(cur_job_id));

                                                    if(change_pad_flag)
                                                    {
                                                        this->ArmRemovePad(last_job_id);
//                                                        sleep.ms(200);
                                                        this->ArmPickPad(cur_job_id);
//                                                        sleep.ms(200);
                                                        this->ArmUpdatePadNo();
//                                                        sleep.ms(200);

                                                        // for remove_tool
                                                        tm5.set_remove_tool_flag(false);

                                                        // for change_pad: restart the timer
                                                        tm5.set_pad_start_timer();
                                                    }
                                                }

                                                // Check if it's the last mopping job, set the remove_tool_flag
                                                if(cur_job_id == q_job_mopping_ids.back())
                                                {
                                                    tm5.set_remove_tool_flag(true);
                                                }

#if 1 //disable for testing
                                                this->ArmAbsorbWater();
#endif

                                                break;
                                            }
                                        }
                                    }

                                    if(cur_model_config_id_ == 8073 && cur_order == cur_first_valid_order_)
                                    {
                                        // do nothing
                                        this->ArmSetOperationArea(cur_operation_area_);

                                        //
                                        tm5.ArmTask("Post tool_angle_45");
                                        cur_tool_angle_ = data::arm::ToolAngle::FortyFive;
                                    }
                                    else
                                    {
                                        ///\ (2) For each order, move to safety position first.
                                        // before moving to the safety position, we need to check whether absorbing water or not.
                                        if(cur_order == cur_first_valid_order_ )
                                        {
                                            this->ArmSetOperationArea(cur_operation_area_);
                                            tm5.ArmTask("Post arm_home_to_safety");
                                        }
                                        else
                                        {
                                            auto cur_valid_order_index = cur_valid_index_index - cur_valid_indexes_.begin();

                                            // if cur_operation_area is not equal to last one, move to safety position first.
                                            if(cur_operation_area_ != last_operation_area)
                                            {
                                                tm5.ArmTask("Post arm_safety_to_front_p1");

                                                if(cur_valid_order_index % 3 == 0)
                                                {
                                                    tm5.ArmTask("Post tool_angle_0");
                                                    this->ArmAbsorbWater();
                                                }

                                                this->ArmSetOperationArea(cur_operation_area_);
                                                tm5.ArmTask("Post arm_home_to_safety");
                                            }
                                            else
                                            {
                                                if(cur_valid_order_index % 3 == 0)
                                                {
                                                    tm5.ArmTask("Post arm_safety_to_home");
                                                    this->ArmAbsorbWater();
                                                    tm5.ArmTask("Post arm_home_to_safety");
                                                }
                                            }
                                        }

                                        ///\ (3) Check current arm_config mission type, and then execute the mission
                                        auto mission_type = tm5.GetMissionType(sql_ptr_->GetMissionTypeId(cur_model_config_id_, cur_order));

                                        switch (mission_type)
                                        {
                                            case data::arm::MissionType::FixedPosition:
                                            {
                                                ///\ (3) Loop all the arm_mission_configs here

                                                ///\brief for MotionType::FixedPosition
                                                /// amc_skip_conditions:
                                                /// 1. For Landmark
                                                ///     1.1. cannot find the landmark
                                                ///     1.2. landmark deviation too large
                                                /// 2. For D455
                                                ///     2.1 cannot match the origin 2d image.
                                                ///     2.2 TF deviation too large
                                                bool amc_skip_flag = true;

                                                bool amc_deviation_skip_flag = true;
                                                bool amc_range_skip_flag = true;

                                                for (int n = 0; n < arm_mission_configs.size(); n++)
                                                {
                                                    /// I: Find the TF and amc_skip_flag
                                                    //  for first order
                                                    //    I.1. Get the TF first(landmark_tf, camera_tf)
                                                    //    I.2. Assign the amc_skip_flag
                                                    if(n == 0)
                                                    {
                                                        /// I.1
                                                        ///   a. Initialization
                                                        ///     a.1 move to standby_position
                                                        ///     a.2 check&set tool_angle
                                                        ///   b. Find the TF & Set the amc_skip_flag / amc_deviation_skip_flag
                                                        ///   c. Calculation
                                                        ///   d. Check if arm is out of range. / amc_range_skip_flag . Set amc_skip_flag
                                                        ///   e. Return standby_position

                                                        // a.1
                                                        //
                                                        //  a.1.1 get standby_point_str
                                                        auto standby_point = arm_mission_configs[n].standby_position;
                                                        std::string standby_point_str = this->ArmGetPointStr(standby_point);
                                                        //  a.1.2 set standby_point
                                                        tm5.ArmTask("Set standby_p0 = "+standby_point_str);
                                                        //  a.1.3 move to standby_point
                                                        tm5.ArmTask("Move_to standby_p0");

                                                        // a.2.
#if 1 /// disable for demo
                                                        if(cur_model_config_id_ !=  8072 || cur_model_config_id_ !=  8073)
                                                            this->ArmSetToolAngle(cur_task_mode_,arm_mission_configs[n].tool_angle);
#endif

                                                        /// b. vision job initialization
                                                        ///   b.1: for None: do nothing
                                                        ///   b.2: for Landmark: scan landmark, mark down the record
                                                        ///   b.3: for D455: record the point clouds, mark down the record.

                                                        if(cur_model_config_id_ ==  8074)
                                                        {
                                                            // do nothing
                                                            amc_deviation_skip_flag = false;
                                                        }
                                                        else
                                                        {
                                                            switch (arm_mission_configs[n].vision_type)
                                                            {
                                                                case data::arm::VisionType::None:
                                                                {
                                                                    // do nothing
                                                                    break;
                                                                }
                                                                case data::arm::VisionType::Landmark:
                                                                {
                                                                    // 1. move to init_lm_vision_position.
                                                                    //  1.1 retrieve init_lm_vision_position_str
                                                                    std::string ref_vision_lm_init_position_str = this->ArmGetPointStr(arm_mission_configs[n].ref_vision_lm_init_position);
                                                                    //  1.2 set init_lm_vision_position
                                                                    tm5.ArmTask("Set vision_lm_init_p0 = " + ref_vision_lm_init_position_str);
                                                                    //  1.3 move_to init_lm_vision_position
                                                                    tm5.ArmTask("Move_to vision_lm_init_p0");

                                                                    // 2. execute task 'vision_find_landmark'
                                                                    switch (arm_mission_configs[n].model_type)
                                                                    {
                                                                        case data::arm::ModelType::Windows:
                                                                        {
                                                                            tm5.ArmTask("Post vision_find_light_landmark");
                                                                            break;
                                                                        }
                                                                        default:
                                                                        {
                                                                            tm5.ArmTask("Post vision_find_landmark");
                                                                            break;
                                                                        }
                                                                    }

                                                                    // 3. check the result: find_landmark_flag
                                                                    tm5.ArmTask("Post get_find_landmark_flag");

                                                                    // 4. set the amc_skip_flag and record the TF(landmark_tf)
                                                                    //  4.1 find the landmark
                                                                    //     a. found, get the real_landmark_pos but is it deviation?
                                                                    //       a.1 yes ---> amc_skip_flag = True
                                                                    //       a.2 no  ---> amc_skip_flag = False
                                                                    //     b. cannot find ---> amc_skip_flag = True

                                                                    if(tm5.GetFindLandmarkFlag())
                                                                    {
                                                                        LOG(INFO) << "Find Landmark!";

                                                                        // get real_landmark_pos
                                                                        tm5.ArmTask("Post get_landmark_pos_str");
                                                                        real_lm_pos_ = tm5.GetRealLandmarkPos();

                                                                        /// Comparison real_lm_pos & ref_lm_pos. Check whether error is too significant
                                                                        if(tm5.IsLMPosDeviation(arm_mission_configs[n].ref_landmark_pos, real_lm_pos_))
                                                                        {
                                                                            // error too significant, skip current arm mission config!

                                                                            LOG(INFO) << "Error too significant! Skip cur_arm_mission_config!!";

                                                                            arm_sub_mission_success_flag = false;

                                                                            LOG(INFO) << "Skip the whole arm mission configs!";

                                                                            ///TIME
                                                                            sleep.ms(200);
                                                                        }
                                                                        else
                                                                        {
                                                                            LOG(INFO) << "No Deviation!";
                                                                            amc_deviation_skip_flag = false;
                                                                        }
                                                                    }
                                                                    else
                                                                    {
                                                                        LOG(INFO) << "Cannot find Landmark! Skip cur_arm_mission_config!!";

                                                                        arm_sub_mission_success_flag = false;

                                                                        LOG(INFO) << "Skip the whole arm mission configs!";

                                                                        ///TIME
                                                                        sleep.ms(200);
                                                                    }

                                                                    break;
                                                                }
                                                                case data::arm::VisionType::D455:
                                                                {
                                                                    ///  b.2 find the TF!

                                                                    // 0. data management
                                                                    auto arm_mission_config_dir = "../data/point_clouds/real/arm_mission_config_" + std::to_string(arm_mission_configs[n].id);
                                                                    std::filesystem::create_directory(arm_mission_config_dir);

                                                                    auto task_group_dir = arm_mission_config_dir + "/task_group_" + std::to_string(task_group_id);
                                                                    std::filesystem::create_directory(task_group_dir);

                                                                    auto pc_dir = task_group_dir + "/point_cloud/";
                                                                    std::filesystem::create_directory(pc_dir);

                                                                    auto tf_dir = task_group_dir + "/tf/";
                                                                    std::filesystem::create_directory(tf_dir);

                                                                    // 1. retrieve point cloud data in real time and then assign the value
                                                                    //  1.1 move to several points.
                                                                    //  1.2 record the point clouds. (several sets....)
                                                                    //  1.3 save the real_pc_files.

                                                                    std::vector<std::vector<std::string>> real_pc_file_names;

                                                                    std::vector<std::string> each_set_real_pc_pos_names;

                                                                    for(int set = 0 ; set < arm_mission_configs[n].ref_tcp_pos_ids.size() ; set ++)
                                                                    {
                                                                        each_set_real_pc_pos_names.clear();

                                                                        for(int view = 0 ; view < arm_mission_configs[n].ref_tcp_pos_ids[set].size() ; view++)
                                                                        {
                                                                            // get the point
                                                                            auto point_str = this->ArmGetPointStr(sql_ptr_->GetArmPoint(arm_mission_configs[n].ref_tcp_pos_ids[set][view]));
                                                                            // set the point
                                                                            tm5.ArmTask("Set ref_tcp_pos = " + point_str);
                                                                            // move!
                                                                            tm5.ArmTask("Move_to ref_tcp_pos");

                                                                            //  1. define the name
                                                                            std::string id_str = std::to_string(arm_mission_configs[n].id);
                                                                            std::string set_no_str = std::to_string(set+1);
                                                                            std::string view_no_str = std::to_string(view+1);
                                                                            std::string feature_type_name = sql_ptr_->GetFeatureTypeName(arm_mission_configs[n].feature_type_ids[set]);

                                                                            // note: without ".pcd"
                                                                            auto real_pc_file_name = std::to_string(task_group_id) + "-" + id_str + "-" + set_no_str + "-" + view_no_str + "-" + feature_type_name;

                                                                            //todo: 1. record the real point cloud.
                                                                            //todo: 2. save the real_point_cloud file
                                                                            LOG(INFO) << "Vision Job [Start]" << std::endl;
                                                                            // ....
                                                                            auto result = tm5.RecordCurRealPointCloud(pc_dir, real_pc_file_name, arm_mission_configs[n].id, set+1, view+1);
                                                                            // wait for vision_job done
                                                                            LOG(INFO) << "Vision Job [Running]" << std::endl;
                                                                            LOG(INFO) << "Vision Job [Finish]" << std::endl;

                                                                            /// for debug
                                                                            auto tf_file_name = real_pc_file_name +"-tf.txt";
                                                                            auto tf_result = tm5.WriteTMatFile(arm_mission_configs[n].ref_tcp_pos_tfs[set][view],tf_dir, tf_file_name);

                                                                            // push back
                                                                            each_set_real_pc_pos_names.push_back(real_pc_file_name);

                                                                            // for safety concern.
                                                                            tm5.ArmTask("Move_to standby_p0");
                                                                        }

                                                                        real_pc_file_names.push_back(each_set_real_pc_pos_names);
                                                                    }

                                                                    //todo: 2. compare!
                                                                    // ...
                                                                    // ...
                                                                    // 3. get the TF!
                                                                    /// 3.1
                                                                    std::string feature_type;

                                                                    switch (arm_mission_configs[n].model_type)
                                                                    {
                                                                        case data::arm::ModelType::Handle:
                                                                        {
                                                                            feature_type = "planar";
                                                                            break;
                                                                        }
                                                                        case data::arm::ModelType::Handrail:
                                                                        {
                                                                            /// for handrail_higher
#if 0
                                                                            auto feature_type = "handrail_higher";
                                                                                        auto feature_type_id = sql_ptr_->GetFeatureTypeId(feature_type);

                                                                                        std::vector<int> handrail_higher_sets;

                                                                                        // find the corresponding files
                                                                                        for (int m =0; m < arm_mission_configs[n].feature_type_ids.size(); m++)
                                                                                        {
                                                                                            if(arm_mission_configs[n].feature_type_ids[m] == feature_type_id)
                                                                                            {
                                                                                                handrail_higher_sets.push_back(m);
                                                                                            }
                                                                                        }

                                                                                        /// for 1 set 1 view algorithm
                                                                                        if(handrail_higher_sets.size() == 1)
                                                                                        {
                                                                                            auto set_no  = handrail_higher_sets[0];

                                                                                            auto cur_set_view_no = arm_mission_configs[n].ref_pc_file_names[set_no].size();

                                                                                            if (cur_set_view_no == 1)
                                                                                            {
                                                                                                // get the file name;
                                                                                                auto ref_pc_file_name = arm_mission_configs[n].ref_pc_file_names[set_no][0];

                                                                                                auto real_pc_file_name = std::to_string(task_group_id) + "-" + ref_pc_file_name;

                                                                                                std::string real_pc_file =   "..\\data\\point_clouds\\real\\arm_mission_config_" + std::to_string(arm_mission_configs[n].id)
                                                                                                                             + "\\task_group_" + std::to_string(task_group_id) + "\\point_cloud\\" + real_pc_file_name + ".pcd";

                                                                                                std::string ref_pos_tf_file = "..\\data\\point_clouds\\real\\arm_mission_config_" + std::to_string(arm_mission_configs[n].id)
                                                                                                                              + "\\task_group_" + std::to_string(task_group_id) + "\\tf\\" + real_pc_file_name + "-tf.txt";

                                                                                                arm_mission_configs[n].real_pc_file = real_pc_file;
                                                                                                arm_mission_configs[n].ref_pos_tf_file = ref_pos_tf_file;

                                                                                                arm_mission_configs[n].vision_success_flag = tm5.Phase2GetTMat4Handle(real_pc_file,ref_pos_tf_file);
                                                                                                arm_mission_configs[n].TMat = tm5.get_TMat();
                                                                                            }
                                                                                        }

                                                                                        break;
#endif
                                                                            /// for handrail_lower
                                                                            feature_type = "handrail_lower";
                                                                            break;
                                                                        }
                                                                        case data::arm::ModelType::ProtectiveWall:
                                                                        {
                                                                            feature_type = "protective_wall";
                                                                            break;
                                                                        }
                                                                        case data::arm::ModelType::NurseStation:
                                                                        {
                                                                            feature_type = "nurse_station";
                                                                            break;
                                                                        }
                                                                        case data::arm::ModelType::DeskRectangle:
                                                                        {
                                                                            feature_type = "rectangle_desk";
                                                                            break;
                                                                        }
                                                                    }

                                                                    auto feature_type_id = sql_ptr_->GetFeatureTypeId(feature_type);

                                                                    std::vector<int> point_cloud_sets;

                                                                    // find the corresponding files
                                                                    for (int m =0; m < arm_mission_configs[n].feature_type_ids.size(); m++)
                                                                    {
                                                                        if(arm_mission_configs[n].feature_type_ids[m] == feature_type_id)
                                                                        {
                                                                            point_cloud_sets.push_back(m);
                                                                        }
                                                                    }

                                                                    /// 3.2 for 1 set 1 view algorithm
                                                                    if(point_cloud_sets.size() == 1)
                                                                    {
                                                                        auto set_no  = point_cloud_sets[0];

                                                                        auto cur_set_view_no = arm_mission_configs[n].ref_pc_file_names[set_no].size();

                                                                        if (cur_set_view_no == 1)
                                                                        {
                                                                            // get the file name;
                                                                            auto ref_pc_file_name = arm_mission_configs[n].ref_pc_file_names[set_no][0];

                                                                            auto real_pc_file_name = std::to_string(task_group_id) + "-" + ref_pc_file_name;

                                                                            std::string real_pc_file =   "..\\data\\point_clouds\\real\\arm_mission_config_" + std::to_string(arm_mission_configs[n].id)
                                                                                                         + "\\task_group_" + std::to_string(task_group_id) + "\\point_cloud\\" + real_pc_file_name + ".pcd";

                                                                            std::string ref_pos_tf_file = "..\\data\\point_clouds\\real\\arm_mission_config_" + std::to_string(arm_mission_configs[n].id)
                                                                                                          + "\\task_group_" + std::to_string(task_group_id) + "\\tf\\" + real_pc_file_name + "-tf.txt";

                                                                            arm_mission_configs[n].real_pc_file = real_pc_file;
                                                                            arm_mission_configs[n].ref_pos_tf_file = ref_pos_tf_file;

                                                                            arm_mission_configs[n].vision_success_flag = tm5.Phase2GetTMat4Handle(real_pc_file,ref_pos_tf_file,arm_mission_configs[n].id);
                                                                            arm_mission_configs[n].TMat = tm5.get_TMat();
                                                                            arm_mission_configs[n].angle_diff = tm5.get_angle_diff();
                                                                        }
                                                                    }

                                                                    // 4. set the amc_skip_flag?

                                                                    LOG(INFO) << "vision_success_flag: " << arm_mission_configs[n].vision_success_flag;

                                                                    if(arm_mission_configs[n].vision_success_flag  == 1)
                                                                    {
                                                                        amc_deviation_skip_flag = false;
                                                                    }
                                                                    else
                                                                    {
                                                                        amc_deviation_skip_flag = true;
                                                                    }

                                                                    break;
                                                                }
                                                                case data::arm::VisionType::D435:
                                                                {
                                                                    break;
                                                                }
                                                            }
                                                        }

                                                        /// e. return standby_position

                                                        // back to standby_point
                                                        tm5.ArmTask("Move_to standby_p0");
                                                    }

                                                    /// II:
                                                    //  check the amc_skip_flag
                                                    //  1. if ture, skip current arm_mission_config
                                                    //  2. if false, just execute the arm_mission_config
                                                    //    2.1. Initialization
                                                    ///    2.2. Calculation (base on vision_type: calculate the real_points)
                                                    ///      2.2.1 new via_points (real_points)
                                                    ///      2.2.2 new approach_point
                                                    ///      2.2.3 new n_points
                                                    //    2.3. post the arm_mission_config

                                                    if(amc_deviation_skip_flag)
                                                    {
                                                        continue;
                                                    }
                                                    else
                                                    {
                                                        /// c. Calculation
                                                        //TODO: For Testing: arm_mission_configs[n] --> arm_mission_configs[0]
                                                        switch (arm_mission_configs[0].vision_type)
                                                        {
                                                            case data::arm::VisionType::None:
                                                            {
                                                                break;
                                                            }
                                                            case data::arm::VisionType::Landmark:
                                                            {
                                                                // 2.1 calculate the new via_points

                                                                std::deque<yf::data::arm::Point3d> real_via_points;

                                                                real_via_points = tm5.GetRealViaPointsByLM(
                                                                        arm_mission_configs[n].via_points,
                                                                        arm_mission_configs[n].ref_landmark_pos, real_lm_pos_);

                                                                arm_mission_configs[n].via_points.clear();

                                                                arm_mission_configs[n].via_points = real_via_points;

                                                                // 2.2 calculate the real approach point

                                                                auto real_via_approach_point = tm5.GetRealPointByLM(arm_mission_configs[n].via_approach_pos, arm_mission_configs[n].ref_landmark_pos, real_lm_pos_);

                                                                arm_mission_configs[n].via_approach_pos.x  = real_via_approach_point.x;
                                                                arm_mission_configs[n].via_approach_pos.y  = real_via_approach_point.y;
                                                                arm_mission_configs[n].via_approach_pos.z  = real_via_approach_point.z;
                                                                arm_mission_configs[n].via_approach_pos.rx = real_via_approach_point.rx;
                                                                arm_mission_configs[n].via_approach_pos.ry = real_via_approach_point.ry;
                                                                arm_mission_configs[n].via_approach_pos.rz = real_via_approach_point.rz;

                                                                break;
                                                            }
                                                            case data::arm::VisionType::D455:
                                                            {
                                                                // 2.1 calculate the new via_points

                                                                std::deque<yf::data::arm::Point3d> real_via_points;

                                                                real_via_points = tm5.GetRealViaPointsByRS(arm_mission_configs[0].TMat, arm_mission_configs[n].via_points);

                                                                arm_mission_configs[n].via_points.clear();

                                                                arm_mission_configs[n].via_points = real_via_points;

                                                                // 2.2 calculate the real approach point

                                                                auto real_via_approach_point = tm5.GetRealPointByRS(arm_mission_configs[0].TMat,arm_mission_configs[n].via_approach_pos);

                                                                arm_mission_configs[n].via_approach_pos.x  = real_via_approach_point.x;
                                                                arm_mission_configs[n].via_approach_pos.y  = real_via_approach_point.y;
                                                                arm_mission_configs[n].via_approach_pos.z  = real_via_approach_point.z;
                                                                arm_mission_configs[n].via_approach_pos.rx = real_via_approach_point.rx;
                                                                arm_mission_configs[n].via_approach_pos.ry = real_via_approach_point.ry;
                                                                arm_mission_configs[n].via_approach_pos.rz = real_via_approach_point.rz;


                                                                break;
                                                            }
                                                            case data::arm::VisionType::D435:
                                                            {
                                                                break;
                                                            }
                                                        }

                                                        /// d. amc_range_skip_flag. amc_skip_flag

                                                        if(!tm5.IsArmOutOfRange(arm_mission_configs[n].via_points, arm_mission_configs[n].task_mode))
                                                        {
                                                            amc_range_skip_flag = false;
                                                        }

                                                        if(amc_deviation_skip_flag == false && amc_range_skip_flag == false)
                                                        {
                                                            amc_skip_flag = false;
                                                        }

                                                        if(amc_skip_flag)
                                                        {
                                                            continue;
                                                        }
                                                        else
                                                        {
                                                            /// 2.1 Initialization

                                                            // 2.1.1 sub_standby_position
                                                            auto sub_standby_point = arm_mission_configs[n].sub_standby_position;
                                                            std::string sub_standby_point_str = this->ArmGetPointStr(sub_standby_point);
                                                            tm5.ArmTask("Set standby_p1 = "+ sub_standby_point_str);
                                                            tm5.ArmTask("Move_to standby_p1");

                                                            // 2.1.2 check&set tool_angle
#if 0 /// Disable for Demo
                                                            if(cur_model_config_id_ !=  8072)
                                                            this->ArmSetToolAngle(cur_task_mode_,arm_mission_configs[n].tool_angle);
#endif
                                                            /// 2.3 Fire the task and then return to standby_p1 ---> standby_p0

                                                            // 2.3.1 assign n_via_points.
                                                            std::string n_via_points_str = std::to_string(arm_mission_configs[n].n_via_points);
                                                            tm5.ArmTask("Set n_points = " + n_via_points_str);

                                                            // 2.3.2 set approach_point
                                                            this->ArmSetApproachPoint(arm_mission_configs[n].via_approach_pos, arm_mission_configs[n].tool_angle);

                                                            // 2.3.3 set via_points
                                                            this->ArmSetViaPoints(arm_mission_configs[n].via_points, arm_mission_configs[n].tool_angle);

                                                            // 2.3.4 post via_points
                                                            this->ArmPostViaPoints(cur_task_mode_, arm_mission_configs[n].tool_angle, arm_mission_configs[n].model_type, arm_mission_configs[n].id);

                                                            // 2.3.5 post return standby_position
                                                            tm5.ArmTask("Move_to standby_p1");
                                                            tm5.ArmTask("Move_to standby_p0");
                                                        }
                                                    }
                                                }

                                                // a.
                                                tm5.ArmTask("Post arm_back_to_safety");
                                                sleep.ms(200);

                                                break;
                                            }

                                            case data::arm::MissionType::RelativeMove:
                                            {
                                                bool amc_skip_flag = true;

                                                bool amc_deviation_skip_flag = true;
                                                bool amc_range_skip_flag = true;

                                                ///\ (3) Loop all the arm_mission_configs here
                                                for (int n = 0; n < arm_mission_configs.size(); n++)
                                                {
                                                    /// I:  RMove Details
                                                    /// II: Check the Robot Status & RMove result

                                                    /// I:  RMove Details

                                                    /// I.1. Wait for flag to execute rmove_missions.
                                                    LOG(INFO) << "RMove: cur_arm_mission_config_no: " << n  << ". Wait for PLC 015 == 1...";

                                                    if(this->WaitForUgvPLCRegisterInt(15,1,5))
                                                    {
                                                        while(mir100_ptr_->GetPLCRegisterIntValue(6) != 0 )
                                                        {
                                                            /// 0. Check Arm Status First.
                                                            if( nw_status_ptr_->arm_mission_status == data::common::MissionStatus::Error ||
                                                                nw_status_ptr_->arm_mission_status == data::common::MissionStatus::EStop)
                                                            {
                                                                // STOP The RMove Mission
                                                                mir100_ptr_->Pause();
                                                                mir100_ptr_->SetPLCRegisterIntValue(6,0);
                                                                mir100_ptr_->SetPLCRegisterIntValue(5,3);
                                                                break;
                                                            }
                                                            else
                                                            {
                                                                // a. get the PLC 006 value. PLC_006/ rmove_mission_flag
                                                                auto PLC_006 = mir100_ptr_->GetPLCRegisterIntValue(6);

                                                                if(this->WaitForUgvPLCRegisterInt(15,1,5))
                                                                {
                                                                    switch (PLC_006)
                                                                    {
                                                                        case 1:
                                                                        {
                                                                            /// a. move to standby_position
                                                                            // a.1 move to standby_position
                                                                            auto standby_point = arm_mission_configs[n].standby_position;
                                                                            std::string standby_point_str = this->ArmGetPointStr(standby_point);
                                                                            tm5.ArmTask("Set standby_p0 = "+standby_point_str);
                                                                            tm5.ArmTask("Move_to standby_p0");

                                                                            // a.2. check&set tool_angle
//                                                                        this->ArmSetToolAngle(cur_task_mode_,arm_mission_configs[n].tool_angle);

#if 0 /// disable for demo
                                                                            if(cur_model_config_id_ !=  8072)
                                                                            tm5.ArmTask("Post tool_angle_45");
#endif

                                                                            /// b. vision job initialization
                                                                            ///   b.1: for None: do nothing
                                                                            ///   b.2: for Landmark: scan landmark, mark down the record
                                                                            ///   b.3: for D455: do nothing
                                                                            switch (arm_mission_configs[n].vision_type)
                                                                            {
                                                                                case data::arm::VisionType::None:
                                                                                {
                                                                                    // do nothing
                                                                                    break;
                                                                                }
                                                                                case data::arm::VisionType::Landmark:
                                                                                {
                                                                                    // 1. move to init_lm_vision_position.
                                                                                    std::string ref_vision_lm_init_position_str = this->ArmGetPointStr(arm_mission_configs[n].ref_vision_lm_init_position);
                                                                                    tm5.ArmTask("Set vision_lm_init_p0 = " + ref_vision_lm_init_position_str);
                                                                                    tm5.ArmTask("Move_to vision_lm_init_p0");

                                                                                    // 2. execute vision_task 'vision_find_landmark'
                                                                                    switch (arm_mission_configs[n].model_type)
                                                                                    {
                                                                                        case data::arm::ModelType::Windows:
                                                                                        {
                                                                                            tm5.ArmTask("Post vision_find_light_landmark");
                                                                                            break;
                                                                                        }
                                                                                        default:
                                                                                        {
                                                                                            tm5.ArmTask("Post vision_find_landmark");
                                                                                            break;
                                                                                        }
                                                                                    }

                                                                                    // 3. check the result: find_landmark_flag
                                                                                    tm5.ArmTask("Post get_find_landmark_flag");

                                                                                    // 4. set the amc_skip_flag and record the TF(landmark_tf)
                                                                                    //  4.1 find the landmark
                                                                                    //     a. found, get the real_landmark_pos but is it deviation?
                                                                                    //       a.1 yes ---> amc_skip_flag = True
                                                                                    //       a.2 no  ---> amc_skip_flag = False
                                                                                    //     b. cannot find ---> amc_skip_flag = True

                                                                                    if(tm5.GetFindLandmarkFlag())
                                                                                    {
                                                                                        LOG(INFO) << "Find Landmark!";

                                                                                        // get real_landmark_pos
                                                                                        tm5.ArmTask("Post get_landmark_pos_str");
                                                                                        real_lm_pos_ = tm5.GetRealLandmarkPos();

                                                                                        /// Comparison real_lm_pos & ref_lm_pos. Check whether error is too significant
                                                                                        if(tm5.IsLMPosDeviationRMove(arm_mission_configs[n].ref_landmark_pos, real_lm_pos_))
                                                                                        {

                                                                                            // error too significant.
                                                                                            LOG(INFO) << "Keep fine tuning the MiR Pos...";

                                                                                            // 0. get the params first
                                                                                            int iteration_no_rz = tm5.get_PLC_int_value(8);
                                                                                            int iteration_no_x = tm5.get_PLC_int_value(9);
                                                                                            int iteration_no_y = tm5.get_PLC_int_value(10);

                                                                                            // 1. adjust rz
                                                                                            if (iteration_no_rz != 0)
                                                                                            {
                                                                                                int orientation_flag = tm5.get_PLC_int_value(11);

                                                                                                mir100_ptr_->SetPLCRegisterIntValue(8,iteration_no_rz);
                                                                                                mir100_ptr_->SetPLCRegisterIntValue(11,orientation_flag);
                                                                                            }
                                                                                            else
                                                                                            {
                                                                                                mir100_ptr_->SetPLCRegisterIntValue(8,0);
                                                                                                mir100_ptr_->SetPLCRegisterIntValue(11,0);
                                                                                            }

#if 0
                                                                                            // 2. adjust x
                                                                                if (iteration_no_x != 0)
                                                                                {
                                                                                    int orientation_flag = tm5.get_PLC_int_value(12);

                                                                                    mir100_ptr_->SetPLCRegisterIntValue(9,iteration_no_rz);
                                                                                    mir100_ptr_->SetPLCRegisterIntValue(12,orientation_flag);
                                                                                }
                                                                                else
                                                                                {
                                                                                    mir100_ptr_->SetPLCRegisterIntValue(9,0);
                                                                                    mir100_ptr_->SetPLCRegisterIntValue(12,0);
                                                                                }
#endif
                                                                                        }
                                                                                        else
                                                                                        {
                                                                                            LOG(INFO) << "RMove: No Deviation!";

                                                                                            mir100_ptr_->SetPLCRegisterIntValue(6,2);

                                                                                            amc_deviation_skip_flag = false;
                                                                                        }
                                                                                    }
                                                                                    else
                                                                                    {
                                                                                        LOG(INFO) << "Cannot find Landmark! Skip cur_arm_mission_config!!";

                                                                                        arm_sub_mission_success_flag = false;

                                                                                        LOG(INFO) << "Skip the whole arm mission configs!";

                                                                                        ///TIME
                                                                                        sleep.ms(200);

                                                                                        /// STOP The RMove Mission
                                                                                        LOG(INFO) << "Stop Current RMove Mission!";

                                                                                        mir100_ptr_->SetPLCRegisterIntValue(6,0);
                                                                                        mir100_ptr_->SetPLCRegisterIntValue(5,3);
                                                                                    }

                                                                                    break;
                                                                                }
                                                                                case data::arm::VisionType::D455:
                                                                                {
                                                                                    ///  b.2 find the TF!

                                                                                    // 0. data management
                                                                                    auto arm_mission_config_dir = "../data/point_clouds/real/arm_mission_config_" + std::to_string(arm_mission_configs[n].id);
                                                                                    std::filesystem::create_directory(arm_mission_config_dir);

                                                                                    auto task_group_dir = arm_mission_config_dir + "/task_group_" + std::to_string(task_group_id);
                                                                                    std::filesystem::create_directory(task_group_dir);

                                                                                    auto pc_dir = task_group_dir + "/point_cloud/";
                                                                                    std::filesystem::create_directory(pc_dir);

                                                                                    auto tf_dir = task_group_dir + "/tf/";
                                                                                    std::filesystem::create_directory(tf_dir);

                                                                                    // 1. retrieve point cloud data in real time and then assign the value
                                                                                    //  1.1 move to several points.
                                                                                    //  1.2 record the point clouds. (several sets....)
                                                                                    //  1.3 save the real_pc_files.

                                                                                    std::vector<std::vector<std::string>> real_pc_file_names;

                                                                                    std::vector<std::string> each_set_real_pc_pos_names;

                                                                                    for(int set = 0 ; set < arm_mission_configs[n].ref_tcp_pos_ids.size() ; set ++)
                                                                                    {
                                                                                        each_set_real_pc_pos_names.clear();

                                                                                        for(int view = 0 ; view < arm_mission_configs[n].ref_tcp_pos_ids[set].size() ; view++)
                                                                                        {
                                                                                            // get the point
                                                                                            auto point_str = this->ArmGetPointStr(sql_ptr_->GetArmPoint(arm_mission_configs[n].ref_tcp_pos_ids[set][view]));
                                                                                            // set the point
                                                                                            tm5.ArmTask("Set ref_tcp_pos = " + point_str);
                                                                                            // move!
                                                                                            tm5.ArmTask("Move_to ref_tcp_pos");

                                                                                            //  1. define the name
                                                                                            std::string id_str = std::to_string(arm_mission_configs[n].id);
                                                                                            std::string set_no_str = std::to_string(set+1);
                                                                                            std::string view_no_str = std::to_string(view+1);
                                                                                            std::string feature_type_name = sql_ptr_->GetFeatureTypeName(arm_mission_configs[n].feature_type_ids[set]);

                                                                                            // note: without ".pcd"
                                                                                            auto real_pc_file_name = std::to_string(task_group_id) + "-" + id_str + "-" + set_no_str + "-" + view_no_str + "-" + feature_type_name;

                                                                                            //todo: 1. record the real point cloud.
                                                                                            //todo: 2. save the real_point_cloud file
                                                                                            LOG(INFO) << "Vision Job [Start]" << std::endl;
                                                                                            // ....
                                                                                            auto result = tm5.RecordCurRealPointCloud(pc_dir, real_pc_file_name, arm_mission_configs[n].id, set+1, view+1);
                                                                                            // wait for vision_job done
                                                                                            LOG(INFO) << "Vision Job [Running]" << std::endl;
                                                                                            LOG(INFO) << "Vision Job [Finish]" << std::endl;

                                                                                            /// for debug
                                                                                            auto tf_file_name = real_pc_file_name +"-tf.txt";
                                                                                            auto tf_result = tm5.WriteTMatFile(arm_mission_configs[n].ref_tcp_pos_tfs[set][view],tf_dir, tf_file_name);

                                                                                            // push back
                                                                                            each_set_real_pc_pos_names.push_back(real_pc_file_name);

                                                                                            // for safety concern.
                                                                                            tm5.ArmTask("Move_to standby_p0");
                                                                                        }

                                                                                        real_pc_file_names.push_back(each_set_real_pc_pos_names);
                                                                                    }

                                                                                    //todo: 2. compare!
                                                                                    // ...
                                                                                    // ...
                                                                                    // 3. get the TF!


                                                                                    /// 3.1
                                                                                    std::string feature_type;

                                                                                    switch (arm_mission_configs[n].model_type)
                                                                                    {
                                                                                        case data::arm::ModelType::Handle:
                                                                                        {
                                                                                            feature_type = "planar";
                                                                                            break;
                                                                                        }
                                                                                        case data::arm::ModelType::Handrail:
                                                                                        {
                                                                                            /// for handrail_higher
#if 0
                                                                                            auto feature_type = "handrail_higher";
                                                                                        auto feature_type_id = sql_ptr_->GetFeatureTypeId(feature_type);

                                                                                        std::vector<int> handrail_higher_sets;

                                                                                        // find the corresponding files
                                                                                        for (int m =0; m < arm_mission_configs[n].feature_type_ids.size(); m++)
                                                                                        {
                                                                                            if(arm_mission_configs[n].feature_type_ids[m] == feature_type_id)
                                                                                            {
                                                                                                handrail_higher_sets.push_back(m);
                                                                                            }
                                                                                        }

                                                                                        /// for 1 set 1 view algorithm
                                                                                        if(handrail_higher_sets.size() == 1)
                                                                                        {
                                                                                            auto set_no  = handrail_higher_sets[0];

                                                                                            auto cur_set_view_no = arm_mission_configs[n].ref_pc_file_names[set_no].size();

                                                                                            if (cur_set_view_no == 1)
                                                                                            {
                                                                                                // get the file name;
                                                                                                auto ref_pc_file_name = arm_mission_configs[n].ref_pc_file_names[set_no][0];

                                                                                                auto real_pc_file_name = std::to_string(task_group_id) + "-" + ref_pc_file_name;

                                                                                                std::string real_pc_file =   "..\\data\\point_clouds\\real\\arm_mission_config_" + std::to_string(arm_mission_configs[n].id)
                                                                                                                             + "\\task_group_" + std::to_string(task_group_id) + "\\point_cloud\\" + real_pc_file_name + ".pcd";

                                                                                                std::string ref_pos_tf_file = "..\\data\\point_clouds\\real\\arm_mission_config_" + std::to_string(arm_mission_configs[n].id)
                                                                                                                              + "\\task_group_" + std::to_string(task_group_id) + "\\tf\\" + real_pc_file_name + "-tf.txt";

                                                                                                arm_mission_configs[n].real_pc_file = real_pc_file;
                                                                                                arm_mission_configs[n].ref_pos_tf_file = ref_pos_tf_file;

                                                                                                arm_mission_configs[n].vision_success_flag = tm5.Phase2GetTMat4Handle(real_pc_file,ref_pos_tf_file);
                                                                                                arm_mission_configs[n].TMat = tm5.get_TMat();
                                                                                            }
                                                                                        }

                                                                                        break;
#endif

                                                                                            /// for handrail_lower
                                                                                            feature_type = "handrail_lower";
                                                                                            break;
                                                                                        }
                                                                                        case data::arm::ModelType::ProtectiveWall:
                                                                                        {
                                                                                            feature_type = "protective_wall";
                                                                                            break;
                                                                                        }
                                                                                        case data::arm::ModelType::NurseStation:
                                                                                        {
                                                                                            feature_type = "nurse_station";
                                                                                            break;
                                                                                        }
                                                                                        case data::arm::ModelType::DeskRectangle:
                                                                                        {
                                                                                            feature_type = "rectangle_desk";
                                                                                            break;
                                                                                        }
                                                                                    }

                                                                                    auto feature_type_id = sql_ptr_->GetFeatureTypeId(feature_type);

                                                                                    std::vector<int> point_cloud_sets;

                                                                                    // find the corresponding files
                                                                                    for (int m =0; m < arm_mission_configs[n].feature_type_ids.size(); m++)
                                                                                    {
                                                                                        if(arm_mission_configs[n].feature_type_ids[m] == feature_type_id)
                                                                                        {
                                                                                            point_cloud_sets.push_back(m);
                                                                                        }
                                                                                    }

                                                                                    /// 3.2 for 1 set 1 view algorithm
                                                                                    if(point_cloud_sets.size() == 1)
                                                                                    {
                                                                                        auto set_no  = point_cloud_sets[0];

                                                                                        auto cur_set_view_no = arm_mission_configs[n].ref_pc_file_names[set_no].size();

                                                                                        if (cur_set_view_no == 1)
                                                                                        {
                                                                                            // get the file name;
                                                                                            auto ref_pc_file_name = arm_mission_configs[n].ref_pc_file_names[set_no][0];

                                                                                            auto real_pc_file_name = std::to_string(task_group_id) + "-" + ref_pc_file_name;

                                                                                            std::string real_pc_file =   "..\\data\\point_clouds\\real\\arm_mission_config_" + std::to_string(arm_mission_configs[n].id)
                                                                                                                         + "\\task_group_" + std::to_string(task_group_id) + "\\point_cloud\\" + real_pc_file_name + ".pcd";

                                                                                            std::string ref_pos_tf_file = "..\\data\\point_clouds\\real\\arm_mission_config_" + std::to_string(arm_mission_configs[n].id)
                                                                                                                          + "\\task_group_" + std::to_string(task_group_id) + "\\tf\\" + real_pc_file_name + "-tf.txt";

                                                                                            arm_mission_configs[n].real_pc_file = real_pc_file;
                                                                                            arm_mission_configs[n].ref_pos_tf_file = ref_pos_tf_file;

                                                                                            arm_mission_configs[n].vision_success_flag = tm5.Phase2GetTMat4Handle(real_pc_file,ref_pos_tf_file, arm_mission_configs[n].id);
                                                                                            arm_mission_configs[n].TMat = tm5.get_TMat();
                                                                                            arm_mission_configs[n].angle_diff = tm5.get_angle_diff();
                                                                                        }
                                                                                    }

                                                                                    // 4. set the amc_skip_flag?

                                                                                    LOG(INFO) << "vision_success_flag: " << arm_mission_configs[n].vision_success_flag;

                                                                                    if(arm_mission_configs[n].vision_success_flag  == 1)
                                                                                    {
                                                                                        if(tm5.IsRSPosDeviationRMove(arm_mission_configs[n].angle_diff))
                                                                                        {

                                                                                            // error too significant.
                                                                                            LOG(INFO) << "Keep fine tuning the MiR Pos...";

                                                                                            // 0. get the params first
                                                                                            int iteration_no_rz = tm5.get_PLC_int_value(8);
                                                                                            int iteration_no_x = tm5.get_PLC_int_value(9);
                                                                                            int iteration_no_y = tm5.get_PLC_int_value(10);

                                                                                            // 1. adjust rz
                                                                                            if (iteration_no_rz != 0)
                                                                                            {
                                                                                                int orientation_flag = tm5.get_PLC_int_value(11);

                                                                                                mir100_ptr_->SetPLCRegisterIntValue(8,iteration_no_rz);
                                                                                                mir100_ptr_->SetPLCRegisterIntValue(11,orientation_flag);
                                                                                            }
                                                                                            else
                                                                                            {
                                                                                                mir100_ptr_->SetPLCRegisterIntValue(8,0);
                                                                                                mir100_ptr_->SetPLCRegisterIntValue(11,0);
                                                                                            }

#if 0
                                                                                            // 2. adjust x
                                                                                if (iteration_no_x != 0)
                                                                                {
                                                                                    int orientation_flag = tm5.get_PLC_int_value(12);

                                                                                    mir100_ptr_->SetPLCRegisterIntValue(9,iteration_no_rz);
                                                                                    mir100_ptr_->SetPLCRegisterIntValue(12,orientation_flag);
                                                                                }
                                                                                else
                                                                                {
                                                                                    mir100_ptr_->SetPLCRegisterIntValue(9,0);
                                                                                    mir100_ptr_->SetPLCRegisterIntValue(12,0);
                                                                                }
#endif
                                                                                        }
                                                                                        else
                                                                                        {
                                                                                            LOG(INFO) << "RS: RMove: No Deviation!";

                                                                                            mir100_ptr_->SetPLCRegisterIntValue(6,2);

                                                                                            amc_deviation_skip_flag = false;
                                                                                        }
                                                                                    }
                                                                                    else
                                                                                    {
                                                                                        LOG(INFO) << "Cannot find BBox! Skip cur_arm_mission_config!!";

                                                                                        arm_sub_mission_success_flag = false;

                                                                                        LOG(INFO) << "Skip the whole arm mission configs!";

                                                                                        ///TIME
                                                                                        sleep.ms(200);

                                                                                        /// STOP The RMove Mission
                                                                                        LOG(INFO) << "Stop Current RMove Mission!";

                                                                                        mir100_ptr_->SetPLCRegisterIntValue(6,0);
                                                                                        mir100_ptr_->SetPLCRegisterIntValue(5,3);
                                                                                    }

                                                                                    break;
                                                                                }
                                                                                case data::arm::VisionType::D435:
                                                                                {
                                                                                    break;
                                                                                }
                                                                            }

                                                                            /// c. return standby_position

                                                                            tm5.ArmTask("Move_to standby_p0");

                                                                            /// d. set PLC 015 = 0. Handover the Mission.
                                                                            mir100_ptr_->SetPLCRegisterIntValue(15,0);

                                                                            break;
                                                                        }

                                                                        case 2:
                                                                        {
                                                                            // 2.1 sub_standby_position
                                                                            auto sub_standby_point = arm_mission_configs[n].sub_standby_position;
                                                                            std::string sub_standby_point_str = this->ArmGetPointStr(sub_standby_point);
                                                                            tm5.ArmTask("Set standby_p1 = "+ sub_standby_point_str);
                                                                            tm5.ArmTask("Move_to standby_p1");

                                                                            // 2.2 check&set tool_angle
                                                                            if(cur_model_config_id_ !=  8072)
                                                                                this->ArmSetToolAngle(cur_task_mode_,arm_mission_configs[n].tool_angle);

                                                                            // 2.3 via_approach_point

                                                                            yf::data::arm::Point3d real_via_approach_point;

                                                                            switch (arm_mission_configs[n].vision_type)
                                                                            {
                                                                                case data::arm::VisionType::Landmark:
                                                                                {
                                                                                    real_via_approach_point = tm5.GetRealPointByLM(arm_mission_configs[n].via_approach_pos, arm_mission_configs[n].ref_landmark_pos, real_lm_pos_);
                                                                                    break;
                                                                                }
                                                                                case data::arm::VisionType::D455:
                                                                                {
                                                                                    real_via_approach_point = tm5.GetRealPointByRS(arm_mission_configs[n].TMat,arm_mission_configs[n].via_approach_pos);
                                                                                    break;
                                                                                }
                                                                            }

                                                                            arm_mission_configs[n].via_approach_pos.x  = real_via_approach_point.x;
                                                                            arm_mission_configs[n].via_approach_pos.y  = real_via_approach_point.y;
                                                                            arm_mission_configs[n].via_approach_pos.z  = real_via_approach_point.z;
                                                                            arm_mission_configs[n].via_approach_pos.rx = real_via_approach_point.rx;
                                                                            arm_mission_configs[n].via_approach_pos.ry = real_via_approach_point.ry;
                                                                            arm_mission_configs[n].via_approach_pos.rz = real_via_approach_point.rz;

                                                                            this->ArmSetApproachPoint(arm_mission_configs[n].via_approach_pos, arm_mission_configs[n].tool_angle);
                                                                            switch (arm_mission_configs[n].tool_angle)
                                                                            {
                                                                                case data::arm::ToolAngle::Zero:
                                                                                {
                                                                                    tm5.ArmTask("Move_to via0_approach_point");
                                                                                    break;
                                                                                }
                                                                                case data::arm::ToolAngle::FortyFive:
                                                                                {
                                                                                    tm5.ArmTask("Move_to via45_approach_point");
                                                                                    break;
                                                                                }
                                                                            }

                                                                            /// 2.4 rmove_param_init
                                                                            std::chrono::time_point<std::chrono::steady_clock> start,end;
                                                                            std::chrono::duration<float> duration;
                                                                            tm5.SetRMoveForceFlag(0);

                                                                            /// 2.5 awake the thread_RMoveForceNode
                                                                            cur_tool_angle_ = arm_mission_configs[n].tool_angle;
                                                                            rmove_start_flag_ = true;
                                                                            sleep.ms(200);
                                                                            //notify
                                                                            std::unique_lock<std::mutex> ull(mux_Blocking_RMoveForceNode);
                                                                            cv_Blocking_RMoveForceNode.notify_one();
                                                                            sleep.ms(200);

                                                                            if(this->WaitForArmRMoveForceFlag(1,1))
                                                                            {
                                                                                // set PLC 006 = 3. notice ugv
                                                                                // set PLC 015 = 0. notice ugv
                                                                                mir100_ptr_->SetPLCRegisterIntValue(6,3);
                                                                                mir100_ptr_->SetPLCRegisterIntValue(15,0);

                                                                                // get current time.
                                                                                start = std::chrono::high_resolution_clock::now();
                                                                            }


                                                                            bool rmove_continue_flag = true;
                                                                            while (rmove_continue_flag)
                                                                            {
                                                                                // 1. check arm status first.
                                                                                if(nw_status_ptr_->arm_mission_status == data::common::MissionStatus::Error ||
                                                                                   nw_status_ptr_->arm_mission_status == data::common::MissionStatus::EStop)
                                                                                {
                                                                                    LOG(INFO) << "RMove: Arm is Error! Please Check";

                                                                                    rmove_continue_flag = false;
                                                                                    /// STOP The RMove Mission
                                                                                    mir100_ptr_->Pause();
                                                                                    mir100_ptr_->SetPLCRegisterIntValue(6,0);
                                                                                    mir100_ptr_->SetPLCRegisterIntValue(5,3);

                                                                                    break;
                                                                                }

                                                                                // 2. timeout?
                                                                                end = std::chrono::high_resolution_clock::now();
                                                                                duration = end - start;
                                                                                float min = duration.count() / 60.0f;
                                                                                if( min > 15 )
                                                                                {
                                                                                    LOG(INFO) << "RMove: already last 15 min. Timeout!";

                                                                                    rmove_continue_flag = false;
                                                                                    /// STOP The RMove Mission
                                                                                    mir100_ptr_->Pause();
                                                                                    mir100_ptr_->SetPLCRegisterIntValue(6,0);
                                                                                    mir100_ptr_->SetPLCRegisterIntValue(5,3);
                                                                                }

                                                                                // update PLC_006
                                                                                PLC_006 = mir100_ptr_->GetPLCRegisterIntValue(6);

                                                                                if(PLC_006 == 4)
                                                                                {
                                                                                    rmove_continue_flag = false;

                                                                                    // wait for thread to finish
                                                                                    LOG(INFO) << "wait for th_rmove_ForceNode_ to finish...";
                                                                                    while(this->get_rmove_start_flag() != false)
                                                                                    {
                                                                                        /// notice arm to stop force node
                                                                                        tm5.SetRMoveForceFlag(0);

                                                                                        sleep.ms(500);
                                                                                    }
                                                                                    LOG(INFO) << "th_rmove_ForceNode_ finished.";

                                                                                    /// return standby position
                                                                                    tm5.ArmTask("Move_to standby_p1");
                                                                                    tm5.ArmTask("Move_to standby_p0");
                                                                                    tm5.ArmTask("Post arm_back_to_safety");

                                                                                    /// STOP The RMove Mission
                                                                                    mir100_ptr_->SetPLCRegisterIntValue(6,0);

                                                                                    /// Reset PLC registers
                                                                                    mir100_ptr_->SetPLCRegisterIntValue(7,0);

                                                                                    mir100_ptr_->SetPLCRegisterIntValue(15,0);
                                                                                }
                                                                            }
                                                                            break;
                                                                        }
                                                                    }
                                                                }
                                                                else
                                                                {
                                                                    LOG(INFO) << "Failed at RMove Start. Cannot set PLC 015 = 1? Please Check.";

                                                                    /// STOP The RMove Mission
                                                                    mir100_ptr_->Pause();
                                                                    mir100_ptr_->SetPLCRegisterIntValue(6,0);
                                                                    mir100_ptr_->SetPLCRegisterIntValue(5,3);
                                                                }
                                                            }
                                                        }
                                                    }
                                                    else
                                                    {
                                                        LOG(INFO) << "Failed at RMove Start. Cannot set PLC 015 = 1? Please Check.";

                                                        /// STOP The RMove Mission
                                                        mir100_ptr_->Pause();
                                                        mir100_ptr_->SetPLCRegisterIntValue(6,0);
                                                        mir100_ptr_->SetPLCRegisterIntValue(5,3);

                                                    }

                                                    /// II: Check the Robot Status & RMove result
                                                    /// II.1 Check Arm Status First.
                                                    if( nw_status_ptr_->arm_mission_status == data::common::MissionStatus::Error ||
                                                        nw_status_ptr_->arm_mission_status == data::common::MissionStatus::EStop)
                                                    {
                                                        LOG(INFO) << "RMove: Arm has Error! Please Check!";

                                                        // 1. Stop The Ugv for safety concern.
                                                        mir100_ptr_->Pause();
                                                        // 2. Stop the whole mission.
                                                        mir100_ptr_->SetPLCRegisterIntValue(4,3);
                                                        // 3. Break the RMove Mission.
                                                        mir100_ptr_->SetPLCRegisterIntValue(6,0);
                                                        // 4. RMove Mission Failed.
                                                        mir100_ptr_->SetPLCRegisterIntValue(5,3);

                                                        break;
                                                    }
                                                    /// II.2 RMove result
                                                    if(mir100_ptr_->GetPLCRegisterIntValue(5) == 3 )
                                                    {
                                                        LOG(INFO) << "RMove Result: Mission Failed! Please Check!";
                                                        continue;
                                                    }
                                                }

                                                break;
                                            }
                                        }

                                        sleep.ms(200);
                                    }

                                    ///\ (4) Finish Current Arm Mission
                                    ///\    (Done Above)a. For Normal orders: Return 'Safety Position'.
                                    ///\    b. For Last order: Place the tool and then return 'Home Position'

                                    // b.
                                    if(cur_order == cur_last_valid_order_)
                                    {
                                        // back to home first
                                        tm5.ArmTask("Post arm_safety_to_home");

                                        // remove pad if its necessary
                                        if (cur_task_mode_ == data::arm::TaskMode::Mopping)
                                        {
                                            if (cur_tool_angle_ == data::arm::ToolAngle::FortyFive)
                                            {
                                                tm5.ArmTask("Post tool_angle_0");
                                            }

                                            if(tm5.get_remove_tool_flag())
                                            {
                                                this->ArmRemovePad(cur_job_id);

                                                // place the tool
                                                this->ArmPlaceTool(cur_task_mode_);
                                            }

                                            cur_tool_angle_ = data::arm::ToolAngle::Zero;
                                        }

                                        // remove uvc tool if it's the last job
                                        if (cur_task_mode_ == data::arm::TaskMode::UVCScanning)
                                        {
                                            if(tm5.get_remove_tool_flag())
                                            {
                                                // place the tool
                                                this->ArmPlaceTool(cur_task_mode_);
                                            }
                                        }
                                    }

                                    /// 4.1.2 Check the execution result of all arm_mission_configs
                                    ///    a. if anything wrong, record and then break the loop
                                    ///    b. if everything okay, record and then break the loop

                                    bool arm_mission_failed_status =
                                            nw_status_ptr_->arm_mission_status == yf::data::common::MissionStatus::Error ||
                                            nw_status_ptr_->arm_mission_status == yf::data::common::MissionStatus::EStop;

                                    if( nw_status_ptr_->arm_connection_status == data::common::ConnectionStatus::Disconnected ||
                                        arm_mission_failed_status )
                                    {
                                        // a.1 Notify mir100 that arm has error, mir100's mission should be aborted by IPC1.

                                        arm_mission_success_flag    = false;
                                        mission_continue_flag       = false;
                                        mir100_ptr_->SetPLCRegisterIntValue(4,3); // error message

                                        // a.2 Update current arm_config status as as Error.
                                        sql_ptr_->UpdateEachTaskStatus(task_group_id, cur_order, 5);

                                        continue;
                                    }
                                    else
                                    {
                                        // situation b.
                                        // b.1 Notify mir100 that all arm_mission_configs has finished, you're ready to go :)
                                        mir100_ptr_->SetPLCRegisterIntValue(1,0);

                                        // b.2 Update current arm_config status as Finished.
                                        sql_ptr_->UpdateEachTaskStatus(task_group_id, cur_order, 3);
                                    }

                                    if(!arm_sub_mission_success_flag)
                                    {
                                        // no need to break the loop, finish the rest of arm_configs first!

                                        // a.2 Update current arm_config status as as Error.
                                        sql_ptr_->UpdateEachTaskStatus(task_group_id, cur_order, 5);
                                    }
                                }
                                else
                                {
                                    /// 4.2 False

                                    LOG(INFO) << "Mission failed: ugv failed to set plc001=1 ? Arm needs plc001==1 to execute all arm_mission_configs";

                                    arm_mission_success_flag = false;
                                    mission_continue_flag = false;

                                    /// 4.2.1 Update current arm_config status as Error.
                                    sql_ptr_->UpdateEachTaskStatus(task_group_id, cur_order, 5);

                                    break;
                                }

                                /// 5. Check if this is the Last order, if true, break the loop.
                                if(cur_order == cur_mission_num_)
                                {
                                    sleep.ms(1000);
                                    /// ipc1 loop
                                    // set arm_mission_success_flag

                                    arm_mission_success_flag = true;

                                    // set ugv_mission_success_flag
                                    if(mir100_ptr_->GetPLCRegisterIntValue(4) == 2)
                                    {
                                        ugv_mission_success_flag    = true;
                                        mission_continue_flag       = false;
                                    }
                                }

                                break;
                            }
                        }
                    }
                    else
                    {
                        LOG(INFO) << "Mission failed: ugv cannot set plc003=1 ? arm must wait for plc003 == 1 to start configuring";

                        arm_mission_success_flag = false;
                        mission_continue_flag = false;
                        break;
                    }
                }
                else
                {
                    int plc_004_value = mir100_ptr_->GetPLCRegisterIntValue(4);

                    switch(plc_004_value)
                    {
                        case 2:
                        {
                            LOG(INFO) << "ugv mission finished.";

                            ugv_mission_success_flag = true;
                            mission_continue_flag = false;
                            break;
                        }
                        case 3:
                        {
                            LOG(INFO) << "ugv mission failed.";

                            ugv_mission_success_flag = false;
                            mission_continue_flag = false;
                            break;
                        }
                    }


                    break;
                }
            }
        }
    }
    else
    {
        LOG(INFO) << "mission failed at initial status check.";
    }

    if(arm_mission_success_flag == true && ugv_mission_success_flag == true)
    {
        // check all statuses of the task_group_id == 3
        if(sql_ptr_->CheckFailedTaskNo(task_group_id) == 0)
        {
            nw_status_ptr_->cur_job_success_flag = true;

            mir100_ptr_->SetPLCRegisterIntValue(4,0);
        }
        else
        {
            // something is wrong.
            nw_status_ptr_->cur_job_success_flag = false;

            mir100_ptr_->SetPLCRegisterIntValue(4,3);
        }
    }
    else
    {
        // something is wrong.
        nw_status_ptr_->cur_job_success_flag = false;
    }

    mir100_ptr_->Pause();

}
