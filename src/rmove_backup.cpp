//
// Created by NW_W0 on 10/12/2021.
//

int main()
{
    /// original
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

#if 1 /// disable for demo
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
                if( min > 5 )
                {
                    LOG(INFO) << "RMove: already last 5 min. Timeout!";

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

                // ugv total_move_time:  3
                // ugv cur_move_time: 1,2,3..

            }
            break;
        }
    }

    /// new method
    switch (PLC_006)
    {
        case 1:
        {
            if(n == 0)
            {
                /// a. move to standby_position
                // a.1 move to standby_position
                auto standby_point = arm_mission_configs[n].standby_position;
                std::string standby_point_str = this->ArmGetPointStr(standby_point);
                tm5.ArmTask("Set standby_p0 = "+standby_point_str);
                tm5.ArmTask("Move_to standby_p0");

                // a.2. check&set tool_angle
//                                                                        this->ArmSetToolAngle(cur_task_mode_,arm_mission_configs[n].tool_angle);

#if 1 /// disable for demo
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

                // 2.1 sub_standby_position
                auto sub_standby_point = arm_mission_configs[n].sub_standby_position;
                std::string sub_standby_point_str = this->ArmGetPointStr(sub_standby_point);
                tm5.ArmTask("Set standby_p1 = "+ sub_standby_point_str);
                tm5.ArmTask("Move_to standby_p1");

                /// d. set PLC 015 = 0. Handover the Mission.
                mir100_ptr_->SetPLCRegisterIntValue(15,0);
            }
            else
            {
                LOG(INFO) << "RMove: No Deviation!";

                mir100_ptr_->SetPLCRegisterIntValue(6,2);

                amc_deviation_skip_flag = false;
            }

            break;
        }

        case 2:
        {
            if(n != 0)
            {
                tm5.ArmTask("Post tool_angle_45");

                // 2.1. tm_vision_task
                switch (arm_mission_configs[n].vision_type)
                {
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
                        /// Check Inheritance_Type
                        switch (arm_mission_configs[n].inheritance_type)
                        {
                            case data::arm::InheritanceType::Null:
                            {
                                // find the TF
                                GetTMatLogistic(arm_mission_configs, n, task_group_id);
                                break;
                            }
                            case data::arm::InheritanceType::Source:
                            {
                                // find the TF
                                GetTMatLogistic(arm_mission_configs, n, task_group_id);

                                // Save Inheritance Result
                                tm5._inheritance_source_id = arm_mission_configs[n].id;

                                tm5._inheritance_vision_success_flag = arm_mission_configs[n].vision_success_flag;
                                tm5._inheritance_TMat = arm_mission_configs[n].TMat;
                                tm5._inheritance_angle_diff = arm_mission_configs[n].angle_diff;

                                break;
                            }
                            case data::arm::InheritanceType::Target:
                            {
                                // grab the previous result.
                                if(arm_mission_configs[n].inheritance_source_id == tm5._inheritance_source_id)
                                {
                                    arm_mission_configs[n].vision_success_flag = tm5._inheritance_vision_success_flag;
                                    arm_mission_configs[n].TMat = tm5._inheritance_TMat;
                                    arm_mission_configs[n].angle_diff = tm5._inheritance_angle_diff;
                                }
                                else
                                {
                                    arm_mission_configs[n].vision_success_flag = 0;
                                }

                                break;
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
                    default: break;
                }
            }

            // 2.2 check&set tool_angle
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
                    real_via_approach_point = tm5.GetRealPointByRS(arm_mission_configs[0].TMat,arm_mission_configs[n].via_approach_pos);
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
                if( min > 5 )
                {
                    LOG(INFO) << "RMove: already last 5 min. Timeout!";

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
                    if(n == arm_mission_configs.size()-1)
                    {
                        tm5.ArmTask("Move_to standby_p1");
                        tm5.ArmTask("Move_to standby_p0");
                        tm5.ArmTask("Post arm_back_to_safety");
                    } else
                    {
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
                    }

                    /// STOP The RMove Mission
                    mir100_ptr_->SetPLCRegisterIntValue(6,0);

                    /// Reset PLC registers
                    mir100_ptr_->SetPLCRegisterIntValue(7,0);

                    mir100_ptr_->SetPLCRegisterIntValue(15,0);
                }

                // ugv total_move_time:  3
                // ugv cur_move_time: 1,2,3..

            }
            break;
        }
    }
}