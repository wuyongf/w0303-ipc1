#pragma once
// STL
#include <thread>
// GLOG
#include <glog/logging.h>
// Net
#include "net.h"
#include "net_w0303_server.h"
#include "net_w0303_client.h"
// Database
#include "sql.h"
// Status & DataTypes
#include "nw_status.h"
// Arm
#include "arm.h"
// UGV
#include "ugv.h"
// IPC2
#include "ipc2.h"

namespace yf
{
    namespace sys
    {
        class nw_sys
        {
        public: /// Sys Basic Method

            // Constructor & Destructor
            nw_sys(const int& ipc_port);
            virtual ~nw_sys(){}

            // Startup the system
            void Start();

            // Close the system
            void Close();

        public: /// Two Main Functions

            void thread_WaitSchedules();

            void thread_DoSchedules();

            void thread_IPCServerStartup(std::shared_ptr<IPCServer>& server_ptr, bool& server_flag);

        public:

            void WaitSchedulesInitialCheck();

        public: /// Arm Methods: based on tm5.ArmTask();

            void ArmPickTool (const yf::data::arm::TaskMode& task_mode);
            void ArmPlaceTool(const yf::data::arm::TaskMode& task_mode);

            void ArmPickPad();
            void ArmRemovePad();
            void ArmAbsorbWater();
            int  ArmGetAbsorbType();

            void ArmSetOperationArea(const yf::data::arm::OperationArea& operation_area);
            void ArmSetToolAngle(const yf::data::arm::TaskMode& task_mode ,const yf::data::arm::ToolAngle& tool_angle);
            void ArmSetApproachPoint(const yf::data::arm::Point3d& approach_point,const yf::data::arm::ToolAngle& tool_angle);
            void ArmSetViaPoints(const std::deque<yf::data::arm::Point3d>& via_points, const yf::data::arm::ToolAngle& tool_angle);
            void ArmPostViaPoints(const yf::data::arm::TaskMode& task_mode, const yf::data::arm::ToolAngle& tool_angle, const yf::data::arm::ModelType& model_type, const int& arm_mission_config_id);

            std::string ArmGetPointStr(const yf::data::arm::Point3d& point);

        public: /// Ugv Method: based on REST API

            bool WaitForUgvPLCRegisterInt(const int& plc_register, const int& value, const int& timeout_min);
            bool WaitForUgvPLCRegisterFloat(const int& plc_register, const float& value, const int& timeout_min);

        public: /// Methods for two main functions

            void DoJobs(const int& cur_schedule_id);
            void DoTasks(const int& cur_job_id, const int& next_job_id);

            void DoJobUgvBackToChargingStation();
            void DoJobArmBackToHomePos();

            void JobsFilter(std::deque<int>& q_ids);

            void UpdateDbScheduleBeforeTask (const int& cur_schedule_id);
            void UpdateDbScheduleAfterTask (const int& cur_schedule_id);

            void thread_WaitForPauseArm();    // if ugv has error, need to pause Arm.
            void thread_WaitForResumeArm();   // if ugv is executing mission and has no error
            void thread_WaitForCancelUgvMission();


            void UpdateDbJobBeforeTask (const int& cur_job_id);
            void UpdateDbJobAfterTask (const int& cur_job_id);

            void UpdateDbTaskBeforeTask (const int& cur_task_id);
            void UpdateDbTaskAfterTask (const int& cur_task_id);

            void UpdateDbDeviceStatusBeforeTask (const int& cur_job_id);
            void UpdateDbDeviceStatusAfterTask (const int& cur_job_id);

        protected: /// SQL Related

            void UpdateDbDeviceArmConnectionStatus();
            void UpdateDbDeviceArmMissionStatus();

            void UpdateDbCurTaskStatusAndLog();

            void UpdateDbErrorLog();

            void UpdateDbCurJobStatusAndLog();

            void GetSysControlMode();
            void GetScheduleCommand(const int&id);

        private: /// Time Sleep Class

            yf::algorithm::TimeSleep sleep;

        private: /// SYS basic properties

            // shared_ptr: nw_status
            std::shared_ptr<yf::status::nw_status> nw_status_ptr_;

            // shared_ptr: sql
            std::shared_ptr<yf::sql::sql_server> sql_ptr_;

            // arm
            yf::arm::tm tm5;

            // shared_ptr: ugv
            std::shared_ptr<yf::ugv::mir> mir100_ptr_;

            // ipc2
            yf::ipc2::raspberry pi4;

            // Schedule
            // Data Structure:
            // Schedule - Job(each model & a task_mode) - Task(each ugv_mission_config_id & arm_config)
            yf::data::common::MissionStatus db_cur_schedule_status_;
            yf::data::common::MissionStatus db_cur_job_status_;
            yf::data::common::MissionStatus db_cur_task_status_;

            // ipc server
            std::shared_ptr<IPCServer> ipc_server_ptr_;

            // flow control variables
            //
            bool        ipc_server_flag_ = true;
            std::thread th_ipc_server_;

            std::thread th_wait_schedules_;

            std::thread th_do_schedules_;


        private: /// Web Status Manager

            // properties
            bool web_status_flag_ = false;

            // methods
            void thread_WebStatusManager();

            void thread_Web_UgvBatteryPercentage(bool& web_status_flag,
                                                 const int& sleep_duration);

            void thread_Web_UgvCurPosition(const bool& web_status_flag,
                                           const int& sleep_duration);

        private: /// threads handle for Web Status Manager

            std::thread th_web_status_manager_;
            std::thread th_web_ugv_battery_;
            std::thread th_web_ugv_position_;


        private: /// Overall Control --- thread wait_schedules and do_schedules

            // schedule_flag
            bool schedule_flag_ = true;

            ///wait schedule
            //
            // (1) control flag
            bool wait_schedule_flag = true;
            // (2) mutex lock
            std::mutex mux_Blocking_wait_schedule;
            std::condition_variable cv_Blocking_wait_schedule;

            /// do schedule
            //
            // (1) control flag..
            bool do_schedule_flag = false;
            // (2) mutex lock
            std::mutex mux_Blocking_do_schedule;
            std::condition_variable cv_Blocking_do_schedule;

            std::deque<int> q_schedule_ids;
            int schedule_number = 0;

            std::deque<int> q_job_ids;
            int job_number = 0;

            std::deque<int> q_job_mopping_ids;
            std::deque<int> q_job_uvc_scanning_ids;

            std::deque<int> q_task_ids;
            int task_number = 0;

            std::deque<int>  all_available_schedule_ids;

        private:
            /// for current job.
            //
            // db info
            int cur_model_config_id_;

            int cur_mission_num_;

            std::vector<int> cur_valid_result_queue_;
            int cur_first_valid_order_;
            int cur_last_valid_order_;
            std::vector<int> cur_valid_indexes_;

            yf::data::arm::TaskMode     cur_task_mode_;
            yf::data::arm::ModelType    cur_model_type_;

            yf::data::arm::OperationArea cur_operation_area_;
            yf::data::arm::OperationArea last_operation_area_;

            yf::data::arm::ToolAngle cur_tool_angle_ = yf::data::arm::ToolAngle::Zero;

            // for transformation.
            bool cur_find_landmark_flag_ = false;

            yf::data::arm::Point3d real_lm_pos_;

            yf::data::arm::TransMatrixInfo trans_;

            // REST info
            std::string cur_mission_guid_;
        };
    }
}

//@@
// 0. This is a constructor
// 1. Establish a network server: create shared ptr: ipc_server_ptr
// 2. Assign ipc port for TM network connection.
yf::sys::nw_sys::nw_sys(const int &ipc_port)
{
    ipc_server_ptr_ = std::make_shared<IPCServer>(ipc_port);    //e.g. ipc_port: 12345
}

void yf::sys::nw_sys::Start()
{
    /// 0. Initialization
    //
    //  0.1 nw_status
    nw_status_ptr_ = std::make_shared<yf::status::nw_status>();
    //  0.2 sql
    sql_ptr_       = std::make_shared<yf::sql::sql_server>("SQL Server","192.168.7.127","NW_mobile_robot_sys","sa","NWcadcam2021");
    //  0.3 ipc_server
    //  establish server and keep updating, keep waiting for devices connection.
    th_ipc_server_ = std::thread(&nw_sys::thread_IPCServerStartup, this, std::ref(ipc_server_ptr_), std::ref(ipc_server_flag_));

    /// 1. Initialization --- arm
    //
    // 1.1 startup tm5 control module and pass <1> ipc_server_ptr, <2> nw_status_ptr <3> sql_ptr
    tm5.Start(ipc_server_ptr_,nw_status_ptr_,sql_ptr_);

    // 1.2 block the program, wait for arm connection
    // notify the database.
    sql_ptr_->UpdateSysAdvice(8);
    tm5.WaitForConnection();

    // 1.3 update the connection status to sys_status & database ;
    tm5.GetConnectionStatus();

    /// 2. Initialization --- ugv
    //
    mir100_ptr_ = std::make_shared<yf::ugv::mir>();
    mir100_ptr_->Start("192.168.7.34", nw_status_ptr_, sql_ptr_);

    /// TIME
    sleep.ms(5000);

    /// 3. Web Status Manager
    th_web_status_manager_ = std::thread(&nw_sys::thread_WebStatusManager, this);

    /// 4. Do Schedule and Wait Schedule
    th_do_schedules_ = std::thread(&nw_sys::thread_DoSchedules, this);

    /// TIME
    sleep.ms(500);

    th_wait_schedules_ = std::thread(&nw_sys::thread_WaitSchedules, this);
}

void yf::sys::nw_sys::Close()
{
    // close thread do_schedule, wait_schedule
    schedule_flag_ = false;

    schedule_number = -999;        // for thread_WaitSchedules to join.
                                   // search:  For sys to shut down
    wait_schedule_flag = false;

    // close arm
    tm5.Close();

    th_wait_schedules_.join();
    th_do_schedules_.join();

    // close thread ipc_server
    // todo: 2021/03/03
    //  (Undone)Fine tune the process.
    //  Problem:
    //  ipc_sever doesn't know whether the connection is close.
    //  needs: 1. establish a new connection. or
    //         2. pause the arm
    //  to let th_ipc_server_ to join.
    ipc_server_flag_ = false;
    th_ipc_server_.join();
}

void yf::sys::nw_sys::thread_IPCServerStartup(std::shared_ptr<IPCServer>& server_ptr, bool& server_flag)
{
    server_ptr->Start();

    while (server_flag)
    {
        server_ptr->Update(-1, true);
    }

    server_ptr->Stop();

    return;
}

void yf::sys::nw_sys::thread_WaitSchedules()
{
    while (schedule_flag_)
    {
        while (wait_schedule_flag)
        {
            LOG(INFO) << "[thread_WaitSchedules]: Start!";

//            LOG(INFO) << "[thread_WaitSchedules]: 1. --- Initial Check [Start]";
//            WaitSchedulesInitialCheck();
//            LOG(INFO) << "[thread_WaitSchedules]: 1. --- Initial Check [Complete]";

            #if 0
            bool initial_check_flag = true;

            // TODO: (0) Initial Check Loop.
            while (initial_check_flag)
            {

                LOG(INFO) << "1. update sys control mode";

                // 1. update sys control mode
                //
                GetSysControlMode();

                LOG(INFO) << "2. updated all devices status";

                // 2. updated all devices status
                //
                //  2.1 for arm
                tm5.UpdateArmCurMissionStatus();

                LOG(INFO) << "3. check function";
                // 3. check function
                //
                //  3.0 if recovery mode, try to recover all devices.
                //
                ///  Check whether arm connected or not
                ///  3.0.1 if disconnected, ask for recovery mode
                if( nw_status_ptr_->arm_connection_status == data::common::ConnectionStatus::Disconnected)
                {
                    LOG(INFO) << "Arm disconnected...";
                    LOG(INFO) << "Please switch to Recovery Mode...";

                    if(nw_status_ptr_->sys_control_mode_ == data::common::SystemMode::Recovery)
                    {
                        ///   3.0.1 Arm
                        tm5.WaitForConnection();

                        // Get Status and update to SQL
                        tm5.GetConnectionStatus();
                        tm5.GetMissionStatus();
                    }
                }

                ///  3.0.2 if connected but mission_status == Error
                if ( nw_status_ptr_->arm_connection_status == data::common::ConnectionStatus::Connected &&
                     nw_status_ptr_->arm_mission_status == data::common::MissionStatus::Error)
                {
                    LOG(INFO) << "Please stop robotic arm project...";
                }
                
                //  3.1 if manual mode, wait until auto mode
                if( nw_status_ptr_->sys_control_mode_ == data::common::SystemMode::Manual ||
                    nw_status_ptr_->sys_control_mode_ == data::common::SystemMode::ManualSetting)
                {
                    // wait for auto mode.
                    while (nw_status_ptr_->sys_control_mode_ != data::common::SystemMode::Auto)
                    {
                        LOG(INFO) << "sys is not in auto mode, keep waiting...";

                        // update sys control mode
                        GetSysControlMode();

                        ///TIME
                        // sleep 2s and then keep checking
                        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                    }
                }
                //
                LOG(INFO) << "3.1 check function --- [Sys_Control_Mode]: Auto";



                //  3.2 wait for all devices idle, both connection status and mission status.
                //
                LOG(INFO) << "3.2 check function --- wait for all devices idle";

                //  3.2.1 For Arm
                // if not normal
                //
                if (nw_status_ptr_->arm_connection_status != data::common::ConnectionStatus::Connected ||
                    nw_status_ptr_->arm_mission_status != data::common::MissionStatus::Idle)
                {
                    ///TIME
                    // do nothing , wait 2s and then check once more.
                    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                }
                else
                {
                    LOG(INFO) << "3.3 check function --- check all device initialized";
                    // 3.3 if all device initialized?
                    //
                    // 3.3.1 check arm initial position.

                    // default success for now.
                    initial_check_flag = false;
                }

                LOG(INFO) << "4. Update Status to SQL";
                // 4.1 For Arm
                //
                UpdateDbDeviceArmConnectionStatus();
                UpdateDbDeviceArmMissionStatus();

            }
            #endif

            LOG(INFO) << "[thread_WaitSchedules]: stage 1 --- Wait for Database Available Schedules";

            /// prerequisite for front-end. If Devices connection are not all IDLE. Do not Publish Schedule.
            //
            while (schedule_number == 0 )
            {
                // (1) Keep checking Sys Devices Status
                LOG(INFO) << "[thread_WaitSchedules]: 1. --- Initial Devices Status Check [Start]";
                WaitSchedulesInitialCheck();
                LOG(INFO) << "[thread_WaitSchedules]: 1. --- Initial Devices Status Check [Complete]";

                // (2) retrieve schedule number from database...
                LOG(INFO) << "[thread_wait_schedules]: wait for incoming schedules...";
                q_schedule_ids   = sql_ptr_->GetAvailableScheuldesId();
                schedule_number  = q_schedule_ids.size();

                ///TIME
                sleep.ms(500);
            }

            schedule_number = 0;
            wait_schedule_flag = false;

            // notify thread: wait schedule
            do_schedule_flag = true;
            std::unique_lock<std::mutex> ul(mux_Blocking_do_schedule);
            cv_Blocking_do_schedule.notify_one();
            LOG(INFO) << "[thread_WaitSchedules]: notify thread_DoSchedules";

            // For sys to shut down
            if(schedule_flag_ == false)
            {
                return;
            }
        }

        ///TIME
        sleep.ms(50);

        LOG(INFO) << "[thread_WaitSchedules]: lock!";
        std::unique_lock<std::mutex> ul_wait(mux_Blocking_wait_schedule);
        cv_Blocking_wait_schedule.wait(ul_wait);
        LOG(INFO) << "[thread_WaitSchedules]: unlock!";

    }

    LOG(INFO) << "schedule flag is false, thread: wait schedules should be shut down...";
}

void yf::sys::nw_sys::thread_DoSchedules()
{
    while (schedule_flag_)
    {
        // Default status is locking....
        while (do_schedule_flag == false)
        {
            LOG(INFO) << "[thread_DoSchedules]: lock!";
            std::unique_lock<std::mutex> ul(mux_Blocking_do_schedule);
            cv_Blocking_do_schedule.wait(ul);
            LOG(INFO) << "[thread_DoSchedules]: unlock!";
        }

        // For sys to shut down
        // (1) Check whether the sys is shutting down.
        if(schedule_flag_ == false)
        {
            return;
        }

        // if there is any schedule
        //
        while (q_schedule_ids.size() != 0)
        {
            // (1) Initial check
            //
            ///TIME
            // All devices should be idle. Otherwise we should keep checking for 3 minutes.. and then alter user.
            auto time_now = sql_ptr_->TimeNow();
            auto time_countdown = sql_ptr_->CountdownTime(time_now, 3);

            while (nw_status_ptr_->arm_mission_status != yf::data::common::MissionStatus::Idle)
            {
                if(!sql_ptr_->isFutureTime(time_countdown, time_now))
                {
                    LOG(INFO) << "[thread_DoSchedules]: The Arm has not responded for 3 minutes.";
                    LOG(INFO) << "[thread_DoSchedules]: The Schedule failed at the initial check stage.";

                    break;
                }

                time_now = sql_ptr_->TimeNow();

                tm5.UpdateArmCurMissionStatus();
                LOG(INFO) << "[thread_DoSchedules]: schedule initial check: wait for arm is idle";

                ///TIME
                sleep.ms(2000);
            }

            // (2) Do Each Schedule

            if(nw_status_ptr_->arm_mission_status == yf::data::common::MissionStatus::Idle)
            {
                auto cur_schedule_id = q_schedule_ids.front();
                q_schedule_ids.pop_front();

                // get current id
                nw_status_ptr_->db_cur_schedule_id = cur_schedule_id;

                // get schedule command
                this->GetScheduleCommand(cur_schedule_id);

                // get execute time..
                sql_ptr_->GetScheduleExecTime(cur_schedule_id);
                std::string execute_time = sql_ptr_->get_execute_time();
                // wait for execute time...
                sql_ptr_->WaitForExecute(execute_time);

                /// Update Mission Status to database
                UpdateDbScheduleBeforeTask(cur_schedule_id);

                /// Execute the schedule! Based on Schedule Command!
                switch (nw_status_ptr_->db_cur_schedule_command_)
                {
                    case data::schedule::ScheduleCommand::CleaningJob:
                    {
                        DoJobs(cur_schedule_id);
                        break;
                    }
                    case data::schedule::ScheduleCommand::UgvBackToChargingStation:
                    {
                        DoJobUgvBackToChargingStation();
                        break;
                    }
                    case data::schedule::ScheduleCommand::ArmBackToHomePos:
                    {
                        DoJobArmBackToHomePos();
                        break;
                    }
                }

                /// Update the schedule status to database!
                UpdateDbScheduleAfterTask(cur_schedule_id);
            }
            else
            {
                LOG(INFO) << "[thread_DoSchedules]: abort current schedules...";
                q_schedule_ids.clear();
            }

            // TODO: (3)
            //  a. break the shcedule loop or not?
            //  b. (done above) Update Schedule Status after job. Record status and update to SQL Server

            // For Arm

            // (0) break the task loop or not?

            bool arm_mission_failed_status = nw_status_ptr_->arm_mission_status == yf::data::common::MissionStatus::Error ||
                                             nw_status_ptr_->arm_mission_status == yf::data::common::MissionStatus::EStop;

            if( nw_status_ptr_->arm_connection_status == data::common::ConnectionStatus::Disconnected || arm_mission_failed_status )
            {
                LOG(INFO) << "[thread_DoSchedules]: abort current schedules...";
                q_schedule_ids.clear();
            }
        }

        // No more schedules
        //
        // if done successfully
        if(nw_status_ptr_->arm_mission_status == data::common::MissionStatus::Idle)
        {
            LOG(INFO) << "[thread_DoSchedules]: All schedules have been done!";
        }
        else
        {
            LOG(INFO) << "[thread_DoSchedules]: Cancel the rest of schedules...";
        }

        // Anyway, lock thread_DoSchedules and then keep waiting
        do_schedule_flag = false;

        // trigger thread_WaitSchedules
        LOG(INFO) << "[thread_DoSchedules]: notify thread_WaitSchedules";
        wait_schedule_flag = true;
        ///TIME
        sleep.ms(50);
        std::unique_lock<std::mutex> ul_wait(mux_Blocking_wait_schedule);
        cv_Blocking_wait_schedule.notify_one();

        // record the end time
        // update the schedule status to database
    }
}


void yf::sys::nw_sys::DoJobs(const int &cur_schedule_id)
{
    LOG(INFO) << "Do Jobs...";

    q_job_ids = sql_ptr_->GetJobsId(cur_schedule_id);

    /// Job Filter, do mopping first!
//    this->JobsFilter(q_job_ids);

    /// Ugv needs to change the map first!
    mir100_ptr_->ClearErrorState();

    mir100_ptr_->ChangeMapByDBMapStatus();

    sleep.sec(5);

    if(mir100_ptr_->WaitForModeId(7,1))
    {
        mir100_ptr_->ChangeInitPositionByDBMapStatus();
    }

    job_number = q_job_ids.size();

    // if there is any Job
    //
    while (q_job_ids.size() != 0)
    {
        int cur_job_id;
        int next_job_id;

        cur_job_id = q_job_ids.front();
        q_job_ids.pop_front();

        /// Find Next Job Id
        if(q_job_ids.size() != 0)
        {
            next_job_id = q_job_ids.front();
        }
        else
        {
            next_job_id = -1;
        }

        nw_status_ptr_->db_cur_job_id = cur_job_id;

        // record the job status
        // TODO: (1) Update Job Log
        sql_ptr_->UpdateJobData(cur_job_id, 2);
        sql_ptr_->UpdateJobLog(cur_job_id, 2);

        // TODO: (2) Execute the job!
        DoTasks(cur_job_id,next_job_id);

        // TODO: (3)
        //  a. Break the loop or not?
        //  b. Record Each Job STATUS.
        bool arm_mission_failed_status = nw_status_ptr_->arm_mission_status == yf::data::common::MissionStatus::Error ||
                                         nw_status_ptr_->arm_mission_status == yf::data::common::MissionStatus::EStop;

        if( nw_status_ptr_->arm_connection_status == data::common::ConnectionStatus::Disconnected || arm_mission_failed_status )
        {
            LOG(INFO) << "abort current jobs...";
            q_job_ids.clear();

            UpdateDbErrorLog();
        }

        // record and update current job status
        LOG(INFO) << "update current jobs status and log...";
        UpdateDbCurJobStatusAndLog();                  // todo: what about cancel Status?

    }

    // No more jobs
    //
    // Cout the result

    if(nw_status_ptr_->arm_mission_status == data::common::MissionStatus::Idle)
    {
        LOG(INFO) << "All Jobs are done";

        auto time_now = sql_ptr_->TimeNow();

        std::cout << "no job now!" << std::endl;
        std::cout << "current time: " << time_now << std::endl;
    }
    else
    {
        LOG(INFO) << "Cancel the rest of jobs...";
    }
}

void yf::sys::nw_sys::DoTasks(const int &cur_job_id, const int& next_job_id)
{
    /// 0. Initialization
    //
    bool mission_success_flag = false;
    bool ugv_mission_success_flag = false;
    bool arm_mission_success_flag = false;

    bool arm_sub_mission_success_flag = false;

    /// 1. Initialization
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


    /// 2. Assign Ugv Mission
    //
    //  2.1. Ugv: post a new mission via REST
    mir100_ptr_->PostMission(cur_model_config_id_);
    //  2.2. Ugv: post actions via REST
    mir100_ptr_->PostActions(cur_model_config_id_);
    //  2.3  Ugv: get current mission_guid
    cur_mission_guid_ = mir100_ptr_->GetCurMissionGUID();

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
        ///TIME
        sleep.ms(200);
        mir100_ptr_->Play();

        /// \brief
        ///
        /// while ugv mission has not finished, keep assigning arm mission config and executing arm mission.
        /// for ugv mission finish info, please refer to PLC Register Assignment.
        bool mission_continue_flag = true;

        while (mission_continue_flag)
        {
            // while mir's mission has not finished
            // if plc4 ==2 || plc4 == 3 --> break
            // if plc4 !=2 && plc4 != 3 ===> continue
            while(  mir100_ptr_->GetPLCRegisterIntValue(4) != 2 &&
                    mir100_ptr_->GetPLCRegisterIntValue(4) != 3)
            {
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

                if(ugv_mission_continue_flag == true)
                {
                    /// stage 0: check arm config is valid or not
                    /// stage 1: if it is valid, configuring arm mission
                    /// stage 2: execute arm mission

                    bool arm_config_stage_success_flag = false;
                    int last_valid_order;

                    arm_wait_plc003_success_flag = this->WaitForUgvPLCRegisterInt(3,1,5);

                    ///TIME
                    sleep.ms(500);

                    if(arm_wait_plc003_success_flag == true)
                    {
                        /// get current order
                        auto cur_order = mir100_ptr_->GetPLCRegisterIntValue(2);

                        /// check arm config is valid or not

                        int  cur_arm_config_id = sql_ptr_->GetArmConfigId(cur_model_config_id_, cur_order);
                        int  is_valid = sql_ptr_->GetArmConfigIsValid(cur_arm_config_id);
                        LOG(INFO) << "current arm config is_valid:" << is_valid;

                        switch(is_valid)
                        {
                            case 0: // invalid
                            {
                                // arm configure stage has finished
                                // notify mir100 and ipc
                                mir100_ptr_->SetPLCRegisterIntValue(3,0);

                                // wait for executing flag/signal.
                                arm_wait_plc001_success_flag = this->WaitForUgvPLCRegisterInt(1,1,5);

                                if(arm_wait_plc001_success_flag == true)
                                {
                                    /// TIME
                                    sleep.ms(200);
                                    mir100_ptr_->SetPLCRegisterIntValue(1,0);
                                }
                                else
                                {
                                    LOG(INFO) << "mission failed at step 2: wait for plc001 == 1";

                                    arm_mission_success_flag = false;
                                    mission_continue_flag = false;
                                    break;
                                }

                                /// Last order
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
                                ///
                                LOG(INFO) << "configure arm mission   [Start!]";
                                auto arm_mission_configs = tm5.ConfigureArmMission(cur_model_config_id_, cur_order);
                                LOG(INFO) << "configure arm mission   [Finished!]";

                                cur_operation_area_ =  arm_mission_configs[0].operation_area;

                                /// find last valid order
                                // input: cur_order, which is the cur_valid_order!
                                /// get_cur_valid_order
                                /// get_last_valid_order
                                if(cur_order == cur_first_valid_order_)
                                {
                                    last_valid_order = cur_order;
                                }
                                else
                                {
                                    std::vector<int>::iterator cur_valid_index_index = std::find(cur_valid_indexes_.begin(), cur_valid_indexes_.end(), cur_order-1);

                                    auto last_valid_order_index = cur_valid_index_index - cur_valid_indexes_.begin() - 1;

                                    last_valid_order = cur_valid_indexes_[last_valid_order_index] + 1;
                                }

                                /// get last operation area
                                int  last_arm_config_id = sql_ptr_->GetArmConfigId(cur_model_config_id_, last_valid_order);
                                auto last_operation_area = tm5.GetOperationArea(last_arm_config_id);

                                // arm configure stage has finished
                                // notify mir100 and ipc
                                mir100_ptr_->SetPLCRegisterIntValue(3,0);
                                arm_config_stage_success_flag = true;

                                // wait for executing flag/signal.
                                arm_wait_plc001_success_flag = this->WaitForUgvPLCRegisterInt(1,1,5);

                                if(arm_wait_plc001_success_flag == true)
                                {
                                    //todo:
                                    // Start Monitoring Arm Status!!!
                                    // 1. Wait for jumping out of the loop, done
                                    // 2. Wait for pausing MiR mission

                                    /// b.3.2 start executing arm mission

                                    /// First order, pick the pad
                                    if(cur_order == cur_first_valid_order_)
                                    {
                                        cur_tool_angle_ = data::arm::ToolAngle::Zero;
#if 1 /// Disable For Testing
                                        this->ArmPickTool(cur_task_mode_);

                                        sleep.ms(200);

                                        if(cur_task_mode_ == data::arm::TaskMode::Mopping)
                                        {
                                            this->ArmPickPad();

                                            sleep.ms(200);

                                            this->ArmAbsorbWater();
                                        }
#endif
                                    }

                                    /// For each order, move to safety position first.
                                    //
                                    if(cur_order == cur_first_valid_order_ )
                                    {
                                        this->ArmSetOperationArea(cur_operation_area_);
                                        tm5.ArmTask("Post arm_home_to_safety");
                                    }
                                    else
                                    {
                                        // if cur_operation_area is not equal to last one, move to safety position first.
                                        if(cur_operation_area_ != last_operation_area)
                                        {
                                            tm5.ArmTask("Post arm_safety_to_front_p1");

                                            this->ArmSetOperationArea(cur_operation_area_);
                                            tm5.ArmTask("Post arm_home_to_safety");
                                        }
                                    }

                                    /// Loop all the arm_mission_configs
                                    for (int n = 0; n < arm_mission_configs.size(); n++)
                                    {
                                        // 1. move to standby_position
                                        //  1.1 get standby_point_str
                                        auto standby_point = arm_mission_configs[n].standby_position;
                                        std::string standby_point_str = this->ArmGetPointStr(standby_point);
                                        //  1.2 set standby_point
                                        tm5.ArmTask("Set standby_p0 = "+standby_point_str);
                                        //  1.3 move to standby_point
                                        tm5.ArmTask("Move_to standby_p0");

                                        // 3. check&set tool_angle
                                        this->ArmSetToolAngle(cur_task_mode_,arm_mission_configs[n].tool_angle);

                                        // 2. check landmark_or_not?
                                        if(arm_mission_configs[n].landmark_flag == true)
                                        {
                                            // 2.1 init_lm_vision_position.
                                            //  2.1.1 retrieve init_lm_vision_position_str
                                            std::string ref_vision_lm_init_position_str = this->ArmGetPointStr(arm_mission_configs[n].ref_vision_lm_init_position);
                                            //  2.1.2 set init_lm_vision_position
                                            tm5.ArmTask("Set vision_lm_init_p0 = " + ref_vision_lm_init_position_str);
                                            //  2.1.3 move_to init_lm_vision_position
                                            tm5.ArmTask("Move_to vision_lm_init_p0");

                                            // 2.2 execute vision_find_landmark
                                            tm5.ArmTask("Post vision_find_landmark");

                                            // 2.3 check find_landmark_flag
                                            tm5.ArmTask("Post get_find_landmark_flag");

                                            if(tm5.GetFindLandmarkFlag() == true)
                                            {
                                                LOG(INFO) << "Find Landmark!";

                                                // 2.4 get real_landmark_pos
                                                tm5.ArmTask("Post get_landmark_pos_str");
                                                real_lm_pos_ = tm5.GetRealLandmarkPos();

                                                // 2.5 post back_to_standby_position.
                                                tm5.ArmTask("Move_to standby_p0");

                                                // 2.6 get TF
                                                // 2.7 calculate the new via_points

                                                std::deque<yf::data::arm::Point3d> real_via_points;

                                                real_via_points = tm5.GetRealViaPoints(arm_mission_configs[n].via_points, arm_mission_configs[n].ref_landmark_pos, real_lm_pos_);

                                                arm_mission_configs[n].via_points.clear();

                                                arm_mission_configs[n].via_points = real_via_points;

                                                // 2.8 calculate the real approach point

                                                yf::data::arm::Point3d real_via_approach_point;

                                                real_via_approach_point = tm5.GetRealPointByLM(arm_mission_configs[n].via_approach_pos, arm_mission_configs[n].ref_landmark_pos, real_lm_pos_);

                                                arm_mission_configs[n].via_approach_pos.x = real_via_approach_point.x;
                                                arm_mission_configs[n].via_approach_pos.y = real_via_approach_point.y;
                                                arm_mission_configs[n].via_approach_pos.z = real_via_approach_point.z;
                                                arm_mission_configs[n].via_approach_pos.rx = real_via_approach_point.rx;
                                                arm_mission_configs[n].via_approach_pos.ry = real_via_approach_point.ry;
                                                arm_mission_configs[n].via_approach_pos.rz = real_via_approach_point.rz;
                                            }
                                            else
                                            {
                                                LOG(INFO) << "Cannot find Landmark! cur_arm_mission_config aborted!!";

                                                // back to standby_point
                                                tm5.ArmTask("Move_to standby_p0");
                                                // back to safety position
                                                tm5.ArmTask("Post arm_back_to_safety");

                                                continue;
                                            }
                                        }



                                        // 4. check motion_type, decide which motion.
                                        // 5. assign n_via_points.
                                        std::string n_via_points_str = std::to_string(arm_mission_configs[n].n_via_points);
                                        tm5.ArmTask("Set n_points = " + n_via_points_str);

                                        // 6. set approach_point
                                        this->ArmSetApproachPoint(arm_mission_configs[n].via_approach_pos, arm_mission_configs[n].tool_angle);

                                        // 7. set via_points
                                        this->ArmSetViaPoints(arm_mission_configs[n].via_points, arm_mission_configs[n].tool_angle);

                                        // 8. post via_points
                                        this->ArmPostViaPoints(cur_task_mode_, arm_mission_configs[n].tool_angle, arm_mission_configs[n].model_type, arm_mission_configs[n].id);

                                        // 9. post return standby_position
                                        tm5.ArmTask("Move_to standby_p0");
                                    }

                                    ///TIME
                                    sleep.ms(200);

                                    // 10. post return safety.
                                    tm5.ArmTask("Post arm_back_to_safety");
                                    ///

                                    ///TIME
                                    sleep.ms(300);

                                    /// Last valid order
                                    if(cur_order == cur_last_valid_order_)
                                    {
                                        /// arm motion

                                        // back to home first
                                        tm5.ArmTask("Post arm_safety_to_home");
#if 1 /// Disable For Testing
                                        // remove pad if its necessary
                                        if (cur_task_mode_ == data::arm::TaskMode::Mopping)
                                        {
                                            if (cur_tool_angle_ == data::arm::ToolAngle::FortyFive)
                                            {
                                                tm5.ArmTask("Post tool_angle_0");
                                            }

                                            this->ArmRemovePad();
                                        }

                                        // place the tool
                                        this->ArmPlaceTool(cur_task_mode_);
#endif
                                        cur_tool_angle_ = data::arm::ToolAngle::Zero;
                                    }

                                    /// For the end of each arm config. Check arm is alright or not
                                    // todo:
                                    //  check if arm is still connected
                                    bool arm_mission_failed_status =
                                            nw_status_ptr_->arm_mission_status == yf::data::common::MissionStatus::Error ||
                                            nw_status_ptr_->arm_mission_status == yf::data::common::MissionStatus::EStop;

                                    if( nw_status_ptr_->arm_connection_status == data::common::ConnectionStatus::Disconnected ||
                                        arm_mission_failed_status )
                                    {
                                        arm_mission_success_flag    = false;
                                        mission_continue_flag       = false;
                                        mir100_ptr_->SetPLCRegisterIntValue(4,3); // error message
                                        continue;
                                    }
                                    else
                                    {
                                        /// info mir100 the arm has finished
                                        mir100_ptr_->SetPLCRegisterIntValue(1,0);
                                    }

                                    /// For Last order -- Need to decide how to break the loop
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
                                }
                                else
                                {
                                    LOG(INFO) << "mission failed at step 2: wait for plc001 == 1";

                                    arm_mission_success_flag = false;
                                    mission_continue_flag = false;
                                    break;
                                }

                                break;
                            }
                        }
                    }
                    else
                    {
                        LOG(INFO) << "mission failed at step 1: wait for plc003 == 1";

                        arm_mission_success_flag = false;
                        mission_continue_flag = false;
                        break;
                    }
                }
                else
                {
                    LOG(INFO) << "ugv mission failed.";

                    ugv_mission_success_flag = false;
                    mission_continue_flag = false;
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
        mission_success_flag = true;

        mir100_ptr_->SetPLCRegisterIntValue(4,0);
    }
    else
    {
        // something is wrong.
        mission_success_flag = false;
    }

    mir100_ptr_->Pause();


#if 0

    // 2. Do Tasks.

    LOG(INFO) << "Do Tasks...";

    q_task_ids = sql_ptr_->GetTasksId(cur_job_id);
    task_number = q_task_ids.size();

    // if there is any Task
    //
    while (q_task_ids.size() != 0)
    {
        LOG(INFO) << "[Do Tasks]: (1) Initial check, all devices should be idle...";

        // TODO: (1) Initial check
        //
        // All devices should be idle. Otherwise we should keep checking for 3 minutes.. and then alter user.
        auto time_now = sql_ptr_->TimeNow();
        auto time_future = sql_ptr_->CountdownTime(time_now, 3);

        while (nw_status_ptr_->arm_mission_status != yf::data::common::MissionStatus::Idle)
        {
            if(!sql_ptr_->isFutureTime(time_future, time_now))
            {
                LOG(INFO) << "The Arm has not responded for 3 minutes.";
                LOG(INFO) << "The Task failed at the initial check stage.";
                //
                // todo: alert the user... No need, stage 3 will do the job.
                break;
            }

            time_now = sql_ptr_->TimeNow();

            tm5.UpdateArmCurMissionStatus();

            LOG(INFO) << "task initial check: wait for arm is idle";
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        }


        LOG(INFO) << "[Do Tasks]: (2) Do each task...";

        // TODO: (2) Do each Task
        // if all devices are idle...
        //
        if(nw_status_ptr_->arm_mission_status == yf::data::common::MissionStatus::Idle)
        {
            auto cur_task_id = q_task_ids.front();

            std::cout << "cur_task_id: "<< cur_task_id << std::endl;

            q_task_ids.pop_front();

            nw_status_ptr_->db_cur_task_id = cur_task_id;

            // TODO: Update Task Status before task, 2 --- in process
            sql_ptr_->UpdateTaskData(cur_task_id, 2);
            sql_ptr_->UpdateTaskLog(cur_task_id, 2);

            // TODO: Execute the task!
            DoTask(cur_task_id);
        }
        else
        {
            LOG(INFO) << "abort current tasks...";
            q_task_ids.clear();
        }

        LOG(INFO) << "[Do Tasks]: (3) break the task loop or not?";

        // TODO: (3)
        //  a. break the task loop or not?
        //  b. Update Task Status after task. Record status and update to SQL Server

        // For Arm

        // (0) break the task loop or not?

        bool arm_mission_failed_status = nw_status_ptr_->arm_mission_status == yf::data::common::MissionStatus::Error ||
                                         nw_status_ptr_->arm_mission_status == yf::data::common::MissionStatus::EStop;

        if( nw_status_ptr_->arm_connection_status == data::common::ConnectionStatus::Disconnected || arm_mission_failed_status )
        {
            LOG(INFO) << "abort current tasks...";
            q_task_ids.clear();
        }

        // (1) record and update arm connection status
        UpdateDbDeviceArmConnectionStatus();

        // (2) record and update arm mission status     // todo:(Log) UpdateDbDeviceArmMissionStatusLogAfterTask();
        UpdateDbDeviceArmMissionStatus();

        // (3) record and update current task status
        UpdateDbCurTaskStatusAndLog();                  // todo: what about cancel Status?

    }

    // todo: (4)
    //  cout the result.
    //
    if(nw_status_ptr_->arm_mission_status == data::common::MissionStatus::Idle)
    {
        // No more jobs
        LOG(INFO) << "All Tasks are done";

        auto time_now = sql_ptr_->TimeNow();

        LOG(INFO) << "no more tasks for job: " << cur_job_id << "now!" << std::endl;
        LOG(INFO) << "current time: " << time_now << std::endl;
    }
    else
    {
        LOG(INFO) << "Cancel the rest of tasks...";
    }

#endif

}

#if 0
void yf::sys::nw_sys::DoTask(const int& cur_task_id)
{
    std::cout << "[cur_task_id]: " << cur_task_id << std::endl;

    // Initialization
    // (1) sql status


    // Check task_mode

    switch (sql_ptr_->GetTaskMode(cur_task_id))
    {
        // task_mode: clean_mopping
        //
        case 1:
        {
            // Get UGV Config, Get all points, sort in order

            // Get Arm Configs, sort in order

            // for loop. go through each agv point (mission_config)

            // for loop. go through each arm config

            break;
        }

        // task_mode: clean_scanning
        case 2:
        {
            break;
        }

        // Task Mode: Arm Command
        case 3:
        {
            if(sql_ptr_->GetTaskCommand(cur_task_id) == 1)
            {
                LOG(INFO) << "Arm Back To Home...";
                ArmTask("Back to home");
            }
            break;
        }

        // Task Mode: Ugv Command
        case 4:
        {
            break;
        }
    }
}
#endif

void yf::sys::nw_sys::UpdateDbScheduleAfterTask(const int& cur_schedule_id)
{

    int schedule_status;

    // todo: fine tune...
    // 2021/02/23: By checking arm_mission_status to decide schedule status...
    //
    switch (nw_status_ptr_->arm_mission_status)
    {
        case yf::data::common::MissionStatus::Error:
        {
            schedule_status = 5;
            break;
        }

        case yf::data::common::MissionStatus::EStop:
        {
            schedule_status = 5;
            break;
        }

        case yf::data::common::MissionStatus::Pause:
        {
            schedule_status = 6;
            break;
        }

        case yf::data::common::MissionStatus::Finish:
        {
            schedule_status = 3;
            break;
        }

        case yf::data::common::MissionStatus::Idle:
        {
            schedule_status = 3;
            break;
        }

        case yf::data::common::MissionStatus::Running:
        {
            schedule_status = 2;
            break;
        }
    }

    sql_ptr_->UpdateScheduleData(cur_schedule_id, schedule_status);
    sql_ptr_->UpdateScheduleLog(cur_schedule_id, schedule_status);

    // todo: (1) Update device_table
    //       (2) Update device_log_table
    //       (3) ...
}

void yf::sys::nw_sys::UpdateDbScheduleBeforeTask(const int &cur_schedule_id)
{

    int schedule_status = 2;

#if 0
    // todo: fine tune...
    // 2021/02/23: By checking arm_mission_status to decide schedule status...
    //
    switch (nw_status_ptr_->arm_mission_status)
    {
        case yf::data::common::MissionStatus::Error:
        {
            schedule_status = 5;
            break;
        }

        case yf::data::common::MissionStatus::EStop:
        {
            schedule_status = 5;
            break;
        }

        case yf::data::common::MissionStatus::Pause:
        {
            schedule_status = 6;
            break;
        }

        case yf::data::common::MissionStatus::Finish:
        {
            schedule_status = 2;
            break;
        }

        case yf::data::common::MissionStatus::Idle:
        {
            schedule_status = 2;
            break;
        }

    }
#endif

    sql_ptr_->UpdateScheduleData(cur_schedule_id, schedule_status);
    sql_ptr_->UpdateScheduleLog(cur_schedule_id, schedule_status);

}

void yf::sys::nw_sys::UpdateDbDeviceArmConnectionStatus()
{
    if(nw_status_ptr_->arm_connection_status == data::common::ConnectionStatus::Connected)
    {
        sql_ptr_->UpdateDeviceConnectionStatus("arm", 1);
    }
    else
    {
        sql_ptr_->UpdateDeviceConnectionStatus("arm", 0);
    }
    return;
}

void yf::sys::nw_sys::UpdateDbDeviceArmMissionStatus()
{
    switch (nw_status_ptr_->arm_mission_status)
    {
        case data::common::MissionStatus::Idle:
        {
            sql_ptr_->UpdateDeviceMissionStatus("arm", 1);
            break;
        }
        case data::common::MissionStatus::Running:
        {
            sql_ptr_->UpdateDeviceMissionStatus("arm", 2);
            break;
        }
        case data::common::MissionStatus::Finish:
        {
            sql_ptr_->UpdateDeviceMissionStatus("arm", 3);
            break;
        }
        case data::common::MissionStatus::Pause:
        {
            sql_ptr_->UpdateDeviceMissionStatus("arm", 4);
            break;
        }
        case data::common::MissionStatus::Cancel:
        {
            sql_ptr_->UpdateDeviceMissionStatus("arm", 5);

            break;
        }
        case data::common::MissionStatus::Error:
        {
            sql_ptr_->UpdateDeviceMissionStatus("arm", 6);

            break;
        }
        case data::common::MissionStatus::EStop:
        {
            sql_ptr_->UpdateDeviceMissionStatus("arm", 7);

            break;
        }
    }
}

void yf::sys::nw_sys::UpdateDbCurTaskStatusAndLog()
{
    // currently, only consider arm mission status.
    if( nw_status_ptr_->arm_mission_status == yf::data::common::MissionStatus::Idle ||
        nw_status_ptr_->arm_mission_status == yf::data::common::MissionStatus::Finish)
    {
        sql_ptr_->UpdateTaskData(nw_status_ptr_->db_cur_task_id, 3);
        sql_ptr_->UpdateTaskLog(nw_status_ptr_->db_cur_task_id, 3);
    }

    if( nw_status_ptr_->arm_mission_status == yf::data::common::MissionStatus::Error ||
        nw_status_ptr_->arm_mission_status == yf::data::common::MissionStatus::EStop)
    {
        sql_ptr_->UpdateTaskData(nw_status_ptr_->db_cur_task_id, 5);
        sql_ptr_->UpdateTaskLog(nw_status_ptr_->db_cur_task_id, 5);
    }

    // todo: what about Cancel?

}

void yf::sys::nw_sys::UpdateDbCurJobStatusAndLog()
{
    // currently, only consider arm mission status.
    if( nw_status_ptr_->arm_mission_status == yf::data::common::MissionStatus::Idle ||
        nw_status_ptr_->arm_mission_status == yf::data::common::MissionStatus::Finish)
    {
        sql_ptr_->UpdateJobData(nw_status_ptr_->db_cur_job_id, 3);
        sql_ptr_->UpdateJobLog(nw_status_ptr_->db_cur_job_id, 3);
    }

    if( nw_status_ptr_->arm_mission_status == yf::data::common::MissionStatus::Error ||
        nw_status_ptr_->arm_mission_status == yf::data::common::MissionStatus::EStop)
    {
        // error
        //
        sql_ptr_->UpdateJobData(nw_status_ptr_->db_cur_job_id, 5);
        sql_ptr_->UpdateJobLog(nw_status_ptr_->db_cur_job_id, 5);
    }
}

void yf::sys::nw_sys::GetSysControlMode()
{
    switch (sql_ptr_->GetSysControlMode())
    {
        case 0:
        {
            nw_status_ptr_->sys_control_mode_ = data::common::SystemMode::Auto;
            break;
        }

        case 1:
        {
            nw_status_ptr_->sys_control_mode_ = data::common::SystemMode::Manual;
            break;
        }

        case 2:
        {
            nw_status_ptr_->sys_control_mode_ = data::common::SystemMode::ManualSetting;
            break;
        }

        case 3:
        {
            nw_status_ptr_->sys_control_mode_ = data::common::SystemMode::Recovery;
            break;
        }

    }
}

void yf::sys::nw_sys::WaitSchedulesInitialCheck()
{
    bool init_check_continue_flag = true;

    bool sys_init_check_continue_flag = true;
    bool ugv_init_check_continue_flag = true;

    bool arm_init_check_continue_flag = true;
    bool arm_init_status_check_continue_flag = true;
    bool arm_init_position_check_continue_flag = true;


    // Initial Check Loop.
    //
    while (init_check_continue_flag)
    {
        LOG(INFO) << "[thread_WaitSchedules]: 1.1 update sys control mode";
        GetSysControlMode();

        LOG(INFO) << "[thread_WaitSchedules]: 1.2 update all devices status";

        LOG(INFO) << "[thread_WaitSchedules]: 1.2.1 update arm_connection_status and arm_mission_status";
        tm5.UpdateArmCurMissionStatus();

        LOG(INFO) << "[thread_WaitSchedules]: 1.3 check functions   [Start]";

        LOG(INFO) << "[thread_WaitSchedules]: 1.3.1 check sys_control_mode   [Start]";
        switch (nw_status_ptr_->sys_control_mode_)
        {

            case data::common::SystemMode::Auto:
            {
                LOG(INFO) << "[thread_WaitSchedules]: 1.3.1.1 sys_control_mode: Auto";
                sql_ptr_->UpdateSysAdvice(0);
                sys_init_check_continue_flag = false;
                break;
            }

            case data::common::SystemMode::Manual:
            {
                LOG(INFO) << "[thread_WaitSchedules]: 1.3.1.1 sys_control_mode: Manual";
                LOG(INFO) << "[thread_WaitSchedules]: 1.3.1.1 wait for Auto Mode";
                sql_ptr_->UpdateSysAdvice(7);
                while (nw_status_ptr_->sys_control_mode_ != data::common::SystemMode::Auto)
                {
                    // update sys control mode
                    GetSysControlMode();

                    ///TIME
                    sleep.sec(2);
                }
                break;
            }

            case data::common::SystemMode::ManualSetting:
            {
                LOG(INFO) << "[thread_WaitSchedules]: 1.3.1.1 sys_control_mode: ManualSetting";
                LOG(INFO) << "[thread_WaitSchedules]: 1.3.1.1 wait for Auto Mode";
                sql_ptr_->UpdateSysAdvice(7);
                while (nw_status_ptr_->sys_control_mode_ != data::common::SystemMode::Auto)
                {
                    // update sys control mode
                    GetSysControlMode();

                    ///TIME
                    sleep.sec(2);
                }
                break;
            }
        }
        LOG(INFO) << "[thread_WaitSchedules]: 1.3.1 check sys_control_mode   [Complete]";

        LOG(INFO) << "[thread_WaitSchedules]: 1.3.2 check arm   [Start]";
        if(arm_init_check_continue_flag)
        {
            /// for arm_init_status_check_continue_flag

            if( nw_status_ptr_->arm_connection_status == data::common::ConnectionStatus::Connected &&
                nw_status_ptr_->arm_mission_status    == data::common::MissionStatus::Idle)
            {
                LOG(INFO) << "[thread_WaitSchedules]: 1.3.2 check arm   [Complete]";
                arm_init_status_check_continue_flag = false;
            }
            else
            {
                // For Arm Paused too long situation
                //
                if ( nw_status_ptr_->arm_connection_status == data::common::ConnectionStatus::Connected &&
                     nw_status_ptr_->arm_mission_status == data::common::MissionStatus::Error)
                {
                    LOG(INFO) << "[thread_WaitSchedules]: 1.3.2.1 Please stop robotic arm project...";
                    sql_ptr_->UpdateSysAdvice(6);
                }

                // For Arm Disconnected situation
                //
                if( nw_status_ptr_->arm_connection_status == data::common::ConnectionStatus::Disconnected)
                {
                    LOG(INFO) << "[thread_WaitSchedules]: 1.3.2.2 Arm disconnected...";
                    LOG(INFO) << "[thread_WaitSchedules]: 1.3.2.3 Please switch to Recovery Mode..."; // Ask for recovery mode
                    sql_ptr_->UpdateSysAdvice(3);

                    if(nw_status_ptr_->sys_control_mode_ == data::common::SystemMode::Recovery)
                    {
                        LOG(INFO) << "[thread_WaitSchedules]: 1.3.2.4 Please play arm project..."; // Ask for recovery mode
                        sql_ptr_->UpdateSysAdvice(5);

                        tm5.WaitForConnection();
                        // Get Status and update to SQL
                        tm5.GetConnectionStatus();
                        tm5.GetMissionStatus();

                        sql_ptr_->UpdateSysAdvice(1);
                        LOG(INFO) << "[thread_WaitSchedules]: 1.3.2 check arm   [Complete]";
                    }
                }
            }

            /// for arm_init_position_check_continue_flag

            arm_init_position_check_continue_flag = false;

            /// overall arm flag checking

            if(arm_init_status_check_continue_flag == false &&
               arm_init_position_check_continue_flag == false)
            {
                arm_init_check_continue_flag = false;
            }
            else
            {
                arm_init_check_continue_flag = true;
            }
        }

        LOG(INFO) << "[thread_WaitSchedules]: 1.3.3 check ugv   [Start]";
        bool ugv_init_check_continue_flag = false;
        LOG(INFO) << "[thread_WaitSchedules]: 1.3.3 check ugv   [Complete]";

        LOG(INFO) << "[thread_WaitSchedules]: 1.3 check functions   [Complete]";

        // check the initial_check_flag
        if(sys_init_check_continue_flag == false &&
           arm_init_check_continue_flag == false &&
           ugv_init_check_continue_flag == false )
        {
            init_check_continue_flag = false;
        }
        else
        {
            init_check_continue_flag = true;
        }

        LOG(INFO) << "[thread_WaitSchedules]: 1.4 Update Status to SQL";
        LOG(INFO) << "[thread_WaitSchedules]: 1.4.1 Update Status to SQL: Arm";

        UpdateDbDeviceArmConnectionStatus();
        UpdateDbDeviceArmMissionStatus();

        ///TIME
        sleep.ms(1500);
    }
}

//@@ input: plc_register: 000-100
//
bool yf::sys::nw_sys::WaitForUgvPLCRegisterInt(const int &plc_register, const int &value, const int &timeout_min)
{

    if(plc_register > 100 || plc_register <= 0)
    {
        std::cerr << "wrong plc register!" << std::endl;
        return false;
    }

    std::string time_future = sql_ptr_->CountdownTime(sql_ptr_->TimeNow(),timeout_min);

    LOG(INFO) << "Wait for PLC Register " << plc_register << " == " << value;
    while (true)
    {
        // get cur ugv plc register xx, value xxx.
        auto result = mir100_ptr_->GetPLCRegisterIntValue(plc_register);

        if (value != result)
        {
            ///TIME
            sleep.ms(200);
        }
        else
        {
            break;
        }

        /// IPC1 waits too long.
        if(!sql_ptr_->isFutureTime(time_future, sql_ptr_->TimeNow()))
        {
            LOG(INFO) << "Wait for PLC " << plc_register << " == " << value << " failed!";

            return false;
        }

        /// Or Ugv has error.
        mir100_ptr_->RetrieveUgvCurrentMissionState();
        if(nw_status_ptr_->ugv_mission_status_ == data::common::UgvState::Error)
        {
            LOG(INFO) << "Wait for PLC " << plc_register << " == " << value << " failed!";
            LOG(INFO) << "Ugv has Error!";

            return false;
        }
    }

    return true;

}

//@@ input: plc_register: 101-200
//
bool yf::sys::nw_sys::WaitForUgvPLCRegisterFloat(const int &plc_register, const float &value, const int &timeout_min)
{

    if(plc_register <= 100 || plc_register >200)
    {
        std::cerr << "wrong plc register!" << std::endl;
        return false;
    }

    std::string time_future = sql_ptr_->CountdownTime(sql_ptr_->TimeNow(),timeout_min);

    LOG(INFO) << "Wait for PLC Register " << plc_register << " == " << value;
    while (true)
    {
        // get cur ugv plc register xx, value xxx.
        auto result = mir100_ptr_->GetPLCRegisterFloatValue(plc_register);

        if (value != result)
        {
            ///TIME
            sleep.ms(200);
        }
        else
        {
            break;
        }

        /// IPC1 waits too long.
        if(!sql_ptr_->isFutureTime(time_future, sql_ptr_->TimeNow()))
        {
            LOG(INFO) << "The PLC Register has not changed.";

            return false;
        }
    }

    return true;

}

void yf::sys::nw_sys::ArmPickTool(const yf::data::arm::TaskMode &task_mode)
{
    switch (task_mode)
    {
        case yf::data::arm::TaskMode::Mopping:
        {
            tm5.ArmTask("Post pick_mop");
            break;
        }
        case yf::data::arm::TaskMode::UVCScanning:
        {
            tm5.ArmTask("Post pick_uvc");
            break;
        }
    }

    return;
}

void yf::sys::nw_sys::ArmPlaceTool(const yf::data::arm::TaskMode &task_mode)
{
    switch (task_mode)
    {
        case yf::data::arm::TaskMode::Mopping:
        {
            tm5.ArmTask("Post place_mop");
            break;
        }
        case yf::data::arm::TaskMode::UVCScanning:
        {
            tm5.ArmTask("Post place_uvc");
            break;
        }
    }

    return;
}

void yf::sys::nw_sys::ArmSetOperationArea(const yf::data::arm::OperationArea &operation_area)
{
    switch (operation_area)
    {
        case data::arm::OperationArea::Rear:
        {
            tm5.ArmTask("Set operation_area = 0");
            break;
        }
        case data::arm::OperationArea::Left:
        {
            tm5.ArmTask("Set operation_area = 1");
            break;
        }
        case data::arm::OperationArea::Right:
        {
            tm5.ArmTask("Set operation_area = 2");
            break;
        }
        case data::arm::OperationArea::RearRightCorner:
        {
            tm5.ArmTask("Set operation_area = 3");
            break;
        }
        case data::arm::OperationArea::RightLower:
        {
            tm5.ArmTask("Set operation_area = 4");
            break;
        }
        case data::arm::OperationArea::RightHigher:
        {
            tm5.ArmTask("Set operation_area = 5");
            break;
        }
        case data::arm::OperationArea::RearLeftCorner:
        {
            tm5.ArmTask("Set operation_area = 6");
            break;
        }
        case data::arm::OperationArea::LeftLower:
        {
            tm5.ArmTask("Set operation_area = 7");
            break;
        }
        case data::arm::OperationArea::LeftHigher:
        {
            tm5.ArmTask("Set operation_area = 8");
            break;
        }
    }

    return;
}

std::string yf::sys::nw_sys::ArmGetPointStr(const yf::data::arm::Point3d &point)
{
    std::string x_str = std::to_string(point.x);
    std::string y_str = std::to_string(point.y);
    std::string z_str = std::to_string(point.z);
    std::string rx_str = std::to_string(point.rx);
    std::string ry_str = std::to_string(point.ry);
    std::string rz_str = std::to_string(point.rz);

    std::string point_str = x_str + "," + y_str + "," + z_str + "," + rx_str + "," + ry_str + "," + rz_str;

    return point_str;
}

void
yf::sys::nw_sys::ArmSetToolAngle(const yf::data::arm::TaskMode &task_mode, const yf::data::arm::ToolAngle &tool_angle)
{
    if(task_mode == data::arm::TaskMode::Mopping)
    {
        switch (tool_angle)
        {
            case data::arm::ToolAngle::Zero:
            {
                if(cur_tool_angle_ == data::arm::ToolAngle::FortyFive)
                {
                    tm5.ArmTask("Post tool_angle_0");
                    cur_tool_angle_ = data::arm::ToolAngle::Zero;
                }
                break;
            }
            case data::arm::ToolAngle::FortyFive:
            {
                if(cur_tool_angle_ == data::arm::ToolAngle::Zero)
                {
                    tm5.ArmTask("Post tool_angle_45");
                    cur_tool_angle_ = data::arm::ToolAngle::FortyFive;
                }
                break;
            }
        }
    }

    return;
}

void yf::sys::nw_sys::ArmSetViaPoints(const std::deque<yf::data::arm::Point3d>& via_points,
                                      const yf::data::arm::ToolAngle &tool_angle)
{
    switch (tool_angle)
    {
        case data::arm::ToolAngle::Zero:
        {
            for (int n = 0; n < via_points.size(); n ++)
            {
                auto via_point = via_points[n];

                std::string via_point_str = this->ArmGetPointStr(via_point);

                int via_num = n+1;
                std::string via_num_str = std::to_string(via_num);

                std::string command = "Set via0_" + via_num_str + " = " +via_point_str;

                tm5.ArmTask(command);
            }

            break;
        }
        case data::arm::ToolAngle::FortyFive:
        {
            for (int n = 0; n < via_points.size(); n ++)
            {
                auto via_point = via_points[n];

                std::string via_point_str = this->ArmGetPointStr(via_point);

                int via_num = n+1;
                std::string via_num_str = std::to_string(via_num);

                std::string command = "Set via45_" + via_num_str + " = " +via_point_str;

                tm5.ArmTask(command);
            }

            break;
        }
    }

    return;
}

void yf::sys::nw_sys::ArmSetApproachPoint(const yf::data::arm::Point3d& approach_point, const yf::data::arm::ToolAngle &tool_angle)
{
    switch (tool_angle)
    {
        case data::arm::ToolAngle::Zero:
        {
            std::string approach_point_str = this->ArmGetPointStr(approach_point);

            std::string command = "Set 0_approach_point = " + approach_point_str;

            tm5.ArmTask(command);

            break;
        }
        case data::arm::ToolAngle::FortyFive:
        {
            std::string approach_point_str = this->ArmGetPointStr(approach_point);

            std::string command = "Set 45_approach_point = " + approach_point_str;

            tm5.ArmTask(command);

            break;
        }
    }

    return;
}

void yf::sys::nw_sys::ArmPostViaPoints(const yf::data::arm::TaskMode& task_mode,
                                       const yf::data::arm::ToolAngle& tool_angle,
                                       const yf::data::arm::ModelType& model_type,
                                       const int& arm_mission_config_id)
{
    if(task_mode == data::arm::TaskMode::Mopping)
    {
        switch (tool_angle)
        {
            case data::arm::ToolAngle::Zero:
            {
                switch(model_type)
                {
                    case data::arm::ModelType::Wall:
                    {
                        // get Force Type
                        auto force_type = tm5.GetForceType(arm_mission_config_id);
                        switch (force_type)
                        {
                            case data::arm::ForceType::via0_z_120N:
                            {
//                                tm5.SetMotorHigh();

                                std::string command = "Post arm_via0_force_120N";
                                tm5.ArmTask(command);

//                                tm5.SetMotorLow();
                            }
                            default:
                            {
                                tm5.SetMotorHigh();

                                std::string command = "Post arm_via0_line";
                                tm5.ArmTask(command);

                                tm5.SetMotorLow();
                                break;
                            }
                        }

                        //Post arm_via0_line_force_120N


                        break;
                    }
                    default:
                    {
                        tm5.SetMotorHigh();

                        std::string command = "Post arm_via0_line";
                        tm5.ArmTask(command);

                        tm5.SetMotorLow();

                        break;
                    }
                }

                break;
            }
            case data::arm::ToolAngle::FortyFive:
            {

                switch (model_type)
                {
                    case data::arm::ModelType::Windows:
                    {
                        tm5.SetMotorHigh();

                        std::string command = "Post arm_via45_line_z";
                        tm5.ArmTask(command);

                        tm5.SetMotorLow();

                        break;
                    }
                    case data::arm::ModelType::DeskRectangle:
                    {
                        tm5.SetMotorHigh();

                        std::string command = "Post arm_via45_line_d";
                        tm5.ArmTask(command);

                        tm5.SetMotorLow();

                        break;
                    }
                    case data::arm::ModelType::DeskCircle:
                    {
                        tm5.SetMotorHigh();

                        std::string command = "Post arm_via45_p2p";
                        tm5.ArmTask(command);

                        tm5.SetMotorLow();

                        break;
                    }
                    case data::arm::ModelType::DeskPolygon:
                    {
                        tm5.SetMotorHigh();

                        std::string command = "Post arm_via45_line_z";
                        tm5.ArmTask(command);

                        tm5.SetMotorLow();

                        break;
                    }
                    case data::arm::ModelType::Wall:
                    {
                        // get Force Type
                        auto force_type = tm5.GetForceType(arm_mission_config_id);

                        switch (force_type)
                        {
                            case data::arm::ForceType::via45_z:
                            {
                                tm5.SetMotorHigh();

                                std::string command = "Post arm_via45_line_z";
                                tm5.ArmTask(command);

                                tm5.SetMotorLow();

                                break;
                            }
                            case data::arm::ForceType::via45_y:
                            {
                                std::string command = "Post arm_via45_line_y";
                                tm5.ArmTask(command);

                                break;
                            }
                            default:
                            {
                                break;
                            }
                        }
                        break;
                    }
                    case data::arm::ModelType::NurseStation:
                    {
                        tm5.SetMotorHigh();

                        std::string command = "Post arm_via45_line_z";
                        tm5.ArmTask(command);

                        tm5.SetMotorLow();

                        break;
                    }
                    case data::arm::ModelType::Skirting:
                    {
                        std::string command = "Post arm_via45_line_y";
                        tm5.ArmTask(command);

                        break;
                    }
                    case data::arm::ModelType::Handrail:
                    {
                        // get Force Type
                        auto force_type = tm5.GetForceType(arm_mission_config_id);

                        switch (force_type)
                        {
                            case data::arm::ForceType::via45_z:
                            {
                                tm5.SetMotorHigh();

                                std::string command = "Post arm_via45_line_z";
                                tm5.ArmTask(command);

                                tm5.SetMotorLow();

                                break;
                            }
                            default:
                            {
                                break;
                            }
                        }
                        break;
                    }
                    default:
                    {
                        tm5.SetMotorHigh();

                        std::string command = "Post arm_via45_line_z";
                        tm5.ArmTask(command);

                        tm5.SetMotorLow();

                        break;
                    }
                }
            }
        }
    }
    else
        if(task_mode == data::arm::TaskMode::UVCScanning)
        {
            std::string command = "Post uvc_via0_line";

            tm5.ArmTask(command);
        }


    return;
}

void yf::sys::nw_sys::ArmPickPad()
{
    switch (cur_model_type_)
    {
        case data::arm::ModelType::Handrail:
        {
            tm5.ArmTask("Post pick_small_pad");
            break;
        }
        case data::arm::ModelType::Windows:
        {
            tm5.ArmTask("Post pick_large_pad");
            break;
        }
        case data::arm::ModelType::NurseStation:
        {
            tm5.ArmTask("Post pick_large_pad");
            break;
        }
        case data::arm::ModelType::DeskPolygon:
        {
            tm5.ArmTask("Post pick_large_pad");
            break;
        }
        case data::arm::ModelType::Skirting:
        {
            tm5.ArmTask("Post pick_large_pad");
            break;
        }
        default:
        {
            tm5.ArmTask("Post pick_small_pad");
            break;
        }
    }
}

void yf::sys::nw_sys::ArmRemovePad()
{
    switch (cur_model_type_)
    {
        case data::arm::ModelType::Handrail:
        {
            tm5.ArmTask("Post remove_small_pad");
            break;
        }
        case data::arm::ModelType::Windows:
        {
            tm5.ArmTask("Post remove_large_pad");
            break;
        }
        case data::arm::ModelType::NurseStation:
        {
            tm5.ArmTask("Post remove_large_pad");
            break;
        }
        case data::arm::ModelType::DeskPolygon:
        {
            tm5.ArmTask("Post remove_large_pad");
            break;
        }
        case data::arm::ModelType::Skirting:
        {
            tm5.ArmTask("Post remove_large_pad");
            break;
        }
        default:
        {
            tm5.ArmTask("Post remove_small_pad");
            break;
        }
    }
}

void yf::sys::nw_sys::ArmAbsorbWater()
{

    // get & set absorb type
    int absorb_type = this->ArmGetAbsorbType();
    std::string absorb_type_command = "Set absorb_type = " + std::to_string(absorb_type);
    tm5.ArmTask(absorb_type_command);

    sleep.ms(200);

    tm5.ArmTask("Post arm_absorb_water");
}

void yf::sys::nw_sys::thread_WebStatusManager()
{
    /// Configuration
    ///
    /// duration for each thread
    int duration_min_ugv_battery = 10;
    int duration_sec_ugv_pos = 10;

    /// each thread's control flag
    web_status_flag_ = true;

    /// kick off each thread
    th_web_ugv_battery_ = std::thread(&nw_sys::thread_Web_UgvBatteryPercentage, this,
                                      std::ref(web_status_flag_),std::move(duration_min_ugv_battery));

    th_web_ugv_position_ = std::thread(&nw_sys::thread_Web_UgvCurPosition, this,
                                       std::ref(web_status_flag_),std::move(duration_sec_ugv_pos));

    /// Duration: 1 minute
    while (schedule_flag_ == true)
    {
        sleep.minute(1);
    }

    /// Stop all web status threads
    web_status_flag_ = false;
}

void yf::sys::nw_sys::thread_Web_UgvBatteryPercentage(bool &web_status_flag,
                                                      const int& sleep_duration)
{
    while(web_status_flag)
    {
        // todo: need to handle how to check whether mir is ON/OFF

        try
        {
            // retrieve data from ugv
            auto result = mir100_ptr_->GetBatteryPercentage();
            // update to sql
            sql_ptr_->UpdateDeviceBatteryCapacity("ugv", result);
        }
        catch (std::error_code ec)
        {
            std::cerr << "Cannot Get MiR100 Battery Percentage. Will Try Again Later";
        }


        sleep.minute(sleep_duration);
    }
}

void yf::sys::nw_sys::thread_Web_UgvCurPosition(const bool &web_status_flag,
                                                const int &sleep_duration)
{
    sleep.sec(sleep_duration);

    while(web_status_flag)
    {
        // todo: need to handle how to check whether mir is ON/OFF


        try
        {
            // retrieve data from ugv
            auto result = mir100_ptr_->GetCurPosition();
            // update to sql
            sql_ptr_->UpdateDeviceUgvCurPosition(result[0],result[1],result[2]);
        }
        catch (std::error_code ec)
        {
            std::cerr << "Cannot Get MiR100 Cur Position. Will Try Again Later";
        }

        sleep.sec(sleep_duration);
    }
}

/// \brief Sorting all incoming job ids, do mopping jobs first, and then ucv scanning jobs
///
void yf::sys::nw_sys::JobsFilter(std::deque<int>& q_ids)
{
    q_job_mopping_ids.clear();
    q_job_uvc_scanning_ids.clear();

    for(int n = 0; n < q_ids.size(); n++)
    {
        // for each job id, check its task_mode
        switch (sql_ptr_->GetTaskMode(q_ids[n]))
        {
            case 1:
            {
                q_job_mopping_ids.push_back(q_ids[n]);
                break;
            }
            case 2:
            {
                q_job_uvc_scanning_ids.push_back(q_ids[n]);
                break;
            }
        }
    }

    q_ids.clear();

    q_ids.insert(q_ids.end(), q_job_mopping_ids.begin(), q_job_mopping_ids.end());
    q_ids.insert(q_ids.end(), q_job_uvc_scanning_ids.begin(), q_job_uvc_scanning_ids.end());

}

int yf::sys::nw_sys::ArmGetAbsorbType()
{
    int absorb_type = 0;

    switch (cur_model_type_)
    {
        case data::arm::ModelType::Skirting:
        {
            absorb_type = 1;
            break;
        }
        default:
        {
            absorb_type = 0;
            break;
        }
    }

    return absorb_type;
}

void yf::sys::nw_sys::GetScheduleCommand(const int& id)
{
    switch (sql_ptr_->GetScheduleCommand(id))
    {
        case 1:
        {
            nw_status_ptr_->db_cur_schedule_command_ = data::schedule::ScheduleCommand::CleaningJob;
            break;
        }

        case 2:
        {
            nw_status_ptr_->db_cur_schedule_command_ = data::schedule::ScheduleCommand::UgvBackToChargingStation;
            break;
        }

        case 3:
        {
            nw_status_ptr_->db_cur_schedule_command_ = data::schedule::ScheduleCommand::ArmBackToHomePos;
            break;
        }

    }
}

void yf::sys::nw_sys::thread_WaitForCancelUgvMission()
{
    // wait for signal

    // keep checking arm mission status and connection status

    // if arm disconnected or error

    // pause mir first

    // cancel mir current mission

    // reset signal

    // wait for signal
}

void yf::sys::nw_sys::thread_WaitForPauseArm()
{
    // wait for signal

    // keep checking ugv status

    // if ugv e-stop

    // pause Arm

    // Notify thread_WaitForResumeArm()

    // reset signal

    // wait for signal
}

void yf::sys::nw_sys::thread_WaitForResumeArm()
{
    // wait for signal

    // keep checking ugv status

    // if ugv resume

    // replay Arm

    // reset signal

    // wait for signal
}

void yf::sys::nw_sys::UpdateDbErrorLog()
{
    /// For Arm

    // Connection Error
    switch (nw_status_ptr_->arm_connection_status)
    {
        case data::common::ConnectionStatus::Disconnected:
        {
            //L"手臂未連接，請聯係管理員！"
            std::string error_description = "Arm Disconnected. Please ask Admin for help!";
            sql_ptr_->UpdateErrorLog(0,error_description);
            break;
        }
    }

    // Mission Error
    switch (nw_status_ptr_->arm_mission_status)
    {
        case data::common::MissionStatus::Error:
        {
//            std::wstring error_description = L"手臂運行錯誤，請聯係管理員！";
            std::string error_description = "Arm has Error. Please ask Admin for help!";
            sql_ptr_->UpdateErrorLog(1,error_description);
            break;
        }
        case data::common::MissionStatus::EStop:
        {
//            std::wstring error_description = L"手臂已急停，請聯係管理員！";
            std::string error_description = "Arm has been Emergency Stop. Please ask Admin for help!";
            sql_ptr_->UpdateErrorLog(2,error_description);
            break;
        }
        default:
        {
            break;
        }
    }

    /// For Ugv

}

void yf::sys::nw_sys::DoJobUgvBackToChargingStation()
{
    // DB: update schedule_job & schedule_job_log

    /// Do job
    // 1. ugv changes map & its init_pos
    mir100_ptr_->ClearErrorState();
    mir100_ptr_->ChangeMapByDBMapStatus();
    sleep.sec(5);
    if(mir100_ptr_->WaitForModeId(7,1))
    {
        mir100_ptr_->ChangeInitPositionByDBMapStatus();
    }

    // 2. ugv creates its charging mission.
    mir100_ptr_->PostMissionForCharging();
    mir100_ptr_->PostActionsForCharging();

    // 3. ugv goes to charging station
    cur_mission_guid_ = mir100_ptr_->GetCurMissionGUID();
    mir100_ptr_->PostMissionQueue(cur_mission_guid_);
    ///TIME
    sleep.ms(200);
    mir100_ptr_->Play();

    // DB: update schedule_job & schedule_job_log
}

void yf::sys::nw_sys::DoJobArmBackToHomePos()
{
    // DB: update schedule_job & schedule_job_log

    // Do job
    tm5.ArmTask("Post arm_back_home");

    // DB: update schedule_job & schedule_job_log
}
















/// \brief
/// \param
/// \param
/// \param
/// \note

// Do Schedule
// (1) Retrieve Schedule from database and parse...

// Do Job
// (1) Parse Jobs...

// Do Task
// (1) Check all device status
// (2) if error, update the status(Task, Job, Schedule), report to the user.
// (3) Task pause. wait for resume. how? keep checking the connection. and check the status for each device.
// (4) if normal(all device are idle), Do each device's task.

// (5) MiR Task
// 1. check arm task.
// 2. if arm is pause. mir should pause also.
// 3. update the status(Task, Job, Schedule), Task pause. wait for resume.

// (6) Arm Task
// 1. check mir task.
// 2. if mir is pause. arm should pause also. *** check whether the arm can be paused via network. and will the arm send message when it is resuming its job...
// 3. update the status(Task, Job, Schedule), Task pause. wait for resume.

// 1. initial check, check arm status
// 2. assign arm task.
// 3. update arm task.

