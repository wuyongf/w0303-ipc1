#pragma once

#include <thread>               // STL
#include <filesystem>
#include <glog/logging.h>       // GLOG
#include "net.h"                // Net
#include "net_w0303_server.h"
#include "net_w0303_client.h"
#include "sql.h"                // Database
#include "nw_status.h"          // Status & DataTypes
#include "arm.h"                // Arm
#include "ugv.h"                // UGV
#include "ipc2.h"               // IPC2

namespace yf
{
    namespace sys
    {
        class nw_sys
        {
        public:
            /// Sys Basic Method

            // Constructor & Destructor
            nw_sys(const int& ipc_port);
            virtual ~nw_sys(){}

            // Startup the system
            void Start();

            // Close the system
            void Close();

        public:
            /// Two Main Functions

            void thread_WaitSchedules();

            void thread_DoSchedules();

        private:

            void thread_IPCServerStartup(std::shared_ptr<IPCServer>& server_ptr, bool& server_flag);

        public:

            void WaitSchedulesInitialCheck();

        public:
            /// Methods for two main functions

            void DoJobs(const int& cur_schedule_id);
            void DoTasks(const int& cur_job_id, const int& task_group_id);

            void DoJobUgvBackToChargingStation();
            void DoJobArmBackToHomePos();

            void RedoJob(const int& cur_schedule_id,const yf::data::schedule::ScheduleCommand& redo_command);

            //
            void JobsFilter(std::deque<int>& q_ids);

        public:
            /// Arm Methods: based on tm5.ArmTask();

            void ArmPickTool (const yf::data::arm::TaskMode& task_mode);
            void ArmPlaceTool(const yf::data::arm::TaskMode& task_mode);
            void ArmPlaceToolSafety();

            void ArmPickPad(const int& job_id);
            void ArmRemovePad(const int& job_id);
            void ArmAbsorbWater();
            int  ArmGetAbsorbType();

            // for safety concern
            void ArmCheckPadIsEmpty();
            // for updating pad number
            void ArmUpdatePadNo();

            void ArmSetOperationArea(const yf::data::arm::OperationArea& operation_area);
            void ArmSetToolAngle(const yf::data::arm::TaskMode& task_mode, const yf::data::arm::ToolAngle& tool_angle);
            void ArmSetApproachPoint(const yf::data::arm::Point3d& approach_point, const yf::data::arm::ToolAngle& tool_angle);
            void ArmSetViaPoints(const std::deque<yf::data::arm::Point3d>& via_points, const yf::data::arm::ToolAngle& tool_angle);
            void ArmPostViaPoints(const yf::data::arm::TaskMode& task_mode, const yf::data::arm::ToolAngle& tool_angle, const yf::data::arm::ModelType& model_type, const int& arm_mission_config_id);

            std::string ArmGetPointStr(const yf::data::arm::Point3d& point);

        public:
            /// Ugv Method: based on REST API

            bool WaitForUgvPLCRegisterInt(const int& plc_register, const int& value, const int& timeout_min);
            bool WaitForUgvPLCRegisterFloat(const int& plc_register, const float& value, const int& timeout_min);

        private:
            /// Functions for the DoTasks()

            std::vector<std::vector<std::string>> GetRealPCFileNames();

        protected:
            /// SQL Related

            void UpdateDbScheduleBeforeTask (const int& cur_schedule_id);
            void UpdateDbScheduleAfterTask  (const int& cur_schedule_id);

            void UpdateDbDeviceArmConnectionStatus();
            void UpdateDbDeviceArmMissionStatus();

            void UpdateDbCurTaskStatusAndLog();

            void UpdateDbErrorLog();

            void UpdateDbCurJobStatusAndLog();

            void GetSysControlMode();
            void GetScheduleCommand(const int&id);

        private:
            /// Web Status Manager

            // properties
            bool web_status_flag_ = false;

            // methods
            void thread_WebStatusManager();

            void thread_Web_UgvStatus(const bool& web_status_flag, const int& sleep_duration);

            std::thread th_web_status_manager_;
            std::thread th_web_ugv_status_;

        private:
            /// Useful tools

            yf::algorithm::TimeSleep sleep;
            yf::algorithm::Timer timer;

        private: /// SYS basic properties

            // shared_ptr: nw_status,sql,mir100,ipc_server
            std::shared_ptr<yf::status::nw_status>  nw_status_ptr_;
            std::shared_ptr<yf::sql::sql_server>    sql_ptr_;
            std::shared_ptr<yf::ugv::mir>           mir100_ptr_;
            std::shared_ptr<IPCServer>              ipc_server_ptr_;

            yf::arm::tm         tm5;
            yf::ipc2::raspberry pi4;

            // flow control variables
            //
            bool        ipc_server_flag_ = true;
            std::thread th_ipc_server_;

            std::thread th_wait_schedules_;
            std::thread th_do_schedules_;

        private:
            /// Overall Control --- thread wait_schedules and do_schedules

            // schedule_flag
            bool schedule_flag_ = true;

            /// wait schedule
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

            std::deque<int> q_job_uvc_scanning_ids;
            std::deque<int> q_job_mopping_ids;

            std::deque<int> q_task_ids;
            int task_number = 0;

            std::deque<int>  all_available_schedule_ids;

        private:
            /// for current job.

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

///@@
/// 0. This is a constructor
/// 1. Establish a network server: create shared ptr: ipc_server_ptr
/// 2. Assign ipc port for TM network connection.
yf::sys::nw_sys::nw_sys(const int &ipc_port)
{
    ipc_server_ptr_ = std::make_shared<IPCServer>(ipc_port);    //e.g. ipc_port: 12345
}

void yf::sys::nw_sys::Start()
{
    /// 0. Initialization
    //
    //  0.1 Initialize a nw_status_ptr, for storing the global variables.
    nw_status_ptr_ = std::make_shared<yf::status::nw_status>();
    //  0.2 Initialize a sql_ptr, for interaction with database.
    sql_ptr_       = std::make_shared<yf::sql::sql_server>("SQL Server","192.168.7.127","NW_mobile_robot_sys","sa","NWcadcam2021");
    //  0.2.1 sql: webpage initialization
    sql_ptr_->UpdateDeviceConnectionStatus("arm", 0);
    sql_ptr_->UpdateDeviceConnectionStatus("ugv", 0);
    sql_ptr_->UpdateDeviceMissionStatus("arm", 6);
    sql_ptr_->UpdateDeviceMissionStatus("ugv", 6);

    //  0.3 Establish an ipc_server and keep updating, keep waiting for connection from other device.
    th_ipc_server_ = std::thread(&nw_sys::thread_IPCServerStartup, this, std::ref(ipc_server_ptr_), std::ref(ipc_server_flag_));

    /// 1. Initialization --- arm
    //
    // 1.1 startup tm5 control module and pass <1> ipc_server_ptr, <2> nw_status_ptr <3> sql_ptr
    tm5.Start(ipc_server_ptr_,nw_status_ptr_,sql_ptr_);

    // 1.2 block the program, wait for arm connection
    // notify the database.
    sql_ptr_->UpdateSysAdvice(11);
    tm5.WaitForConnection();

    // 1.3 update the connection status to sys_status & database ;
    tm5.GetConnectionStatus();

    /// 2. Initialization --- ugv
    //
    mir100_ptr_ = std::make_shared<yf::ugv::mir>();
    mir100_ptr_->Start("192.168.7.34", nw_status_ptr_, sql_ptr_);

    /// TIME
    sleep.ms(500);

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
                        // prerequisite
                        // 1. Input data in DB (dock_info)
                        // 2. position name in MiR website: ChargingStation
                        DoJobUgvBackToChargingStation();
                        break;
                    }
                    case data::schedule::ScheduleCommand::ArmBackToHomePos:
                    {
                        DoJobArmBackToHomePos();
                        break;
                    }
                    case data::schedule::ScheduleCommand::RedoCleaningJobErrorPart:
                    {
                        // redo job
                        RedoJob(cur_schedule_id,nw_status_ptr_->db_cur_schedule_command_);

                        break;
                    }
                    case data::schedule::ScheduleCommand::RedoCleaningJobWhole:
                    {
                        RedoJob(cur_schedule_id,nw_status_ptr_->db_cur_schedule_command_);
                        break;
                    }
                    case data::schedule::ScheduleCommand::CustomPlan:
                    {
                        /// Testing

                        #if 0
                        /// custom_plan: 1-6
                        auto plan_no = sql_ptr_->GetAvailableCustomPlan();

                        switch (plan_no)
                        {
                            case 1:
                            {
                                tm5.ArmTask("Post demo1");
                                sql_ptr_->ResetCustomPlan(1);
                                break;
                            }
                            case 2:
                            {
                                tm5.ArmTask("Post demo2");
                                sql_ptr_->ResetCustomPlan(2);
                                break;
                            }
                            case 3:
                            {
                                sql_ptr_->ResetCustomPlan(3);
                                break;
                            }
                            case 4:
                            {
                                sql_ptr_->ResetCustomPlan(4);
                                break;
                            }
                            case 5:
                            {
                                sql_ptr_->ResetCustomPlan(5);
                                break;
                            }
                            case 6:
                            {
                                sql_ptr_->ResetCustomPlan(6);
                                break;
                            }
                        }
                        #endif

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
            //  a. break the schedule loop or not?
            //  b. (done above) Update Schedule Status after job. Record status and update to SQL Server

            // For Arm

            // (0) break the task loop or not?

            bool arm_mission_failed_status = nw_status_ptr_->arm_mission_status == yf::data::common::MissionStatus::Error ||
                                             nw_status_ptr_->arm_mission_status == yf::data::common::MissionStatus::EStop;

            if( nw_status_ptr_->arm_connection_status == data::common::ConnectionStatus::Disconnected || arm_mission_failed_status )
            {
                LOG(INFO) << "[thread_DoSchedules]: Abort current schedules...";
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

    /// Job Filter, do small_pad_mopping first!
    this->JobsFilter(q_job_ids);

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

        cur_job_id = q_job_ids.front();
        q_job_ids.pop_front();

        nw_status_ptr_->db_cur_job_id = cur_job_id;

        /// Record the job status to DB

        // 1. Fill table "schedule_job_task"
        sql_ptr_->FillTaskTableForCurJob(cur_job_id);
        nw_status_ptr_->db_cur_task_group_id = sql_ptr_->GetLatestTaskGroupId();

        // 2. Update Job Table & Job Log (Insert job_log_id & Update task_group_id)
        // 2.1 Update table "sys_schedule_job"
        sql_ptr_->UpdateJobTable(cur_job_id, 2);

        // 2.2 Insert job_log_id & Update task_group_id in table "sys_schedule_job_log"
        sql_ptr_->InsertNewJobLog(cur_job_id, 2);
        nw_status_ptr_->db_cur_job_log_id = sql_ptr_->GetLatestJobLogId();
        sql_ptr_->UpdateJobLogTaskGroupId(nw_status_ptr_->db_cur_job_log_id, nw_status_ptr_->db_cur_task_group_id);

        /// Execute Each job!
        DoTasks(cur_job_id, nw_status_ptr_->db_cur_task_group_id);

        /// Job Result Handler
        //  a. Break the loop or not?
        //  b. Record Each Job STATUS.

        LOG(INFO) << "update current jobs status and log...";

        if(nw_status_ptr_->cur_job_success_flag == true)
        {
            // current job finished!
            sql_ptr_->UpdateJobLog(nw_status_ptr_->db_cur_job_log_id, 3);
            sql_ptr_->UpdateJobTable(nw_status_ptr_->db_cur_job_id,3);
        }
        else
        {
            // current job error!
            sql_ptr_->UpdateJobLog(nw_status_ptr_->db_cur_job_log_id, 5);
            sql_ptr_->UpdateJobTable(nw_status_ptr_->db_cur_job_id,5);
        }

        bool arm_program_stop_status = nw_status_ptr_->arm_mission_status == yf::data::common::MissionStatus::Error ||
                                         nw_status_ptr_->arm_mission_status == yf::data::common::MissionStatus::EStop;

        if( nw_status_ptr_->arm_connection_status == data::common::ConnectionStatus::Disconnected || arm_program_stop_status )
        {
            LOG(INFO) << "Arm Disconnected. Cancel the rest of jobs...";
            q_job_ids.clear();

            // current job error!
            sql_ptr_->UpdateJobLog(nw_status_ptr_->db_cur_job_log_id, 5);
            sql_ptr_->UpdateJobTable(nw_status_ptr_->db_cur_job_id,5);

            /// Update Table "sys_status_error_log", informing USER
            UpdateDbErrorLog();
        }
    }

//    LOG(INFO) << "No more jobs...";

    // No more jobs
    //
    // cout the result

    if(nw_status_ptr_->arm_mission_status == data::common::MissionStatus::Idle)
    {
        LOG(INFO) << "All jobs are done";
    }
    else
    {
        LOG(INFO) << "Cancel the rest of jobs...";
    }
}

void yf::sys::nw_sys::DoTasks(const int &cur_job_id, const int& task_group_id)
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
                    sleep.ms(200);

                    if(arm_wait_plc003_success_flag)
                    {
                        /// get current order

                        while(cur_order == pre_order || cur_order == 0)
                        {
                            cur_order = mir100_ptr_->GetPLCRegisterIntValue(2);

                            ///TIME
                            sleep.ms(200);
                        }

                        pre_order = cur_order;

                        /// check arm config is valid or not

                        sleep.ms(1000);

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

                                // get last operation area
                                int  last_arm_config_id = sql_ptr_->GetArmConfigId(cur_model_config_id_, last_valid_order);
                                auto last_operation_area = tm5.GetOperationArea(last_arm_config_id);

                                /// 2. Notify mir100 that arm configuration stage has finished

                                mir100_ptr_->SetPLCRegisterIntValue(3,0);

                                arm_config_stage_success_flag = true; // no use now?

                                /// 3. Wait for flag for executing all arm_mission_configs
                                arm_wait_plc001_success_flag = this->WaitForUgvPLCRegisterInt(1,1,5);

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
                                                    sleep.ms(200);

                                                    this->ArmPickTool(cur_task_mode_);
                                                    sleep.ms(200);

                                                    this->ArmPickPad(cur_job_id);
                                                    sleep.ms(200);

                                                    this->ArmUpdatePadNo();
                                                    sleep.ms(200);

                                                    // for remove_tool
                                                    tm5.set_remove_tool_flag(false);

                                                    // for change_pad: start the timer
                                                    tm5.set_pad_start_timer();
                                                } else
                                                {
                                                    auto change_pad_flag = tm5.CheckChangePadFlag(sql_ptr_->GetModelConfigId(cur_job_id));

                                                    if(change_pad_flag)
                                                    {
                                                        this->ArmRemovePad(cur_job_id);
                                                        sleep.ms(200);
                                                        this->ArmPickPad(cur_job_id);
                                                        sleep.ms(200);
                                                        this->ArmUpdatePadNo();
                                                        sleep.ms(200);

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

                                    ///\ (2) For each order, move to safety position first.
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

                                    ///\brief
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

                                    ///\ (3) Loop all the arm_mission_configs
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
                                            this->ArmSetToolAngle(cur_task_mode_,arm_mission_configs[n].tool_angle);

                                            /// b. vision job initialization
                                            ///   b.1: for None: do nothing
                                            ///   b.2: for Landmark: scan landmark, mark down the record
                                            ///   b.3: for D455: record the point clouds, mark down the record.

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

                                                            continue;

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

                                                        continue;
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
                                                            auto result = tm5.RecordCurRealPointCloud(pc_dir, real_pc_file_name);
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
                                                    switch (arm_mission_configs[n].model_type)
                                                    {
                                                        case data::arm::ModelType::Handle:
                                                        {
                                                            auto feature_type = "planar";
                                                            auto feature_type_id = sql_ptr_->GetFeatureTypeId(feature_type);

                                                            std::vector<int> planar_sets;

                                                            // find the corresponding files
                                                            for (int m =0; m < arm_mission_configs[n].feature_type_ids.size(); m++)
                                                            {
                                                                if(arm_mission_configs[n].feature_type_ids[m] == feature_type_id)
                                                                {
                                                                    planar_sets.push_back(m);
                                                                }
                                                            }

                                                            /// for 1 set 1 view algorithm
                                                            if(planar_sets.size() == 1)
                                                            {
                                                                auto set_no  = planar_sets[0];

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
                                                        }
                                                        case data::arm::ModelType::Handrail:
                                                        {
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
                                                this->ArmSetToolAngle(cur_task_mode_,arm_mission_configs[n].tool_angle);

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

                                    sleep.ms(200);

                                    ///\ (4) Finish Current Arm Mission
                                    ///\    a. For Normal orders: Return 'Safety Position'.
                                    ///\    b. For Last order: Place the tool and then return 'Home Position'

                                    // a.
                                    tm5.ArmTask("Post arm_back_to_safety");
                                    sleep.ms(500);

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
//    // currently, only consider arm mission status.
//    if( nw_status_ptr_->arm_mission_status == yf::data::common::MissionStatus::Idle ||
//        nw_status_ptr_->arm_mission_status == yf::data::common::MissionStatus::Finish)
//    {
//        sql_ptr_->UpdateJobTable(nw_status_ptr_->db_cur_job_log_id, 3);
//        sql_ptr_->UpdateJobLog(nw_status_ptr_->db_cur_job_log_id, 3);
//    }

    if( nw_status_ptr_->arm_mission_status == yf::data::common::MissionStatus::Error ||
        nw_status_ptr_->arm_mission_status == yf::data::common::MissionStatus::EStop)
    {
        // error
        //
        sql_ptr_->UpdateJobTable(nw_status_ptr_->db_cur_job_log_id, 5);
        sql_ptr_->UpdateJobLog(nw_status_ptr_->db_cur_job_log_id, 5);
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
            sleep.ms(1000);
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
//                                tm5.SetMotorHigh();

                                std::string command = "Post arm_via0_line";
                                tm5.ArmTask(command);

//                                tm5.SetMotorLow();
                                break;
                            }
                        }

                        //Post arm_via0_line_force_120N


                        break;
                    }
                    default:
                    {
//                        tm5.SetMotorHigh();

                        std::string command = "Post arm_via0_line";
                        tm5.ArmTask(command);

//                        tm5.SetMotorLow();

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
//                        tm5.SetMotorHigh();

                        std::string command = "Post arm_via45_line_z";
                        tm5.ArmTask(command);

//                        tm5.SetMotorLow();

                        break;
                    }
                    case data::arm::ModelType::DeskRectangle:
                    {
//                        tm5.SetMotorHigh();

                        std::string command = "Post arm_via45_line_d";
                        tm5.ArmTask(command);

//                        tm5.SetMotorLow();

                        break;
                    }
                    case data::arm::ModelType::DeskCircle:
                    {
//                        tm5.SetMotorHigh();

                        std::string command = "Post arm_via45_p2p";
                        tm5.ArmTask(command);

//                        tm5.SetMotorLow();

                        break;
                    }
                    case data::arm::ModelType::DeskPolygon:
                    {
//                        tm5.SetMotorHigh();

                        std::string command = "Post arm_via45_line_z";
                        tm5.ArmTask(command);

//                        tm5.SetMotorLow();

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
//                                tm5.SetMotorHigh();

                                std::string command = "Post arm_via45_line_z";
                                tm5.ArmTask(command);

//                                tm5.SetMotorLow();

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
//                        tm5.SetMotorHigh();

                        std::string command = "Post arm_via45_line_z";
                        tm5.ArmTask(command);

//                        tm5.SetMotorLow();

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
//                                tm5.SetMotorHigh();

                                std::string command = "Post arm_via45_line_z";
                                tm5.ArmTask(command);

//                                tm5.SetMotorLow();

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
//                        tm5.SetMotorHigh();

                        std::string command = "Post arm_via45_line_z";
                        tm5.ArmTask(command);

//                        tm5.SetMotorLow();

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

void yf::sys::nw_sys::ArmPickPad(const int& job_id)
{
    // check each pad_size
    auto pad_type_id = sql_ptr_->GetModelConfigElement(sql_ptr_->GetModelConfigId(job_id),"pad_type");

    // push back respectively.
    switch (pad_type_id)
    {
        case 1: // small_pad
        {
            tm5.ArmTask("Post pick_small_pad");
            tm5.set_cur_pad_type(data::arm::PadType::Small);
            break;
        }
        case 2: // large_pad
        {
            tm5.ArmTask("Post pick_large_pad");
            tm5.set_cur_pad_type(data::arm::PadType::Large);
            break;
        }
    }
}

void yf::sys::nw_sys::ArmRemovePad(const int& job_id)
{
    // check each pad_size
    auto pad_type_id = sql_ptr_->GetModelConfigElement(sql_ptr_->GetModelConfigId(job_id),"pad_type");

    // push back respectively.
    switch (pad_type_id)
    {
        case 1: // small_pad
        {
            tm5.ArmTask("Post remove_small_pad");
            break;
        }
        case 2: // large_pad
        {
            tm5.ArmTask("Post remove_large_pad");
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
    int duration_sec_ugv_status = 2;

    /// each thread's control flag
    web_status_flag_ = true;

    th_web_ugv_status_ = std::thread(&nw_sys::thread_Web_UgvStatus, this,
                                       std::ref(web_status_flag_),duration_sec_ugv_status);

    /// Check Duration: 1 minute
    while (schedule_flag_)
    {
        sleep.minute(1);
    }

    /// Stop all web status threads
    web_status_flag_ = false;
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
        switch (sql_ptr_->GetTaskMode(sql_ptr_->GetModelConfigId(q_ids[n])))
        {
            case 1: // mopping
            {
                q_job_mopping_ids.push_back(q_ids[n]);
                break;
            }
            case 2: // uvc scanning
            {
                q_job_uvc_scanning_ids.push_back(q_ids[n]);
                break;
            }
        }
    }

    // ids filter for small/large pad
    if(!q_job_mopping_ids.empty())
    {
        std::deque<int> small_pad_mopping_ids;
        std::deque<int> large_pad_mopping_ids;

        small_pad_mopping_ids.clear();
        large_pad_mopping_ids.clear();

        for(int n = 0; n < q_job_mopping_ids.size(); n++)
        {
            // check each pad_size
            auto pad_type_id = sql_ptr_->GetModelConfigElement(sql_ptr_->GetModelConfigId(q_job_mopping_ids[n]),
                                                               "pad_type");

            // push back respectively.
            switch (pad_type_id)
            {
                case 1: // small_pad
                {
                    small_pad_mopping_ids.push_back(q_job_mopping_ids[n]);
                    break;
                }
                case 2: // large_pad
                {
                    large_pad_mopping_ids.push_back(q_job_mopping_ids[n]);
                    break;
                }
            }
        }

            // redesign q_job_mopping_ids
            q_job_mopping_ids.clear();

            q_job_mopping_ids.insert(q_job_mopping_ids.end(), small_pad_mopping_ids.begin(), small_pad_mopping_ids.end());
            q_job_mopping_ids.insert(q_job_mopping_ids.end(), large_pad_mopping_ids.begin(), large_pad_mopping_ids.end());
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
        case 0:
        {
            nw_status_ptr_->db_cur_schedule_command_ = data::schedule::ScheduleCommand::None;
            break;
        }

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

        case 4:
        {
            nw_status_ptr_->db_cur_schedule_command_ = data::schedule::ScheduleCommand::RedoCleaningJobErrorPart;
            break;
        }

        case 5:
        {
            nw_status_ptr_->db_cur_schedule_command_ = data::schedule::ScheduleCommand::RedoCleaningJobWhole;
            break;
        }

        case 6:
        {
            nw_status_ptr_->db_cur_schedule_command_ = data::schedule::ScheduleCommand::CustomPlan;
            break;
        }

    }
}

/// Frontend Page "Error Log"

void yf::sys::nw_sys::UpdateDbErrorLog()
{
    // For Arm

    // Connection Error
    switch (nw_status_ptr_->arm_connection_status)
    {
        case data::common::ConnectionStatus::Disconnected:
        {
            //L"手臂未連接，請聯係管理員！"
            sql_ptr_->UpdateErrorLog(2);
            break;
        }
    }

    // Mission Error
    switch (nw_status_ptr_->arm_mission_status)
    {
        case data::common::MissionStatus::Error:
        {
//          "手臂運行錯誤，請聯係管理員！";
            sql_ptr_->UpdateErrorLog(1);
            break;
        }
        case data::common::MissionStatus::EStop:
        {
//          手臂已急停，請聯係管理員！";
            sql_ptr_->UpdateErrorLog(5);
            break;
        }
        default:
        {
            break;
        }
    }

    // For Ugv
}

/// Frontend Page2 Button "小車回原位"

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
    sleep.ms(200);
    mir100_ptr_->Play();

    // DB: update schedule_job & schedule_job_log
}

/// Frontend Page2 Button "手臂回原位"

void yf::sys::nw_sys::DoJobArmBackToHomePos()
{
    // 1. arm back to home position
    // 2. check whether there is a tool
    // 3. if yes, place the corresponding tool

    // Do job
    tm5.ArmTask("Post arm_back_home");

    // if there is any tool attaching on the arm, place it first
    auto arm_current_tool = tm5.GetCurrentTool();

    switch (arm_current_tool)
    {
        case data::arm::Tool::None:
        {
            break;
        }
        case data::arm::Tool::UvcLed:
        {
            tm5.ArmTask("Post place_uvc");
            break;
        }
        case data::arm::Tool::Brush:
        {
            tm5.ArmTask("Post place_mop");
            break;
        }
    }

}

/// Frontend Page4 Button "重做"

void yf::sys::nw_sys::RedoJob(const int &cur_schedule_id, const yf::data::schedule::ScheduleCommand& redo_command)
{
    /// Ugv needs to change the map first!
    mir100_ptr_->ClearErrorState();

    mir100_ptr_->ChangeMapByDBMapStatus();

    sleep.sec(5);

    if(mir100_ptr_->WaitForModeId(7,1))
    {
        mir100_ptr_->ChangeInitPositionByDBMapStatus();
    }

    /// filter, Re-Design the table "ugv_mission_config_redo"

    // get task_group_id
    int task_group_id = sql_ptr_->GetTaskGroupIdFromScheduleTable(cur_schedule_id);

    std::vector<int> redo_ids;

    switch (redo_command)
    {
        case data::schedule::ScheduleCommand::RedoCleaningJobErrorPart:
        {
            redo_ids = sql_ptr_->FillUMCRedoTableErrorPart(task_group_id);
            break;
        }
        case data::schedule::ScheduleCommand::RedoCleaningJobWhole:
        {
            redo_ids = sql_ptr_->FillUMCRedoTableWhole(task_group_id);
            break;
        }
        default:
        {
            return;
        }
    }

    // get job_id from job_log
    int origin_job_id = sql_ptr_->GetJobIdFromJobLog(task_group_id);

    /// 0. Initialization
    //
    nw_status_ptr_->cur_job_success_flag = false;
    bool ugv_mission_success_flag = false;
    bool arm_mission_success_flag = false;

    /// 1. Initialization
    //
    //  1.1 Get cur_model_config_id From DB
    //  1.2 Get related variables
    cur_model_config_id_    = sql_ptr_->GetModelConfigId(origin_job_id);
    cur_mission_num_        = redo_ids.size();
    cur_model_type_         = tm5.GetModelType(cur_model_config_id_);
    cur_task_mode_          = tm5.GetTaskMode(cur_model_config_id_);

    //  1.3 Get Valid Order For Arm Configs
    //  e.g. {0,0,0,1,1,1}
    cur_valid_result_queue_ = sql_ptr_->GetRedoArmConfigValidResultQueue(task_group_id);
    cur_first_valid_order_  = sql_ptr_->GetRedoFirstValidOrder(task_group_id);
    cur_last_valid_order_   = sql_ptr_->GetRedoLastValidOrder(task_group_id);
    //  e.g {3,4,5}
    cur_valid_indexes_ = sql_ptr_->GetRedoValidIndexes(task_group_id);

    int cur_order = 0;
    int pre_order = 0;

    /// 2. Assign Ugv Mission
    //
    //  2.1. Ugv: post a new mission via REST
    mir100_ptr_->PostRedoMission(cur_model_config_id_);
    //  2.2. Ugv: post actions via REST
    mir100_ptr_->PostRedoActions(cur_model_config_id_,redo_ids);
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
                    /// stage 0: check arm config is valid or not
                    /// stage 1: if it is valid, configuring arm mission
                    /// stage 2: execute arm mission

                    bool arm_config_stage_success_flag = false;
                    int last_valid_order;

                    arm_wait_plc003_success_flag = this->WaitForUgvPLCRegisterInt(3,1,5);

                    ///TIME
                    sleep.ms(200);

                    if(arm_wait_plc003_success_flag)
                    {
                        /// get current order

                        while(cur_order == pre_order || cur_order == 0)
                        {
                            cur_order = mir100_ptr_->GetPLCRegisterIntValue(2);

                            ///TIME
                            sleep.ms(200);
                        }

                        pre_order = cur_order;

                        /// check arm config is valid or not

                        sleep.ms(1000);

                        int  cur_arm_config_id = sql_ptr_->GetRedoArmConfigId(task_group_id, cur_order);
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

                                    ///\@ redo part
                                    // b.2 Update current arm_config status as Finished.
                                    sql_ptr_->UpdateEachTaskStatus(redo_ids[cur_order-1], 3);
                                }
                                else
                                {
                                    /// 4.2 False

                                    LOG(INFO) << "Mission failed: ugv failed to set plc001=1 ? Arm needs plc001==1 to execute all arm_mission_configs";

                                    arm_mission_success_flag = false;
                                    mission_continue_flag = false;

                                    /// 4.2.1 Update current arm_config status as Error.
                                    // ...
                                    ///\@ redo part
                                    sql_ptr_->UpdateEachTaskStatus(redo_ids[cur_order-1], 5);

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

                                LOG(INFO) << "configure redo arm mission   [Start!]";
                                auto arm_mission_configs = tm5.ConfigureRedoArmMission(task_group_id, cur_order, cur_model_config_id_);
                                LOG(INFO) << "configure redo arm mission   [Finished!]";

                                cur_operation_area_ =  arm_mission_configs[0].operation_area;

                                // 1. find last valid order
                                // input: cur_order, which is the cur_valid_order!
                                // 2. get cur_valid_order & last_valid_order

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

                                // get last operation area
                                int  last_arm_config_id = sql_ptr_->GetRedoArmConfigId(task_group_id, last_valid_order);
                                auto last_operation_area = tm5.GetOperationArea(last_arm_config_id);

                                /// 2. Notify mir100 that arm configuration stage has finished

                                mir100_ptr_->SetPLCRegisterIntValue(3,0);

                                arm_config_stage_success_flag = true; // no use now?

                                /// 3. Wait for flag for executing all arm_mission_configs
                                arm_wait_plc001_success_flag = this->WaitForUgvPLCRegisterInt(1,1,5);

                                /// 4. Check the flag result
                                if(arm_wait_plc001_success_flag)
                                {
                                    /// 4.1 True
                                    /// 4.1.1 Loop all arm_mission_configs here

                                    ///\workflow Start executing arm mission

                                    ///\ (1) First order, pick the pad
                                    if(cur_order == cur_first_valid_order_)
                                    {
                                        cur_tool_angle_ = data::arm::ToolAngle::Zero;

                                        this->ArmPickTool(cur_task_mode_);

                                        sleep.ms(200);

                                        if(cur_task_mode_ == data::arm::TaskMode::Mopping)
                                        {
                                            this->ArmPickPad(origin_job_id);

                                            sleep.ms(200);

                                            this->ArmUpdatePadNo();

                                            sleep.ms(200);
#if 0 /// Disable For Testing
                                            this->ArmAbsorbWater();
#endif
                                        }
                                    }

                                    ///\ (2) For each order, move to safety position first.
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

                                    ///\brief
                                    /// amc_skip_conditions:
                                    /// 1. For Landmark
                                    ///     1.1. cannot find the landmark
                                    ///     1.2. landmark deviation too large
                                    /// 2. For D455
                                    ///     2.1 cannot match the origin 2d image.
                                    ///     2.2 TF deviation too large
                                    bool amc_skip_flag = true;

                                    //TODO: testing
                                    ///\ (3) Loop all the arm_mission_configs
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
                                            ///   b. Find the TF & Set the amc_skip_flag
                                            ///   c. Return standby_position

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
                                            this->ArmSetToolAngle(cur_task_mode_,arm_mission_configs[n].tool_angle);

                                            /// b. vision job initialization
                                            ///   b.1: for None: do nothing
                                            ///   b.2: for Landmark: scan landmark, mark down the record
                                            ///   b.3: for D455: record the point clouds, mark down the record.

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

                                                            continue;

                                                        }
                                                        else
                                                        {
                                                            LOG(INFO) << "No Deviation!";
                                                            amc_skip_flag = false;
                                                        }
                                                    }
                                                    else
                                                    {
                                                        LOG(INFO) << "Cannot find Landmark! Skip cur_arm_mission_config!!";

                                                        arm_sub_mission_success_flag = false;

                                                        LOG(INFO) << "Skip the whole arm mission configs!";

                                                        ///TIME
                                                        sleep.ms(200);

                                                        continue;
                                                    }

                                                    break;
                                                }
                                                case data::arm::VisionType::D455:
                                                {
                                                    amc_skip_flag = false;

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
                                                            auto result = tm5.RecordCurRealPointCloud(pc_dir, real_pc_file_name);
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
                                                    switch (arm_mission_configs[n].model_type)
                                                    {
                                                        case data::arm::ModelType::Handle:
                                                        {
                                                            auto feature_type = "planar";
                                                            auto feature_type_id = sql_ptr_->GetFeatureTypeId(feature_type);

                                                            std::vector<int> planar_sets;

                                                            // find the corresponding files
                                                            for (int m =0; m < arm_mission_configs[n].feature_type_ids.size(); m++)
                                                            {
                                                                if(arm_mission_configs[n].feature_type_ids[m] == feature_type_id)
                                                                {
                                                                    planar_sets.push_back(m);
                                                                }
                                                            }

                                                            /// for 1 set 1 view algorithm
                                                            if(planar_sets.size() == 1)
                                                            {
                                                                auto set_no  = planar_sets[0];

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
                                                        }
                                                    }

                                                    // 4. set the amc_skip_flag?

                                                    break;
                                                }
                                                case data::arm::VisionType::D435:
                                                {
                                                    break;
                                                }
                                            }

                                            /// c. return standby_position

                                            // back to standby_point
                                            tm5.ArmTask("Move_to standby_p0");
                                        }

                                        /// II:
                                        //  check the amc_skip_flag
                                        //  1. if ture, skip current arm_mission_config
                                        //  2. if false, just execute the arm_mission_config
                                        //    2.1. Initialization
                                        //    2.2. Calculation (base on vision_type: calculate the real_points)
                                        //      2.2.1 new via_points (real_points)
                                        //      2.2.2 new approach_point
                                        //      2.2.3 new n_points
                                        //    2.3. post the arm_mission_config

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
                                            this->ArmSetToolAngle(cur_task_mode_,arm_mission_configs[n].tool_angle);

                                            /// 2.2 Calculation

                                            switch (arm_mission_configs[n].vision_type)
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

                                                    real_via_points = tm5.GetRealViaPointsByRS(arm_mission_configs[n].TMat, arm_mission_configs[n].via_points);

                                                    arm_mission_configs[n].via_points.clear();

                                                    arm_mission_configs[n].via_points = real_via_points;

                                                    // 2.2 calculate the real approach point

                                                    auto real_via_approach_point = tm5.GetRealPointByRS(arm_mission_configs[n].TMat,arm_mission_configs[n].via_approach_pos);

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

                                    sleep.ms(200);

                                    ///\ (4) Finish Current Arm Mission
                                    ///\    a. For Normal orders: Return 'Safety Position'.
                                    ///\    b. For Last order: Place the tool and then return 'Home Position'

                                    // a.
                                    tm5.ArmTask("Post arm_back_to_safety");
                                    sleep.ms(500);

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

                                            this->ArmRemovePad(origin_job_id);
                                        }

                                        // place the tool
                                        this->ArmPlaceTool(cur_task_mode_);

                                        cur_tool_angle_ = data::arm::ToolAngle::Zero;
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
                                        ///\@ redo part
                                        sql_ptr_->UpdateEachTaskStatus(redo_ids[cur_order-1], 5);

                                        continue;
                                    }
                                    else
                                    {
                                        // situation b.
                                        // b.1 Notify mir100 that all arm_mission_configs has finished, you're ready to go :)
                                        mir100_ptr_->SetPLCRegisterIntValue(1,0);

                                        // b.2 Update current arm_config status as Finished.
                                        ///\@ redo part
                                        sql_ptr_->UpdateEachTaskStatus(redo_ids[cur_order-1], 3);
                                    }

                                    if(!arm_sub_mission_success_flag)
                                    {
                                        // no need to break the loop, finish the rest of arm_configs first!

                                        // a.2 Update current arm_config status as as Error.
                                        ///\@ redo part
                                        sql_ptr_->UpdateEachTaskStatus(redo_ids[cur_order-1], 5);
                                    }
                                }
                                else
                                {
                                    /// 4.2 False

                                    LOG(INFO) << "Mission failed: ugv failed to set plc001=1 ? Arm needs plc001==1 to execute all arm_mission_configs";

                                    arm_mission_success_flag = false;
                                    mission_continue_flag = false;

                                    ///\@ redo part
                                    sql_ptr_->UpdateEachTaskStatus(redo_ids[cur_order-1], 5);

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

    //todo: what about table: job_log ???
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

void yf::sys::nw_sys::ArmCheckPadIsEmpty()
{
    // check if there is any small/large pad
    if(!tm5.GetSmallPadIsExist())
    {
        nw_status_ptr_->small_pad_no = 0;
        sql_ptr_->UpdatePadNo("small_pad", nw_status_ptr_->small_pad_no );
    }
    else
    {
        nw_status_ptr_->small_pad_no = nw_status_ptr_->small_pad_total;
        sql_ptr_->UpdatePadNo("small_pad", nw_status_ptr_->small_pad_no );
    }

    if(!tm5.GetLargePadIsExist())
    {
        nw_status_ptr_->large_pad_no = 0;
        sql_ptr_->UpdatePadNo("large_pad", nw_status_ptr_->large_pad_no );
    }
    else
    {
        nw_status_ptr_->large_pad_no = nw_status_ptr_->large_pad_total;
        sql_ptr_->UpdatePadNo("large_pad", nw_status_ptr_->large_pad_no );
    }
}

void yf::sys::nw_sys::ArmUpdatePadNo()
{
    // check if there is any small/large pad
    if(!tm5.GetSmallPadIsExist())
    {
        nw_status_ptr_->small_pad_no = 0;
        sql_ptr_->UpdatePadNo("small_pad", nw_status_ptr_->small_pad_no );
    }
    else
    {
        // get small pad no
        tm5.ArmTask("Post get_small_pad_no");
        nw_status_ptr_->small_pad_no = tm5.GetSmallPadNo();
        sql_ptr_->UpdatePadNo("small_pad", nw_status_ptr_->small_pad_no );
    }

    if(!tm5.GetLargePadIsExist())
    {
        nw_status_ptr_->large_pad_no = 0;
        sql_ptr_->UpdatePadNo("large_pad", nw_status_ptr_->large_pad_no );
    }
    else
    {
        // get large pad no
        tm5.ArmTask("Post get_large_pad_no");
        nw_status_ptr_->large_pad_no = tm5.GetLargePadNo();
        sql_ptr_->UpdatePadNo("large_pad", nw_status_ptr_->large_pad_no );
    }
}

///\brief: Sys needs to meet the below conditions to finish initial checking
/// 1.
/// []system control mode: Auto
/// []ugv: 1. Startup 2. connection normal 3. battery > 25%
/// []arm: 1. Startup 2. connection normal
/// []consumables:
///     1. large_pad     > 0
///     2. small_pad     > 0
///     3. sanitizer     > 20   * sensor???

/**
 * @note Sys needs to meet the below conditions to finish initial checking
 *
 * @brief
 * @param
 * @return
 * */
void yf::sys::nw_sys::WaitSchedulesInitialCheck()
{
    // final flag
    bool init_check_continue_flag = true;

    // sys_control_mode flag
    bool sys_init_check_continue_flag = true;

    // ugv_flag
    bool ugv_init_check_continue_flag = true;

    // arm_flag
    bool arm_init_check_continue_flag = true;
    bool arm_init_status_check_continue_flag = true;
    bool arm_init_position_check_continue_flag = true;

    // consumable_flag
    bool consumables_init_check_continue_flag = true;

    /// Initial Checking Loop.
    //
    while (init_check_continue_flag)
    {
        LOG(INFO) << "[thread_WaitSchedules]: 1.1 update sys control mode";
        GetSysControlMode();

        LOG(INFO) << "[thread_WaitSchedules]: 1.2 update all devices status";

        LOG(INFO) << "[thread_WaitSchedules]: 1.2.1 update arm_connection_status and arm_mission_status";
        tm5.ArmTask("Get Status");
        //tm5.UpdateArmCurMissionStatus();

        LOG(INFO) << "[thread_WaitSchedules]: 1.3 check functions   [Start]";

        LOG(INFO) << "[thread_WaitSchedules]: 1.3.1 check sys_control_mode   [Start]";
        switch (nw_status_ptr_->sys_control_mode_)
        {
            case data::common::SystemMode::Auto:
            {
                LOG(INFO) << "[thread_WaitSchedules]: 1.3.1.1 sys_control_mode: Auto";
                sql_ptr_->UpdateSysAdvice(10);
                sys_init_check_continue_flag = false;
                break;
            }

            case data::common::SystemMode::Manual:
            {
                LOG(INFO) << "[thread_WaitSchedules]: 1.3.1.1 sys_control_mode: Manual";
                LOG(INFO) << "[thread_WaitSchedules]: 1.3.1.1 wait for Auto Mode";

                while (nw_status_ptr_->sys_control_mode_ != data::common::SystemMode::Auto)
                {
                    if(nw_status_ptr_->sys_control_mode_ == data::common::SystemMode::Manual)
                    {
                        if(mir100_ptr_->IsConnected())
                        {
                            sql_ptr_->UpdateSysAdvice(19);
                        }
                        else
                        {
                            sql_ptr_->UpdateSysAdvice(20);
                        }
                    }

                    if(nw_status_ptr_->sys_control_mode_ == data::common::SystemMode::Recovery)
                    {
                        if(nw_status_ptr_->arm_connection_status == data::common::ConnectionStatus::Connected)
                        {
                            sql_ptr_->UpdateSysAdvice(17);
                        }
                        else
                        {
                            sql_ptr_->UpdateSysAdvice(18);
                        }
                    }

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

            case data::common::SystemMode::Recovery:
            {
                // For user wrong operation
                if(nw_status_ptr_->arm_connection_status == data::common::ConnectionStatus::Connected)
                {
                    LOG(INFO) << "[thread_WaitSchedules]: 1.3.1.1 sys_control_mode: Recovery";
                    LOG(INFO) << "[thread_WaitSchedules]: 1.3.1.1 wait for Auto Mode";
                    sql_ptr_->UpdateSysAdvice(17);
                    while (nw_status_ptr_->sys_control_mode_ != data::common::SystemMode::Auto)
                    {
                        if(nw_status_ptr_->sys_control_mode_ == data::common::SystemMode::Manual)
                        {
                            if(mir100_ptr_->IsConnected())
                            {
                                sql_ptr_->UpdateSysAdvice(19);
                            }
                            else
                            {
                                sql_ptr_->UpdateSysAdvice(20);
                            }
                        }

                        if(nw_status_ptr_->sys_control_mode_ == data::common::SystemMode::Recovery)
                        {
                            if(nw_status_ptr_->arm_connection_status == data::common::ConnectionStatus::Connected)
                            {
                                sql_ptr_->UpdateSysAdvice(17);
                            }
                            else
                            {
                                sql_ptr_->UpdateSysAdvice(18);
                            }
                        }

                        // update sys control mode
                        GetSysControlMode();

                        ///TIME
                        sleep.sec(2);
                    }
                }
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
                if( nw_status_ptr_->arm_connection_status == data::common::ConnectionStatus::Connected &&
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
                    sql_ptr_->UpdateSysAdvice(16);

                    if(nw_status_ptr_->sys_control_mode_ == data::common::SystemMode::Recovery)
                    {
                        LOG(INFO) << "[thread_WaitSchedules]: 1.3.2.4 Please play arm project..."; // Ask for recovery mode
                        sql_ptr_->UpdateSysAdvice(18);

                        tm5.WaitForConnection();
                        // Get Status and update to SQL
                        tm5.GetConnectionStatus();
                        tm5.GetMissionStatus();

                        sql_ptr_->UpdateSysAdvice(17);
                        LOG(INFO) << "[thread_WaitSchedules]: 1.3.2 check arm   [Complete]";
                    }
                }
            }

            /// for arm_init_position_check_continue_flag

            arm_init_position_check_continue_flag = false;

            /// overall arm flag checking

            if( arm_init_status_check_continue_flag == false &&
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


        if(nw_status_ptr_->arm_connection_status == data::common::ConnectionStatus::Connected)
        {
            LOG(INFO) << "[thread_WaitSchedules]: 1.3.4 check consumables   [Start]";
            this->ArmCheckPadIsEmpty();

            if(nw_status_ptr_->small_pad_no != 0 && nw_status_ptr_->large_pad_no != 0)
            {
                consumables_init_check_continue_flag = false;
                LOG(INFO) << "[thread_WaitSchedules]: 1.3.4 check consumables   [Complete]";
            }
            else
            {
                LOG(INFO) << "[thread_WaitSchedules]: 1.3.4 check consumables: Please refill consumables!   [Uncompleted]";
                if(nw_status_ptr_->small_pad_no == 0)
                {
                    sql_ptr_->UpdateSysAdvice(13);
                }
                else if (nw_status_ptr_->large_pad_no == 0)
                {
                    sql_ptr_->UpdateSysAdvice(12);
                }

            }
        }
        else
        {
            LOG(INFO) << "[thread_WaitSchedules]: 1.3.4 check consumables   [Skip]: Please check the arm's connection status!";
        }

        LOG(INFO) << "[thread_WaitSchedules]: 1.3 check functions   [Complete]";

        // check the initial_check_flag
        if( sys_init_check_continue_flag == false &&
            arm_init_check_continue_flag == false &&
            ugv_init_check_continue_flag == false &&
            consumables_init_check_continue_flag == false)
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
        sleep.ms(250);
        UpdateDbDeviceArmMissionStatus();
        sleep.ms(200);
    }
}

void yf::sys::nw_sys::thread_Web_UgvStatus(const bool &web_status_flag, const int &sleep_duration)
{
    while(web_status_flag)
    {
        if(mir100_ptr_->IsConnected())
        {
            try
            {
                /// Retrieve ugv status
                auto status = mir100_ptr_->GetUgvStatus();

                /// Update database
                // 1. battery_percentage
                sql_ptr_->UpdateDeviceBatteryCapacity("ugv", status.battery_percentage);
                // 2. position
                sql_ptr_->UpdateDeviceUgvCurPosition(status.position.x,status.position.y,status.position.orientation);
                // 3. ugv_connection_status
                nw_status_ptr_->ugv_connection_status_ = data::common::ConnectionStatus::Connected;
                sql_ptr_->UpdateDeviceConnectionStatus("ugv", 1);
                // 4. ugv_mission_status
                mir100_ptr_->UpdateUgvMissionStatus(status);

            }
            catch (std::error_code& ec)
            {
                std::cerr << "Cannot Get Ugv Status. Will Try Again Later";
            }
        }
        else
        {
            /// update nw_status and database
            // 1. nw_status
            nw_status_ptr_->ugv_connection_status_ = data::common::ConnectionStatus::Disconnected;
            nw_status_ptr_->ugv_mission_status_ = data::common::UgvState::Error;
            // 2. database
            sql_ptr_->UpdateDeviceConnectionStatus("ugv", 0);
            sql_ptr_->UpdateDeviceMissionStatus("ugv",6);
        }

        sleep.sec(sleep_duration);
    }
}

void yf::sys::nw_sys::ArmPlaceToolSafety()
{
    auto arm_current_tool = tm5.GetCurrentTool();

    switch (arm_current_tool)
    {
        case data::arm::Tool::None:
        {
            break;
        }
        case data::arm::Tool::UvcLed:
        {
            tm5.ArmTask("Post place_uvc");
            break;
        }
        case data::arm::Tool::Brush:
        {
            tm5.ArmTask("Post place_mop");
            break;
        }
    }
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

/// For Relative Move

/// Ugv Workflow
///  1. Initialization
//     1.1 Set PLC 001 = 0, PLC 002 = 0, -PLC 003 = 0,   -PLC 004 = 1 (phase 1 static point)
//     1.2 Set PLC 005 = 0, PLC 006 = 0, PLC 007 = 0  (for mir relative move)
//     1.3 Set PLC 101 = 0, (for mir relative move parameters)

    // PLC 002 ++(arm mission order ++).
///  2. Moves to P0_ref.
    // Set PLC 003  = 1.
///  3. Start Relative Move Flag: Set PLC 005 = 1.
///  4. While(PLC 005 != 2 && PLC 005 != 3)
///     a. Wait for PLC 001 = 0; (default PLC 001 == 0)
///     b. While (PLC 006 != 2); (default PLC 006 == 0)
///         b.1. fire the arm. Set PLC 001 = 1.
///         b.2. wait for PLC 001 == 0.
///         b.3  get the relative_move_parameters from PLC 001/002/003, which were calculated from ipc1.
///         b.4  relative move!!!
///         b.5. Counting++ (PLC 007++)
///     c. if (PLC 006 == 2)
///         c.1. get the relative_move_parameters from PLC 001/002/003, which were from DB.
///         c.2. relative move!!!
///         c.3. done.
///         c.4. Set PLC 001 = 1. Notice the arm to return to safety_pos.
///         c.5. Wait for PLC 001 = 0.
///         c.7. Break the loop. set PLC 005 = 2.
///         c.8. Reset PLC 006 == 0.

///  5. Reset PLC 005 = 0; PLC 006 = 0;

///  6。 Moves to P1_ref
///  ...
///  X. Set PLC 004 = 2


/// Arm Workflow

/// For Each Arm Mission Config.
/// 1. Set Continue_Flag.
///   a. Check if PLC 004 = 2
///   c. Check Ugv Status
///   d. Check Arm Status
/// 2. While(Continue_Flag)   --- !( PLC 005 == 2 || PLC 005 == 3)
///   2.1. While(PLC 005 != 2 && PLC 005 != 3)
///     a. if(PLC 001 == 1)
///       a.1 if (PLC 006 == 1).
///          1. Move to vision_pos_ref
///          2. Read LM_real
///          3. Compare with LM_ref, get the difference. (delta x, y, z, rx, ry, rz)
///          4. Set mir relative move params: Set PLC 101/102/103 = xxx...
///          5. Set ugv_rotate_flag. (start or finish?) (PLC 006 == 1 or 2?)
///          6. Move back to Safety_Pos
///       a.2 if (PLC 006 == 2)
///          1. check the arm_mission_config_no.
///          2. move to rmove_approach_point.
///          3. start rmove_force_node. (Arm Running)
///       a.3 check rmove result.
///          1. if everything okay. Mission Handover: Set PLC 001 = 0;
///          2. if Arm error. Set PLC 005 == 3. Ugv Mission Abortion.
///     c. wait 500ms

/// 2. Check PLC 005.