#pragma once

#include <thread>

//glog
#include <glog/logging.h>

//Net
#include "../include/net.h"
#include "../include/net_w0303_server.h"
#include "../include/net_w0303_client.h"

// Database
#include "../include/sql.h"

// Status
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
        public:
            // sys overall
            //
            // constructor, destructor
            nw_sys(const int& ipc_port);
            virtual ~nw_sys(){}

            // Startup the system
            void Start();

            // Close the system
            void Close();

        public:

            void thread_WaitSchedules();

            void thread_DoSchedules();

            void thread_IPCServerStartup(std::shared_ptr<IPCServer>& server_ptr, bool& server_flag);

        public:

            void WaitSchedulesInitialCheck();

        public:

            void UpdateArmCommand();     // for each arm_config_id
            void ArmTask(const std::string& arm_command);
            void CheckDevicesStatus();

        public:

            void DoJobs(const int& cur_schedule_id);
            void DoTasks(const int& cur_job_id);
            void DoTask(const int& cur_task_id);

            void UpdateDbScheduleBeforeTask (const int& cur_schedule_id);
            void UpdateDbScheduleAfterTask (const int& cur_schedule_id);

            void UpdateDbJobBeforeTask (const int& cur_job_id);
            void UpdateDbJobAfterTask (const int& cur_job_id);

            void UpdateDbTaskBeforeTask (const int& cur_task_id);
            void UpdateDbTaskAfterTask (const int& cur_task_id);

            void UpdateDbDeviceStatusBeforeTask (const int& cur_job_id);
            void UpdateDbDeviceStatusAfterTask (const int& cur_job_id);

        protected: // SQL

            void UpdateDbDeviceArmConnectionStatus();
            void UpdateDbDeviceArmMissionStatus();

            void UpdateDbCurTaskStatusAndLog();

            void UpdateDbCurJobStatusAndLog();

            void GetSysControlMode();

        private:

            // SYS
            // shared_ptr: nw_status
            std::shared_ptr<yf::status::nw_status> nw_status_ptr_;

            // Each Module
            // shared_ptr: sql
            std::shared_ptr<yf::sql::sql_server> sql_ptr_;

            // arm
            yf::arm::tm tm5;

            // ugv
            yf::ugv::mir mir100;

            // ipc2;
            yf::ipc2::raspberry pi4;

            // algorithm
            // yf::al::transformation landmark_trans;

            // tm5.PublishTask("BackToHomePosition");
            // arm_mission_status_ = tm5.GetMissionStatus;
            // schedule_id

            // Schedule
            // database - Schedule - Job - Task current status
            yf::data::common::MissionStatus db_cur_schedule_status_;
            yf::data::common::MissionStatus db_cur_job_status_;
            yf::data::common::MissionStatus db_cur_task_status_;

            // ipc server...
            std::shared_ptr<IPCServer> ipc_server_ptr_;

            // flow control variable
            //
            bool        ipc_server_flag_ = true;
            std::thread th_ipc_server_;

            std::thread th_wait_schedules_;

            std::thread th_do_schedules_;


        private:
            // Overall control --- thread wait schedules and thread do schdules
            //
            // schedule_flag
            bool schedule_flag_ = true;

            //wait schedule
            //
            // (1) control flag
            bool wait_schedule_flag = true;
            // (2) mutex lock
            std::mutex mux_Blocking_wait_schedule;
            std::condition_variable cv_Blocking_wait_schedule;

            // do schedule
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

            std::deque<int> q_task_ids;
            int task_number = 0;

            std::deque<int>  all_avaiable_schedule_ids;
        };
    }
}

yf::sys::nw_sys::nw_sys(const int &ipc_port)
{
    // 0. constructor
    // 1. create shared ptr: ipc_server_ptr
    // 2. assign ipc port for TM network connection.
    ipc_server_ptr_ = std::make_shared<IPCServer>(ipc_port);    //ipc_port: 12345
}

void yf::sys::nw_sys::Start()
{
    // (0) Initialization
    //
    nw_status_ptr_ = std::make_shared<yf::status::nw_status>();
    sql_ptr_       = std::make_shared<yf::sql::sql_server>();

    // (1) thread ipc_server, establish server and keep updating, keep waiting for devices connection.
    th_ipc_server_ = std::thread(&nw_sys::thread_IPCServerStartup, this, std::ref(ipc_server_ptr_), std::ref(ipc_server_flag_));

    // (2) startup tm5 control module and pass <1> ipc_server_ptr, <2> nw_status_ptr <3> sql_ptr
    tm5.Start(ipc_server_ptr_,nw_status_ptr_,sql_ptr_);

    // (3) block the program, wait for arm connection
    // todo(undone): should notify the database!
    tm5.WaitForConnection();

    // update the connection status to sys_status & database ;
    tm5.GetConnectionStatus();

    th_do_schedules_ = std::thread(&nw_sys::thread_DoSchedules, this);

    std::this_thread::sleep_for(std::chrono::milliseconds(500));    // wait 500 ms

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

void yf::sys::nw_sys::ArmTask(const std::string &arm_command)
{
    tm5.CheckArmInitMssionStaus();
    tm5.AssignArmMission(arm_command);
    tm5.UpdateArmCurMissionStatus();
}

void yf::sys::nw_sys::thread_WaitSchedules()
{
    while (schedule_flag_)
    {
        while (wait_schedule_flag)
        {
            LOG(INFO) << "[thread_wait_schedules]: unlock!";

            LOG(INFO) << "[thread_wait_schedules]: stage 0 --- Initial Check [Start]";

            WaitSchedulesInitialCheck();

            LOG(INFO) << "[thread_wait_schedules]: stage 0 --- Initial Check [Complete]";
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

            LOG(INFO) << "[thread_wait_schedules]: stage 1 --- Wait for Database Available Schedules";

            /// prerequisite for front-end. If Devices connection are not all IDLE. Do not Publish Schedule.
            //
            while (schedule_number == 0 )
            {

                LOG(INFO) << "wait for incoming schedules";

                // (1) retrieve schedule number from database...
                q_schedule_ids   = sql_ptr_->GetAvailableScheuldesId();

                schedule_number  = q_schedule_ids.size();

                // (2) update sys control mode
                GetSysControlMode();

                // wait for auto mode.
                while (nw_status_ptr_->sys_control_mode_ != data::common::SystemMode::Auto)
                {
                    LOG(INFO) << "sys is not in auto mode, keep waiting...";

                    // update sys control mode
                    GetSysControlMode();

                    ///TIME
                    // sleep 2s and then keep checking
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                }

                ///TIME
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }

            schedule_number = 0;
            wait_schedule_flag = false;

            // notify thread: wait schedule
            do_schedule_flag = true;
            std::unique_lock<std::mutex> ul(mux_Blocking_do_schedule);
            cv_Blocking_do_schedule.notify_one();

            LOG(INFO) << "[thread_wait_schedules]: unlock thread: do schedule";

            // For sys to shut down
            if(schedule_flag_ == false)
            {
                return;
            }
        }

        // (1) option 1 -- keep looping
        //std::this_thread::sleep_for(std::chrono::milliseconds(2000));

        // (2) option 2 -- mutex lock
        //
        // lock and wait for notify.
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        LOG(INFO) << "[thread_wait_schedules]: lock thread: wait schedule";

        std::unique_lock<std::mutex> ul_wait(mux_Blocking_wait_schedule);
        cv_Blocking_wait_schedule.wait(ul_wait);

        LOG(INFO) << "[thread_wait_schedules]: unlock!";
    }

    LOG(INFO) << "schedule flag is false, thread: wait schedules should be shut down...";
}

void yf::sys::nw_sys::thread_DoSchedules()
{
    // wait for do_schedule_flag = ture
    while (schedule_flag_)
    {
        // Default status is locking....
        while (do_schedule_flag == false)
        {
            LOG(INFO) << "[thread_do_schedules]: lock!";

            std::unique_lock<std::mutex> ul(mux_Blocking_do_schedule);
            cv_Blocking_do_schedule.wait(ul);

            LOG(INFO) << "[thread_do_schedules]: unlock!";
        }

        LOG(INFO) << "thread: do schedule has been unlocked";

        // For sys to shut down
        // (1) Check whether the sys is shutting down.
        if(schedule_flag_ == false)
        {
            return;
        }

        // if there is schedule
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

                    LOG(INFO) << "The Arm has not responded for 3 minutes.";
                    LOG(INFO) << "The Schedule failed at the initial check stage.";
                    //
                    // todo: alert the user... No need, stage 3 will do the job.

                    break;
                }

                time_now = sql_ptr_->TimeNow();

                tm5.UpdateArmCurMissionStatus();
                LOG(INFO) << "schedule initial check: wait for arm is idle";
                std::this_thread::sleep_for(std::chrono::milliseconds(2000));

            }

            // (2) Do Each Schedule

            if(nw_status_ptr_->arm_mission_status == yf::data::common::MissionStatus::Idle)
            {
                auto cur_schedule_id = q_schedule_ids.front();
                q_schedule_ids.pop_front();

                nw_status_ptr_->db_cur_schedule_id = cur_schedule_id;

                // get current id execute time..
                sql_ptr_->GetScheduleExecTime(cur_schedule_id);
                std::string execute_time = sql_ptr_->get_execute_time();
                // wait for execute time...
                sql_ptr_->WaitForExecute(execute_time);

                // TODO: Update Mission Status to database
                UpdateDbScheduleBeforeTask(cur_schedule_id);

                // TODO: Execute the schedule!
                DoJobs(cur_schedule_id);

                // TODO: Update the schedule status to database!
                UpdateDbScheduleAfterTask(cur_schedule_id);
            }
            else
            {
                LOG(INFO) << "abort current schedules...";
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
                LOG(INFO) << "abort current schedules...";
                q_schedule_ids.clear();
            }
        }

        // No more schedules
        //
        // if done successfully
        if(nw_status_ptr_->arm_mission_status == data::common::MissionStatus::Idle)
        {
            LOG(INFO) << "All schedules are done";

            LOG(INFO) << "current time: " << sql_ptr_->TimeNow() << std::endl;
        }
        else
        {
            LOG(INFO) << "Cancel the rest of schedules...";
        }

        // Any way
        //
        // reset do schedule thread to lock
        // wait for notified by wait schedule thread
        do_schedule_flag = false;

        // trigger thread: wait schedule
        wait_schedule_flag = true;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        std::unique_lock<std::mutex> ul_wait(mux_Blocking_wait_schedule);
        cv_Blocking_wait_schedule.notify_one();

        LOG(INFO) << "unlock thread: wait schedule";

        // record the end time
        // update the schedule status to database

    }
}



void yf::sys::nw_sys::DoJobs(const int &cur_schedule_id)
{
    LOG(INFO) << "Do Jobs...";

    q_job_ids = sql_ptr_->GetJobsId(cur_schedule_id);
    job_number = q_job_ids.size();

    // if there is any Job
    //
    while (q_job_ids.size() != 0)
    {

        auto cur_job_id = q_job_ids.front();
        q_job_ids.pop_front();

        nw_status_ptr_->db_cur_job_id = cur_job_id;

        // record the job status
        // TODO: (1) Update Job Log
        sql_ptr_->UpdateJobData(cur_job_id, 2);
        sql_ptr_->UpdateJobLog(cur_job_id, 2);

        // TODO: (2) Execute the job!
        DoTasks(cur_job_id);

        // TODO: (3)
        //  a. Break the loop or not?
        //  b. Record Each Job STATUS.
        bool arm_mission_failed_status = nw_status_ptr_->arm_mission_status == yf::data::common::MissionStatus::Error ||
                                         nw_status_ptr_->arm_mission_status == yf::data::common::MissionStatus::EStop;

        if( nw_status_ptr_->arm_connection_status == data::common::ConnectionStatus::Disconnected || arm_mission_failed_status )
        {
            LOG(INFO) << "abort current jobs...";
            q_job_ids.clear();
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

void yf::sys::nw_sys::DoTasks(const int &cur_job_id)
{
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
}

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
            schedule_status = 2;
            break;
        }

        case yf::data::common::MissionStatus::Idle:
        {
            schedule_status = 2;
            break;
        }

    }

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
        LOG(INFO) << "1. update sys control mode";

        GetSysControlMode();

        LOG(INFO) << "2. updated all devices status";
        LOG(INFO) << "2.1 request arm_connection_status and arm_mission_status from arm";

        tm5.UpdateArmCurMissionStatus();

        LOG(INFO) << "3. check function";

        LOG(INFO) << "3.0 check function: sys_status   [Start]";

        switch (nw_status_ptr_->sys_control_mode_)
        {
            case data::common::SystemMode::Auto:
            {
                LOG(INFO) << "sys_control_mode: Auto";
                sys_init_check_continue_flag = false;
                break;
            }

            case data::common::SystemMode::Manual:
            {
                LOG(INFO) << "sys_control_mode: Manual";
                LOG(INFO) << "wait for Auto Mode";
                while (nw_status_ptr_->sys_control_mode_ != data::common::SystemMode::Auto)
                {
                    // update sys control mode
                    GetSysControlMode();

                    ///TIME
                    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                }
                break;
            }

            case data::common::SystemMode::ManualSetting:
            {
                LOG(INFO) << "sys_control_mode: ManualSetting";
                LOG(INFO) << "wait for Auto Mode";
                while (nw_status_ptr_->sys_control_mode_ != data::common::SystemMode::Auto)
                {
                    // update sys control mode
                    GetSysControlMode();

                    ///TIME
                    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                }
                break;
            }

        }

        LOG(INFO) << "3.1 check function: arm   [Start]";

        if(arm_init_check_continue_flag)
        {
            /// for arm_init_status_check_continue_flag

            if( nw_status_ptr_->arm_connection_status == data::common::ConnectionStatus::Connected &&
                nw_status_ptr_->arm_mission_status    == data::common::MissionStatus::Idle)
            {
                LOG(INFO) << "3.1 check function: arm   [Complete]";
                arm_init_status_check_continue_flag = false;
            }
            else
            {
                // For Arm Paused too long situation
                //
                if ( nw_status_ptr_->arm_connection_status == data::common::ConnectionStatus::Connected &&
                     nw_status_ptr_->arm_mission_status == data::common::MissionStatus::Error)
                {
                    LOG(INFO) << "Please stop robotic arm project...";
                }

                // For Arm Disconnected situation
                //
                if( nw_status_ptr_->arm_connection_status == data::common::ConnectionStatus::Disconnected)
                {
                    LOG(INFO) << "Arm disconnected...";
                    LOG(INFO) << "Please switch to Recovery Mode..."; // Ask for recovery mode

                    if(nw_status_ptr_->sys_control_mode_ == data::common::SystemMode::Recovery)
                    {
                        LOG(INFO) << "Please play arm project..."; // Ask for recovery mode

                        tm5.WaitForConnection();
                        // Get Status and update to SQL
                        tm5.GetConnectionStatus();
                        tm5.GetMissionStatus();
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

        LOG(INFO) << "3.2 check function: ugv   [Start]";
        bool ugv_init_check_continue_flag = false;
        LOG(INFO) << "3.2 check function: ugv   [Complete]";

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

        LOG(INFO) << "4. Update Status to SQL";
        LOG(INFO) << "4.1 Update Status to SQL: Arm";

        UpdateDbDeviceArmConnectionStatus();
        UpdateDbDeviceArmMissionStatus();

        ///TIME
        std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    }
}



















//

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