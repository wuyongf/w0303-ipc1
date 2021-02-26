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
    // close arm
    tm5.Close();

    // close thread ipc_server
    ipc_server_flag_ = false;
    th_ipc_server_.join();

    // close thread do_schedule, wait_schedule
    schedule_flag_ = false;

    th_wait_schedules_.join();
    th_do_schedules_.join();
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
            // Check whether all device is idle
            // Check whether arm is idle
            while (nw_status_ptr_->arm_mission_status != yf::data::common::MissionStatus::Idle)
            {
                tm5.UpdateArmCurMissionStatus();
                LOG(INFO) << "wait for arm is idle";
                std::this_thread::sleep_for(std::chrono::milliseconds(2000));
            }

            while (schedule_number == 0)
            {
                LOG(INFO) << "wait for incoming schedules";

                //todo: retrieve schedule number from database...
                sql_ptr_->WaitAvailableSchedules();

                q_schedule_ids = sql_ptr_->GetSchedulesId();
                schedule_number = q_schedule_ids.size();

                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }

            schedule_number = 0;
            wait_schedule_flag = false;

            // notify thread: wait schedule
            do_schedule_flag = true;
            std::this_thread::sleep_for(std::chrono::milliseconds(50));

            std::unique_lock<std::mutex> ul(mux_Blocking_do_schedule);
            cv_Blocking_do_schedule.notify_one();

            LOG(INFO) << "unlock thread: do schedule";

        }
        // (1) option 1 -- keep looping
        //std::this_thread::sleep_for(std::chrono::milliseconds(2000));

        // (2) option 2 -- mutex lock
        //
        // lock and wait for notify.
        LOG(INFO) << "lock thread: wait schedule";

        std::unique_lock<std::mutex> ul_wait(mux_Blocking_wait_schedule);
        cv_Blocking_wait_schedule.wait(ul_wait);

        LOG(INFO) << "thread: wait schedule has been unlocked";

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
            LOG(INFO) << "lock thread: do schedule";

            std::unique_lock<std::mutex> ul(mux_Blocking_do_schedule);
            cv_Blocking_do_schedule.wait(ul);
        }
        LOG(INFO) << "thread: do schedule has been unlocked";

        // if there is schedule
        //
        while (q_schedule_ids.size() != 0)
        {
            // (1) Initial check
            //

            // All devices should be idle. Otherwise we should keep checking for 3 minutes.. and then alter user.
            auto time_now = sql_ptr_->TimeNow();
            auto time_countdown = sql_ptr_->CountdownTime(time_now, 3);

            while (nw_status_ptr_->arm_mission_status != yf::data::common::MissionStatus::Idle)
            {
                if(!sql_ptr_->isLatestSchedule(time_countdown,time_now))
                {
                    LOG(INFO) << "The Arm has not responded for 3 minutes.";
                    //
                    // alert the user...
                    sql_ptr_->UpdateDeviceMissionStatus("arm", nw_status_ptr_->ArmMissionStatus());

                    break;
                }
                time_now = sql_ptr_->TimeNow();

                tm5.UpdateArmCurMissionStatus();
                LOG(INFO) << "wait for arm is idle";
                std::this_thread::sleep_for(std::chrono::milliseconds(2000));

            }

            // (2) Do Schedule

            if(nw_status_ptr_->arm_mission_status == yf::data::common::MissionStatus::Idle)
            {
                auto cur_schedule_id = q_schedule_ids.front();
                q_schedule_ids.pop_front();

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
        }

        // No more schedules
        //
        // reset do schedule thread to lock
        // wait for nitified by wait schedule thread
        do_schedule_flag = false;

        // trigger thread: wait schedule
        wait_schedule_flag = true;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        std::unique_lock<std::mutex> ul_wait(mux_Blocking_wait_schedule);
        cv_Blocking_wait_schedule.notify_one();

        LOG(INFO) << "unlock thread: wait schedule";
        LOG(INFO) << "All schedules are done";

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
    while (q_job_ids.size() != 0) {

        auto cur_job_id = q_job_ids.front();
        q_job_ids.pop_front();

        // TODO: Execute the job!
        DoTasks(cur_job_id);

        // TODO: Record Each Job STATUS.

    }

    // No more jobs

    LOG(INFO) << "All Jobs are done";

    auto time_now = sql_ptr_->TimeNow();

    std::cout << "no job now!" << std::endl;
    std::cout << "current time: " << time_now << std::endl;

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
        // (1) Initial check
        //


        // All devices should be idle. Otherwise we should keep checking for 3 minutes.. and then alter user.
        auto time_now = sql_ptr_->TimeNow();
        auto time_countdown = sql_ptr_->CountdownTime(time_now, 3);

        while (nw_status_ptr_->arm_mission_status != yf::data::common::MissionStatus::Idle)
        {
            if(!sql_ptr_->isLatestSchedule(time_countdown,time_now))
            {
                LOG(INFO) << "The Arm has not responded for 3 minutes.";
                //
                // alert the user...
                sql_ptr_->UpdateDeviceMissionStatus("arm", nw_status_ptr_->ArmMissionStatus());
            }
            time_now = sql_ptr_->TimeNow();

            tm5.UpdateArmCurMissionStatus();
            LOG(INFO) << "wait for arm is idle";
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        }


        // (2) Do each Task

        if(nw_status_ptr_->arm_mission_status == yf::data::common::MissionStatus::Idle)
        {
            auto cur_task_id = q_task_ids.front();
            q_task_ids.pop_front();

            // TODO: Update Task Status before task
            sql_ptr_->UpdateTaskLog(cur_task_id, 2);

            // TODO: Execute the task!
            DoTask(cur_task_id);

            // TODO: Update Task Status after task

            if(nw_status_ptr_->arm_mission_status == yf::data::common::MissionStatus::Idle)
            {
                sql_ptr_->UpdateTaskLog(cur_task_id, 3);
            }

            if(nw_status_ptr_->arm_mission_status == yf::data::common::MissionStatus::Error)
            {
                sql_ptr_->UpdateTaskLog(cur_task_id, 5);
            }
        }
        else
        {
            LOG(INFO) << "abort current tasks...";
            q_task_ids.clear();
        }

    }

    // No more jobs
    LOG(INFO) << "All Tasks are done";

    auto time_now = sql_ptr_->TimeNow();

    LOG(INFO) << "no more tasks for job: " << cur_job_id << "now!" << std::endl;
    LOG(INFO) << "current time: " << time_now << std::endl;
}

void yf::sys::nw_sys::DoTask(const int& cur_task_id)
{

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