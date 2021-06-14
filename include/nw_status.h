#pragma once

#include "data.h"

namespace yf
{
    namespace status
    {
        class nw_status
        {
        public:

            nw_status(){}

            virtual ~nw_status(){}

            /// For Sys
            // system mode: auto mode ---> manual mode
            yf::data::common::SystemMode sys_control_mode_ = yf::data::common::SystemMode::Auto;

            // ipc1 status
            yf::data::common::ConnectionStatus      ipc1_connection_status = yf::data::common::ConnectionStatus::Disconnected;

            // arm status
            yf::data::common::ConnectionStatus      arm_connection_status   = yf::data::common::ConnectionStatus::Disconnected;
            yf::data::common::MissionStatus         arm_mission_status      = yf::data::common::MissionStatus::Null;

            // ugv status
            yf::data::common::ConnectionStatus      ugv_connection_status_;
            yf::data::common::UgvState              ugv_mission_status_;

            // ipc2--pi4 status
            yf::data::common::ConnectionStatus      ipc2_connection_status_;
            yf::data::common::MissionStatus         ipc2_mission_status_;

            // schedule status
            // database - Schedule - Job - Task current status
            yf::data::common::MissionStatus db_cur_schedule_status_;

            yf::data::common::MissionStatus db_cur_job_status_;

            yf::data::common::MissionStatus db_cur_task_status;

            /// DB properties

            int db_cur_schedule_id;

            int db_cur_job_id;

            int db_cur_job_log_id;

            int db_cur_task_group_id;

            bool cur_job_success_flag = false;

            int db_cur_task_id;

            bool cur_task_continue_flag = false;

            /// For A Schedule
            yf::data::schedule::ScheduleCommand db_cur_schedule_command_;

        public:

            int ArmMissionStatusForSQL();

            int ArmConnectionStatusForSQL();


        };
    }
}

int yf::status::nw_status::ArmMissionStatusForSQL()
{
    switch (arm_mission_status)
    {
        case yf::data::common::MissionStatus::Idle:
        {
            return 1;
        }

        case yf::data::common::MissionStatus::Running:
        {
            return 2;
        }

        case yf::data::common::MissionStatus::Finish:
        {
            return 3;
        }

        case yf::data::common::MissionStatus::Pause:
        {
            return 4;
        }

        case yf::data::common::MissionStatus::Cancel:
        {
            return 5;
        }

        case yf::data::common::MissionStatus::Error:
        {
            return 6;
        }

        case yf::data::common::MissionStatus::EStop:
        {
            return 7;
        }
    }
}

int yf::status::nw_status::ArmConnectionStatusForSQL()
{
    switch (arm_connection_status)
    {
        case data::common::ConnectionStatus::Connected:
        {
            return 1;
        }
        case data::common::ConnectionStatus::Disconnected:
        {
            return 0;
        }
    }
}
