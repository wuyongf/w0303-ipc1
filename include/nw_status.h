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

            // system mode: auto mode ---> manual mode
            yf::data::common::SystemMode sys_control_mode_ = yf::data::common::SystemMode::Auto;

            // ipc1 status
            yf::data::common::ConnectionStatus      ipc1_connection_status = yf::data::common::ConnectionStatus::Disconnected;

            // arm status
            yf::data::common::ConnectionStatus      arm_connection_status   = yf::data::common::ConnectionStatus::Disconnected;
            yf::data::common::MissionStatus         arm_mission_status      = yf::data::common::MissionStatus::Null;

            // ugv status
            yf::data::common::ConnectionStatus      ugv_connection_status_;
            yf::data::common::MissionStatus         ugv_mission_status_;

            // ipc2--pi4 status
            yf::data::common::ConnectionStatus      ipc2_connection_status_;
            yf::data::common::MissionStatus         ipc2_mission_status_;

            // schedule status
            // database - Schedule - Job - Task current status
            yf::data::common::MissionStatus db_cur_schedule_status_;
            yf::data::common::MissionStatus db_cur_job_status_;

            yf::data::common::MissionStatus db_cur_task_status_;

            bool cur_task_fail_flag = true;

        public:

            int ArmMissionStatus();

        };
    }
}

int yf::status::nw_status::ArmMissionStatus()
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