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
            // system control mode
            yf::data::common::SystemMode            sys_control_mode_ = yf::data::common::SystemMode::Auto;

            // ipc1 status
            yf::data::common::ConnectionStatus      ipc1_connection_status = yf::data::common::ConnectionStatus::Disconnected;

            // arm status
            yf::data::common::ConnectionStatus      arm_connection_status   = yf::data::common::ConnectionStatus::Disconnected;
            yf::data::common::MissionStatus         arm_mission_status      = yf::data::common::MissionStatus::Null;

            // ugv status
            yf::data::common::ConnectionStatus      ugv_connection_status_ = data::common::ConnectionStatus::Disconnected;
            yf::data::common::UgvState              ugv_mission_status_;

            // ipc2--pi4 status
            yf::data::common::ConnectionStatus      ipc2_connection_status_;
            yf::data::common::MissionStatus         ipc2_mission_status_;

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

            /// For Consumables
            int small_pad_no = 5;
            int large_pad_no = 4;

            const int small_pad_total = 5;
            const int large_pad_total = 4;

            /// Ugv
            yf::data::ugv::Status mirStatus;

        };
    }
}


