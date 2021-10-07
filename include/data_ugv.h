#pragma once

#include "data_common.h"
#include "data_models.h"

// handle Ugv(mir100)
// ugv_status
// ugv_connection_status: idle, running, error.
// ugv_task_status ; idle, running, error.
//

namespace  yf
{
    namespace data
    {
        namespace ugv
        {
            /// modified by YF on 31th Aug, 2021

            struct Position
            {
                float x;
                float y;
                float orientation;
            };

            struct Status
            {
                float       battery_percentage;
                Position    position;
                int         state_id;
                int         mode_id;
            };

            /// previous version

            struct UgvBasicInfo
            {
                std::string robot_name          = "MiR_201803054";
                std::string robot_model         = "MiR100";
                std::string serial_number       = "201803054";
                std::string mir_version         = "2.10.3.1";
                std::string ip_address          = "192.168.7.34";
                uint16_t    server_port_no      = 0;
            };

            struct MapPosition2d
            {
                std::string pos_name;
                float pos_x;
                float pos_y;
                float pos_theta;
            };

            enum class UgvConnectionStatus
            {
                Disconnected    = 0,
                Connected       = 1
            };

            yf::data::common::MissionStatus UgvTaskStatus;

            struct BasicMotionInfo              // preset...
            {
                int speed;                      //  mm/s
                int acc;                        //  mm/s^2
            };

            enum class MissionType
            {
                FixedPosition       =   1,
                RelativeMove        =   2,
            };

            /// yf: using
            struct MissionConfig
            {
                int id;
                /// yf: for relative move
                MissionType mission_type;
            };
        }
    }
}








