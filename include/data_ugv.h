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
            struct MapPosition2d
            {
                std::string pos_name;
                float pos_x;
                float pos_y;
                float pos_theta;
            };

            struct UgvBasicInfo
            {
                std::string model_name      = "MiR100";
                std::string model_serial_no = "xxxxxx";
                std::string mir_version     = "xx.xx.xx";
                std::string ip_address      = "xxx.xxx.x.xx";
                uint16_t    server_port_no     = 0;
            };

            enum class UgvConnectionStatus
            {
                Disconnected    = 0,
                Connected       = 1
            };

            //todo: ugv task status
            yf::data::common::MissionStatus UgvTaskStatus;

            struct BasicMotionInfo              // preset...
            {
                int speed;                      //  mm/s
                int acc;                        //  mm/s^2
            };


            // Mission Config
            struct MotionConfig
            {
                std::string                  map_id;
                BasicMotionInfo              basic_motion_info;
                std::vector<MapPosition2d>   mir_positions;
                float                        mir_step_distance;

            };
        }
    }
}








