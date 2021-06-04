#pragma once

#include "data_common.h"

// handle Arm(tm5-900)
// arm_status
// arm_net_status: idle, running, error.
// arm_task_status ; idle, running, error.
//

namespace  yf
{
    namespace data
    {
        namespace arm
        {
            struct BasicInfo
            {
                std::string model_name      = "tm5-900";
                std::string model_serial_no = "AA180152";
                std::string tmflow_version  = "1.80.5300";
                uint16_t    server_port_no  = 12345;
                uint16_t    network_port_no = 12345;
                const char* tm_ip_address_ = "192.168.7.29";
                int   modbus_port_no_ = 502;

            };

            struct Point3d {
                float x;
                float y;
                float z;
                float rx;
                float ry;
                float rz;
            };

            struct Joint3d {
                float j1;
                float j2;
                float j3;
                float j4;
                float j5;
                float j6;
            };

            // Transformation Matrix Info
            struct TransMatrixInfo
            {
                // for phase1: Landmark
                Eigen::Matrix4f T_flange_to_base;           // calculate in realtime.
                Eigen::Matrix4f T_ref_landmark_to_base;
                Eigen::Matrix4f T_real_landmark_to_base;

                // for phase2: Vision module
                Eigen::Matrix4f T_cam_to_flange;
                Eigen::Matrix4f T_ref_init_p_to_cam;
                Eigen::Matrix4f T_real_init_p_to_cam;

                // transformation matrix
                Eigen::Matrix4f T_base_to_ugvbase;
            };

            // tcp coordinate
            struct ControllerInfo
            {
                // flange coordinate
                Point3d flange3d;
                // joint angle
                Joint3d joints;
            };

            struct BasicMotionInfo              // preset...
            {
                int speed   = 150;              //  mm/s
                int acc     = 50;               //  mm/s^2
            };

            enum class CleanType          // for end user choose
            {
                Mopping     = 1,
                UVCScanning = 2
            };

            enum class TaskMode          // for end user choose
            {
                Mopping     = 1,
                UVCScanning = 2
            };

            enum class MotionType
            {
                Plane       = 1,        // 4 points
                Line        = 2,        // 2 points
                CircleFull  = 3,        // 3 points
                CircleHalf  = 4,        // 3 points
                Curve       = 5,        //
                Surface     = 6
            };

            // for operation area
            enum class OperationArea
            {
                Rear            = 0,
                Left            = 1,
                Right           = 2,

                RearRightCorner = 3,
                RightLower      = 4,
                RightHigher     = 5,

                RearLeftCorner  = 6,
                LeftLower       = 7,
                LeftHigher      = 8
            };

            enum class MotionMode
            {
                Null       = 0,
                TMLandMark = 1,
                ListenNode = 2,
                NetCommand = 3
            };

            struct MotionConfig
            {
                // (1)
                CleanType	            clean_type;	                //	cleaning motion type. e.g. Mopping == 1 UVCScanning == 2

                // (2)
                BasicMotionInfo         basic_motion_info;          //  arm speed, acc.
                //
                OperationArea	        ref_operation_area;		    //	robotic arm operation areas. e.g. left hand side(LHS) == 1

                Point3d	                ref_vision_obs_p0;          //  ref: arm init position for pedestrian checking
                Point3d	                ref_vision_p0;      	    //	ref: arm init position for vision task: for locate tm landmark, for d435 scanning

                // todo: phase 1 --- landmark
                //  (1) ref position
                //  (2) ref path
                Point3d                 ref_landmark_pos;           //  ref: landmark position
                std::vector<Point3d>	ref_path;		            //	ref: clean path

                Point3d                 real_landmark_pos;          //  real: landmark position
                std::vector<Point3d>	real_path;		            //	real: clean path

                MotionMode              arm_motion_mode;            //  choose arm motion mode. Landmark or ListenNode or Network.

                // todo: for planar cleaning task. predefine clean length, width, step distance.
                float                   ref_clean_length;           //  cleaning task parameters
                float                   ref_clean_width;            //  cleaning task parameters
                float                   ref_clean_step_distance;    //  cleaning task parameters
                float                   ref_cover_range;            //  arm reachable range, for record
                uint8_t                 clean_times = 0;            //  total clean times, based on the cover range
            };

            enum class Tool
            {
                None        = 0,
                Brush       = 1,
                UvcLed      = 2
            };

            enum class ToolAngle
            {
                Zero        =   0,
                FortyFive   =   45
            };

            enum class ModelType
            {
                Handrail        = 1,
                Chair           = 2,
                Wall            = 4,
                Handle          = 5,
                LiftButton      = 6,
                DeskRectangle   = 7,
                Skirting        = 8,
                FootPanel       = 9,
                Windows         = 10,
                ProtectiveWall  = 11,
                Sink            = 12,
                NurseStation    = 13,

                DeskCircle      = 14,
                DeskPolygon     = 15,

                HandleSurface   = 16
            };

            struct MissionConfig
            {

                ModelType               model_type;

                TaskMode	            task_mode;	                //	cleaning motion type. e.g. Mopping == 1 UVCScanning == 2

                OperationArea           operation_area;

                Tool                    cur_tool;

                Tool                    mission_tool;

                ToolAngle               tool_angle = ToolAngle::Zero;

                MotionType              motion_type;

                Point3d                 standby_position;

                bool                    landmark_flag;

                Point3d                 ref_vision_lm_init_position;

                Point3d                 ref_landmark_pos;

                Point3d                 real_landmark_pos;

                Eigen::Matrix4f         Mat_TF;

                Point3d                 via_approach_pos;

                int                     n_via_points;

                std::deque<Point3d>     via_points;

                int                     mission_order;

            };
        }
    }
}








