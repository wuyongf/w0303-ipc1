#pragma once

#include "data_common.h"
#include "data_arm.h"
#include "data_ugv.h"
#include "data_models.h"

namespace yf
{
    namespace data
    {
        namespace schedule
        {
            enum class Location
            {
                Null    = 0,
                EMSD    = 1,
                HA      = 2,
                HKSTP   = 3
            };

            // A subtask. -- basic unit.
            struct SubTaskForRefModel
            {
                int  sub_task_id = 0;

                bool isFinished = false;         //

                yf::data::common::MissionStatus sub_task_status;          // running, idle, pause, error,....
            };

            struct TaskForRefModel
            {
                // basic info
                int task_id = 0;

                //prepare
                yf::data::models::RefModel                    model_config;

                int arm_motion_config_no;
                std::vector<yf::data::arm::MotionConfig>      arm_motion_configs;

                int ugv_motion_config_no;
                yf::data::ugv::MotionConfig                   ugv_motion_config;

                //process --- record...
                std::vector<SubTaskForRefModel> subtasks;

                //result
                std::vector<bool> sub_task_results;

                yf::data::common::MissionStatus task_status;   // running, idle, pause, finish ....

                void Clear()
                {
                    arm_motion_configs.clear();
                    sub_task_results.clear();
                }
            };

            struct AppointmentInfo
            {
                //where
                Location location = Location::HKSTP;
                int floor = 0;

                // when
                std::string start_time = "2021.1.16 15:00";

            };

            struct Job
            {

                // prepare...
                AppointmentInfo  appointment_config;                 // location, when to start...
                std::string      estimate_end_time;                  // estimate end time

                // propcess...
                std::deque<TaskForRefModel> tasks;
                int task_no;

                // result
                std::string     end_time;		                    // record the actual end time.
                std::vector<bool> task_results;
                yf::data::common::MissionStatus        schedule_status;                    // enum:  idle = 0, running = 1, pause = 2, finish = 3, error =4

                void clear()                                        // for initialization
                {
                    schedule_status = yf::data::common::MissionStatus::Idle;
                    estimate_end_time.clear();
                    end_time.clear();
                }

// Time Function
#if 0
                const std::string currentDateTime() {
                    time_t     now = time(0);
                    struct tm  tstruct;
                    char       buf[80];
                    tstruct = *localtime(&now);
                    // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
                    // for more information about date/time format
                    strftime(buf, sizeof(buf), "%Y-%m-%d %X.000", &tstruct);

                    return buf;
}
#endif


            };

            // Job ---> Tasks
            //                  Task   ---> sub_task
            //                  ---> ugv config
            //                  ---> arm config-1
            //                  ---> arm config-2

        }
    }
}


