#pragma once

#include <memory>
#include <thread>
#include <mutex>
#include <deque>
#include <optional>  // ?
#include <vector>
#include <iostream>
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace yf
{
    namespace data
    {
        namespace common
        {
            // Status
            // for: schedule, arm task, mir task, vision task...
            enum class MissionStatus
            {
                Null    = 0,
                Idle    = 1,       // No mission				 //waiting	    // Normal status
                Running = 2,       // Executing a mission        //running
                Finish  = 3,       // Pause the current mission  //       // Pause the project/robotic arm
                Pause   = 4,       // Finish a mission
                Cancel  = 5,
                Error   = 6,	   // Error & Failed					// Project not running. robotic arm error
                EStop   = 7        // E-Stop
            };

            enum class ConnectionStatus
            {
                Disconnected    = 0,
                Connected       = 1
            };

            enum class SystemMode
            {
                Auto    = 0,
                Manual  = 1,
                ManualSetting = 2
            };
        }
    }


}












