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
                Null    = 0,       // nw_sys initialization
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
                ManualSetting = 2,
                Recovery = 3
                // ArmInitPosition
                // UgvDockingPosition
            };

            enum class UgvState
            {
                Starting        = 1,        // connected
                ShuttingDown    = 2,        // disconnected
                Ready           = 3,        // idle
                Pause           = 4,        // pasue
                Executing       = 5,        // running
                Aborted         = 6,        //
                Completed       = 7,        // finished
                EmergencyStop   = 10,       // EStop
                ManualControl   = 11,       // manual_control
                Error           = 12        // error
            };
        }
    }


}












