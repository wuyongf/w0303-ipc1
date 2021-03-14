/// condition variable and mutex lock

// todo: for block. and notify. ^_^
//  1. define mutex and condition variable
std::mutex mux_Blocking;
std::condition_variable cv_Blocking;

// todo:
//  2. where to block
std::unique_lock<std::mutex> ul(mux_Blocking);
cv_Blocking.wait(ul);

// todo:
//  3. where will I notify.
std::unique_lock<std::mutex> ul(mux_Blocking);
cv_Blocking.notify_one();


/// for time control

#include <chrono>
#include <thread>

std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // wait some time

/// shared pointer

/// for time measurement

// (1)
// https://www.youtube.com/watch?v=YG4jexlSAjc

// (2)
#include <iostream>
#include <thread>
#include <future>
#include <cmath>
#include <vector>
#include <chrono>
//
// start time measurement
std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
// stop time measurement and print execution time
std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
auto duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
std::cout << "Execution finished after " << duration <<" microseconds" << std::endl;

/// catch error
//
#include <exception>

try
{
    ...
}
catch (std::exception& e)
{
std::cout << "Error occurred: " << e.what() << std::endl;
}

/// cout enum class
enum class Status
{
    Idle    = 0,       // No mission					// Normal status
    Running = 1,    // Executing a mission
    Pause   = 2,      // Pause the current mission    // Pause the project/robotic arm
    Finish  = 3,      // Finish a mission
    Error   = 4	     // Error						// Project not running. robotic arm error
};

/// template class

template<typename T>
std::ostream& operator<<(typename std::enable_if<std::is_enum<T>::value, std::ostream>::type& stream, const T& e)
{
    return stream << static_cast<typename std::underlying_type<T>::type>(e);
}

int main()
{
    Status arm_task_status = Status::Idle;


    std::cout << "arm task status is: " << arm_task_status << std::endl;

    return 0;
}