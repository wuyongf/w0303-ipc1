/// ping function fot windows10


#if 0

#include <iostream>
#include <stdio.h>
#include <time.h>
#include <string.h>

int main(int argc,char *argv[])
{
    // ping /?

    if (system( "ping 192.168.2.113 -n 1 -w 800") )
    {
        printf("internet connx failed \n");
    }
    else {
        printf("internet connx OK ! :) \n");
    }
}

#endif

/// Determine if Linux or Windows in C++

#include <iostream>
#include <string>

#include <chrono>
#include <thread>

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
static const std::string slash="\\";
#else
static const std::string slash="/";
#endif

struct Timer
{
    std::chrono::time_point<std::chrono::steady_clock> start,end;
    std::chrono::duration<float> duration;

    Timer()
    {
        start = std::chrono::high_resolution_clock::now();
    }

    ~Timer()
    {
        end = std::chrono::high_resolution_clock::now();
        duration = end - start;

        float ms = duration.count() * 1000.0f;
        std::cout << "Timer took " << ms << "ms" << std::endl;
    }
};


bool IsArmControlBoxPowerOn()
{
    Timer timer;

    int fail_no = 0;
    int try_no = 3;

    std::string tm_ip_address_ = "192.168.7.34";
    std::string ping_command;
    std::string tm_ip_address = tm_ip_address_;

    //Windows
    //By using win ping method.

    int ping_request_no = 1;
    int ping_timeout = 50; //ms

    ping_command =  "ping " + tm_ip_address +
            " -n " + std::to_string(ping_request_no) +
            " -w " + std::to_string(ping_timeout);

    const char* cstr_ping_command = ping_command.c_str();

    for (int i = 1; i <= try_no; i++)
    {
        if(system( cstr_ping_command ))
        {
            fail_no++;
        }
    }

    if(fail_no == try_no)
    {
        return false;
    }
    else
    {
        return true;
    }
}


int main()
{
    while (1)
    {
        std::cout << "IsConnected? " << IsArmControlBoxPowerOn() << std::endl;

    }


    //std::cout << "slash is: " << slash << std::endl;

    return 1;
}