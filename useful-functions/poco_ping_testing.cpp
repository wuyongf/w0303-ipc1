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
#include <stdio.h>
#include <time.h>
#include <string.h>


#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
static const std::string slash="\\";
#else
static const std::string slash="/";
#endif

int main()
{
    std::cout << "slash is: " << slash << std::endl;

    return 1;
}