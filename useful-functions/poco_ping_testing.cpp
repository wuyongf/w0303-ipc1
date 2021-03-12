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