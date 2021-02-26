// simple client .. tm network tcp/ip connection
// send and receive messages at any time...


#include <iostream>
#include "../include/net.h"

#include <boost/algorithm/string/trim.hpp>

enum class CustomMsgTypes : uint32_t
{
    ServerAccept,
    ServerOutput,
    BackToHomePosition,
    MoveToObstacleCheckingPosition,
    MoveToInitVisionPosition,
    MoveToCleanTaskInitPosition,
};

class IPCClient : public yf::net::client_interface<CustomMsgTypes>
{
public:

    void SendMsg(const std::string& str)
    {
        yf::net::message<CustomMsgTypes> msg;

        msg.body.resize(str.size());
        msg.body.assign(str.begin(),str.end());

        Send(msg);
        // update the arm status, will block the code.
    }

};

int main()
{
    IPCClient arm_ln_client;

    arm_ln_client.Connect("192.168.2.29", 5890);

    std::cout << "wait 15s..." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(15000));

    if (arm_ln_client.isConnected())
    {
        std::cout<< "is connected?" << arm_ln_client.isConnected() << std::endl;
        //wait 10s
        std::cout << "wait 10s..." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(10000));

        arm_ln_client.SendMsg("$TMSCT,75,1,\r\n"
                              "float[] targetP1= {-90,0,90,0,90,0}\r\n"
                              "PTP(\"JPP\",targetP1,10,200,0,false),*56\r\n");

        //wait 10s
        std::cout << "wait 10s..." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(10000));

        arm_ln_client.SendMsg("< $TMSCT,78,2,ChangeBase(\"RobotBase\")\r\n"
                              "ChangeTCP(\"NOTOOL\")\r\n"
                              "ScriptExit()\r\n"
                              "ChangeLoad(10.1),*6C\r\n");

        arm_ln_client.Disconnect();
        std::cout << "wait 10s..." << std::endl;
    }
    else
    {
        std::cerr << "Not connection established." << std::endl;
    }



#if 0
    bool bQuit = false;

    while(!bQuit)
    {
        if (arm_ln_client.isConnected())
        {
            if(!arm_ln_client.Incoming().is_empty())
            {
                auto msg = arm_ln_client.Incoming().pop_front().msg;

                std::string s(msg.body.begin(), msg.body.end());
                boost::trim_right(s);

            }
        }
        else
        {
            std::cerr << "Arm didn't EnterListenNode!" << std::endl;
            bQuit = true;
        }
    }
#endif


    return 0;
}