#pragma once

//glog
#include <glog/logging.h>

#include <boost/algorithm/string/trim.hpp>

#include "../include/net.h"
#include "../include/net_w0303_common.h"

#include "../include/data.h"

#include "modbus_tm_cllient.h"

#include <Poco/Net/ICMPClient.h>

class IPCServer : public yf::net::server_interface<CustomMsgTypes>
{
public:
    IPCServer(uint16_t nPort) : yf::net::server_interface<CustomMsgTypes>(nPort)
    {

    }

//Arm Methods
//
//Getter
public:
    // Status
    //
    // Mission Status
    void set_notify_flag(const bool& boolean);

    void set_thread_modbus_notified();

    void set_thread_modbus_blocked();

    yf::data::common::MissionStatus GetArmMissionStatus()
    {
        // kick off thread to check modbus!
        // 0. establish thread?
        // 1. change flag
        // 2. notify thread to keep checking modbus

        std::unique_lock<std::mutex> ul_arm_status(mux_arm_Blocking);        // create a unique lock, not blocking now.
        cv_arm_Blocking.wait(ul_arm_status);

        return arm_mission_status;
    }

    // Connection Status
    yf::data::common::ConnectionStatus GetArmConnectionStatus()
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        return arm_connection_status;
    }

    //
    std::string GetArmLatestNetMsg()
    {
        if(!arm_net_recv_msgs.empty())
        {
            return arm_net_recv_msgs.back();
        }
        else
        {
            std::cerr << "Not msg from Arm!" << std::endl;
        }
    }

    std::deque<std::string> GetArmAllNetMsg()
    {
        if(!arm_net_recv_msgs.empty())
        {
            return arm_net_recv_msgs;
        }
        else
        {
            std::cerr << "Not msg from Arm!" << std::endl;
        }
    }

    std::shared_ptr<yf::net::connection<CustomMsgTypes>> GetArmClient()
    {
        return client_arm;
    }

    uint32_t GetArmConnectionId()
    {
        return arm_connection_id;
    }

    void WaitConnectionFromArm()
    {
        while(arm_connection_id == 0)
        {
            std::unique_lock<std::mutex> ul(mux_Blocking);        // create a unique lock, not blocking now.
            cv_Blocking.wait(ul);                                  // blocking now! wait for "ul" to be notified.
        }
    }

    void ClearArmConnection()
    {
        int pre_arm_connection_id = arm_connection_id;

        // clear connection id, client ptr, arm net status, arm mission status.
        arm_connection_id = 0;
        client_arm = nullptr;
        arm_connection_status = yf::data::common::ConnectionStatus::Disconnected;
        arm_mission_status = yf::data::common::MissionStatus::Error;

        LOG(INFO) << "Removing Arm. client [" << pre_arm_connection_id << "]";
    }

public:

    // Thread_KeepCheckingModbusAndThenNotifyIPC
    void thread_ModbusWaitForNotifyIPC(bool& thread_continue_flag);

private:

    bool thread_continue_flag_ = true;
    bool notify_flag_ = false;

    // thread: keep checking modbus
    std::mutex mux_modbus_thread_Blocking;
    std::condition_variable cv_modbus_thread_Blocking;

    // modbus client
    yf::modbus::tm_modbus arm_modbus;
    yf::data::arm::BasicInfo arm_basic_info;

    std::thread th_modbus_notify_ipc_;

    int create_once = 0;

private:

    bool IsDeviceAlive(const std::string& ip_address);

private:

    void parse_landmark_pos_str(std::string& msg);
    void parse_small_pad_no_str(std::string& msg);
    void parse_large_pad_no_str(std::string& msg);

public:

    bool get_find_landmark_flag();

    yf::data::arm::Point3d get_landmark_pos();

    int get_small_pad_no();
    int get_large_pad_no();

/// IPC2 Methods
//
public:

    std::shared_ptr<yf::net::connection<CustomMsgTypes>> GetIPC2Client()
    {
        return client_ipc2;
    }

    uint32_t GetIPC2ConnectionId()
    {
        return ipc2_connection_id;
    }

    void WaitConnectionFromIPC2()
    {
        while(ipc2_connection_id == 0)
        {
            std::unique_lock<std::mutex> ul(mux_Blocking);        // create a unique lock, not blocking now.
            cv_Blocking.wait(ul);                                  // blocking now! wait for "ul" to be notified.
        }
    }

protected:

    std::mutex mux_Blocking;
    std::condition_variable cv_Blocking;

    //For Arm
    std::mutex mux_arm_Blocking;
    std::condition_variable cv_arm_Blocking;

    uint32_t arm_connection_id{};
    std::shared_ptr<yf::net::connection<CustomMsgTypes>> client_arm;

    // todo: manage the size of queue. --- (done1/2)
    std::string pre_recv_msg;
    std::deque<std::string> arm_net_recv_msgs;

    yf::data::common::ConnectionStatus arm_connection_status;            // arm network connection status
    yf::data::common::MissionStatus    arm_mission_status;
    yf::data::common::ConnectionStatus arm_ln_status;             // arm listen node connection status

    bool find_landmark_flag = false;
    yf::data::arm::Point3d landmark_pos;

    // for consumables
    int small_pad_no = 0;
    int large_pad_no = 0;

    //for ipc2
    std::shared_ptr<yf::net::connection<CustomMsgTypes>> client_ipc2;
    uint32_t ipc2_connection_id{};
    yf::data::common::ConnectionStatus ipc2_net_status;            // ipc2 network connection status
    yf::data::common::MissionStatus    ipc2_mission_status;

protected:

    virtual bool OnClientConnect(std::shared_ptr<yf::net::connection<CustomMsgTypes>> client)
    {
        // verify which device!
        yf::net::message<CustomMsgTypes> msg;
        std::string str_verify = "which device";
        msg.body.resize(str_verify.size());
        msg.body.assign(str_verify.begin(),str_verify.end());

        client->SendRawMsg(msg);
        return true;
    }

    // Called when a client appears to have disconnected
    virtual void OnClientDisconnect(std::shared_ptr<yf::net::connection<CustomMsgTypes>> client)
    {
        // Clear arm connection id.
        if(client->GetID() == arm_connection_id)
        {
            // clear connection id, client ptr, arm net status, arm mission status.
            arm_connection_id = 0;
            client_arm = nullptr;
            arm_connection_status = yf::data::common::ConnectionStatus::Disconnected;
            arm_mission_status = yf::data::common::MissionStatus::Error;
/// Thread Modbus Never Close??
//            // sleep 2s
//            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
//
//
//            // Modbus thread Join
//            thread_continue_flag_ = false;
//            std::unique_lock<std::mutex> ul_modbus(mux_modbus_thread_Blocking);
//            cv_modbus_thread_Blocking.notify_one();
//            th_modbus_notify_ipc_.join();
//
//            arm_modbus.Close();

            std::cout << "Removing Arm. client [" << client->GetID() << "]\n";
            LOG(INFO) << "Removing Arm. client [" << client->GetID() << "]";
        }

        // Clear ipc2 connection id.
        if(client->GetID() == ipc2_connection_id)
        {
            ipc2_connection_id = 0;
            client_ipc2 = nullptr;
            ipc2_net_status = yf::data::common::ConnectionStatus::Disconnected;
            ipc2_mission_status = yf::data::common::MissionStatus::Error;

            LOG(INFO) << "Removing IPC2. client [" << client->GetID() << "]";
        }
    }

    // Called when a message arrives
    virtual void OnMessage(std::shared_ptr<yf::net::connection<CustomMsgTypes>> client, yf::net::message<CustomMsgTypes>& msg)
    {
        /// Verify each connection with sever.
        //
        // 1. get latest msg

        std::string latest_msg(msg.body.begin(), msg.body.end());
        boost::trim_right(latest_msg);

        // 2. classify each device based on what they return
        auto index_arm     = latest_msg.find("TM5-900");
        auto index_ipc2    = latest_msg.find("IPC2");

        // todo: fine tune the connection verify step. IPC can only accept Arm and IPC2.
        // For first time connection. ipc_server will collect each connection, because (arm_connection_id == 0)
        // (1) ipc_server will collect the client_id.
        // (2) and then ipc_server will return client to our arm_client, ipc2_client for further manipulation.
        if(this->arm_connection_id == 0)
        {
            if (index_arm != std::string::npos)
            {
                // todo: not clear information about whether the connection is robust and stable or not.
                //  (1) Need to check by experience.
                // Assign client id to each device.
                //
                this->arm_connection_id = client->GetID();
                client_arm = client;
                arm_connection_status = yf::data::common::ConnectionStatus::Connected;

                if(create_once == 0)
                {
                    // Modbus
                    arm_modbus.Start(arm_basic_info.tm_ip_address_, arm_basic_info.modbus_port_no_);
                    // Modbus thread
                    th_modbus_notify_ipc_ = std::thread(&IPCServer::thread_ModbusWaitForNotifyIPC, this, std::ref(thread_continue_flag_));

                    //
                    create_once++;
                }

                // Notify WaitForConnectionFromArm()
                std::unique_lock<std::mutex> ul(mux_Blocking);
                cv_Blocking.notify_one();
            }
        }

        if(this->ipc2_connection_id == 0)
        {
            if (index_ipc2 != std::string::npos)
            {
                this->ipc2_connection_id = client->GetID();
                client_ipc2 = client;
                ipc2_net_status = yf::data::common::ConnectionStatus::Connected;

                std::unique_lock<std::mutex> ul(mux_Blocking);
                cv_Blocking.notify_one();
            }
        }

        /// Msg for Arm, Update Arm Status.
        //  1. preserve latest no repeated Network msg in Q --- arm_net_recv_msgs
        //  2. Parse received msg
        //  3. Update the arm status...
        if(this->arm_connection_id == client->GetID())
        {
            // 1. Arm msg log
            if(pre_recv_msg != latest_msg)
            {
                //  push back to msg deque
                arm_net_recv_msgs.push_back(latest_msg);

                pre_recv_msg = latest_msg;

                LOG(INFO) << "[IPC1 <--- Arm]: " << latest_msg ;
            }


            // 2. Arm msg flag
            // 2.1 Status
            auto index_error    = latest_msg.find("Arm Error");
            auto index_running  = latest_msg.find("Arm Running");
            auto index_idle     = latest_msg.find("Arm Idle");
            auto index_pause    = latest_msg.find("Arm Pause");
            auto index_finish   = latest_msg.find("Arm Finish");
            // 2.2 Arm Info
            auto index_find_landmark_flag   = latest_msg.find("find_landmark_flag = ");
            auto index_landmark_pos_str     = latest_msg.find("landmark_pos_str = ");
            auto index_small_pad_no         = latest_msg.find("small_pad_no = ");
            auto index_large_pad_no         = latest_msg.find("large_pad_no = ");

//            ///TIME
//            std::this_thread::sleep_for(std::chrono::milliseconds(50)); // wait 50 ms

            // 3.
            // 3.1 Status
            // for error status
            if (index_error != std::string::npos)
            {
                //Update Arm Network Status
                arm_mission_status = yf::data::common::MissionStatus::Error;
                std::unique_lock<std::mutex> ul_arm_status(mux_arm_Blocking);
                cv_arm_Blocking.notify_one();
                LOG(INFO) << "[net_w0303_server.h] ipc1 detected error command!" << std::endl;
            }
            // for running status
            else
            if (index_running != std::string::npos)
            {
                // update listen node connection status to IPC1
                auto index_EnterListenNode  = latest_msg.find("EnterListenNode");

                if (index_EnterListenNode != std::string::npos)
                {
                    arm_ln_status = yf::data::common::ConnectionStatus::Connected;
                }

                //update arm network status
                arm_mission_status = yf::data::common::MissionStatus::Running;
                std::unique_lock<std::mutex> ul_arm_status(mux_arm_Blocking);
                cv_arm_Blocking.notify_one();
                std::cout << "[net_w0303_server.h] ipc1 knew robot is running! " << std::endl;
            }
            // for idle status
            else
            if (index_idle != std::string::npos)
            {
                //update arm network status
                arm_mission_status = yf::data::common::MissionStatus::Idle;
                std::unique_lock<std::mutex> ul_arm_status(mux_arm_Blocking);
                cv_arm_Blocking.notify_one();

                std::cout << "[net_w0303_server.h] ipc1 knew robot is idle! " << std::endl;
            }
            // for pause status
            else
            if  (index_pause != std::string::npos)
            {
                //update arm network status
                arm_mission_status = yf::data::common::MissionStatus::Pause;
                std::unique_lock<std::mutex> ul_arm_status(mux_arm_Blocking);
                cv_arm_Blocking.notify_one();
                std::cout << "[net_w0303_server.h] ipc1 knew robot has been paused! " << std::endl;

            }
            // for finish status
            else
            if(index_finish != std::string::npos)
            {
                // update listen node connection status to IPC1
                auto index_EnterListenNode  = latest_msg.find("EnterListenNode");

                if (index_EnterListenNode != std::string::npos)
                {
                    arm_ln_status = yf::data::common::ConnectionStatus::Disconnected;
                }

                //update arm network status
                arm_mission_status = yf::data::common::MissionStatus::Finish;
                std::unique_lock<std::mutex> ul_arm_status(mux_arm_Blocking);
                cv_arm_Blocking.notify_one();
                std::cout << "[net_w0303_server.h] ipc1 knew robot task has finished! " << std::endl;
            }

            // 3.2 Arm Info
            if (index_find_landmark_flag != std::string::npos)
            {
                auto index_true = latest_msg.find("true");

                if (index_true != std::string::npos)
                {
                    find_landmark_flag = true;
                } else
                {
                    find_landmark_flag = false;
                }
            }

            if(index_landmark_pos_str != std::string::npos)
            {
                this->parse_landmark_pos_str(latest_msg);
            }

            // 3.3 consumables info
            if(index_small_pad_no != std::string::npos)
            {
                this->parse_small_pad_no_str(latest_msg);
            }

            if(index_large_pad_no != std::string::npos)
            {
                this->parse_large_pad_no_str(latest_msg);
            }
        }
    }
};


void IPCServer::parse_landmark_pos_str(std::string &msg)
{
    std::deque<std::string> q_landmark_pos;

    auto index_left_curly_bracket = msg.find("{");
    auto index_right_curly_bracket = msg.find("}");

    auto s = msg.substr(index_left_curly_bracket+1,index_right_curly_bracket-1 );

    std::string delimiter = ",";

    size_t pos = 0;
    std::string token;
    while ((pos = s.find(delimiter)) != std::string::npos) {
        token = s.substr(0, pos);
//              std::cout << token << std::endl;
        q_landmark_pos.push_back(token);
        s.erase(0, pos + delimiter.length());
    }
    // last token
//          std::cout << s << std::endl;
    q_landmark_pos.push_back(s);

    landmark_pos.x = std::stof(q_landmark_pos[0]);
    landmark_pos.y = std::stof(q_landmark_pos[1]);
    landmark_pos.z = std::stof(q_landmark_pos[2]);
    landmark_pos.rx = std::stof(q_landmark_pos[3]);
    landmark_pos.ry = std::stof(q_landmark_pos[4]);
    landmark_pos.rz = std::stof(q_landmark_pos[5]);

    return;
}

bool IPCServer::get_find_landmark_flag()
{
    return find_landmark_flag;
}

yf::data::arm::Point3d IPCServer::get_landmark_pos()
{
    return landmark_pos;
}

void IPCServer::thread_ModbusWaitForNotifyIPC(bool& thread_continue_flag)
{
    // th_continue_flag
    while(thread_continue_flag)
    {
        while(notify_flag_ == true)
        {
            // keep checking modbus every 10ms
            if(arm_modbus.read_isEStop() || arm_modbus.read_isError() || !arm_modbus.read_isProjectRunning() ||
            !IsDeviceAlive(arm_basic_info.tm_ip_address_))
            {
                //
                std::cout << "[net_w0303_server.h]: Got it!!! Arm is Error!!!"<<std::endl;

                // if error, notify GetArmMissionStatus()
                std::unique_lock<std::mutex> ul_arm_status(mux_arm_Blocking);
                cv_arm_Blocking.notify_one();

                // sleep 200ms
                std::this_thread::sleep_for(std::chrono::milliseconds(200));

                // block the thread itself
                notify_flag_ = false;
            }

        }

//        std::cout << "thread modbus lock!!!"<< std::endl;
        // block, wait for notify
        std::unique_lock<std::mutex> ul_modbus_thread(mux_modbus_thread_Blocking);        // create a unique lock, not blocking now.
        cv_modbus_thread_Blocking.wait(ul_modbus_thread);
//        std::cout << "thread modbus unlock!!!"<< std::endl;

    }



    // block
}

void IPCServer::set_notify_flag(const bool &boolean)
{
    notify_flag_ = boolean;
}

void IPCServer::set_thread_modbus_notified()
{
    notify_flag_ = true;

    std::unique_lock<std::mutex> ul_modbus_thread(mux_modbus_thread_Blocking);
    cv_modbus_thread_Blocking.notify_one();

}

void IPCServer::set_thread_modbus_blocked()
{
    notify_flag_ = false;
}

void IPCServer::parse_small_pad_no_str(std::string &msg)
{
    auto index_left_curly_bracket = msg.find("{");
    auto index_right_curly_bracket = msg.find("}");

    auto s = msg.substr(index_left_curly_bracket+1,index_right_curly_bracket-1 );

    small_pad_no = std::stoi(s);

    return;
}

void IPCServer::parse_large_pad_no_str(std::string &msg)
{
    auto index_left_curly_bracket = msg.find("{");
    auto index_right_curly_bracket = msg.find("}");

    auto s = msg.substr(index_left_curly_bracket+1,index_right_curly_bracket-1 );

    large_pad_no = std::stoi(s);

    return;
}

int IPCServer::get_small_pad_no()
{
    return small_pad_no;
}

int IPCServer::get_large_pad_no()
{
    return large_pad_no;
}

bool IPCServer::IsDeviceAlive(const std::string &ip_address)
{
    Poco::Net::AddressFamily family;

    Poco::Net::ICMPClient icmpClient(family.IPv4);

    auto result = icmpClient.ping(ip_address,3);

    if(result == 0)
    {
        return false;
    }
    else
    {
        return true;
    }
}
