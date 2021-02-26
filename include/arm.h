#pragma once

#include "data.h"
#include "nw_status.h"

// modbus

#include "modbus_tm_cllient.h"

//Net
#include "../include/net.h"
#include "../include/net_w0303_server.h"
#include "../include/net_w0303_client.h"

//glog
#include <glog/logging.h>

namespace yf
{
    namespace arm
    {
        class tm
        {
        public:
            tm(){};
            virtual ~tm(){}

        public:
            void Start(std::shared_ptr<IPCServer> server_ptr, std::shared_ptr<yf::status::nw_status> status_ptr, std::shared_ptr<yf::sql::sql_server> sql_ptr);

            void Close();

            void ModbusCheckArmStatus();

            void WaitForConnection();

            void GetConnectionStatus();

            //todo: Arm --- --- Check function 1.
            // function: Initial check the status of Arm.
            void CheckArmInitMssionStaus();

            //todo: Arm --- --- Assign Arm Job. Arm Task...
            void AssignArmMission(const std::string& arm_command_str);

            //todo: Arm --- --- Check function 2.
            // function: keep sending "Get Status" and check the stauts.
            void UpdateArmCurMissionStatus();

        protected:

            void UpdateSQLArmStatus();
            void UpdateSQLScheduleStatus();

            void RetrieveArmCurrentMissionStatus();

            void NetMessageArm(const std::string& str);

            std::string RetrieveRealPath(const std::string& str);

        private:
            // properties
            const char* tm_ip_address_ = "192.168.2.29";
            int   network_port_no_;
            int   listen_node_port_no_;
            int   modbus_port_no_ = 502;
            std::string device_name_ = "arm";                           // for sql...

            // shared status
            std::shared_ptr<yf::status::nw_status> nw_status_ptr_;

            // shared database
            std::shared_ptr<yf::sql::sql_server> sql_ptr_;

            // modbus client
            yf::modbus::tm_modbus tm_modbus;

            // Arm Status
            bool  arm_mission_flag_ = false;

            // Net
            //
            // ipc_sever: for Network connection with Arm. (IPC2 in the future)
            std::shared_ptr<IPCServer>    ipc_server_ptr_;                      // tm_net_server
            uint16_t    ipc_port_{};                                            // tm_net_server_port
            bool        ipc_server_flag_ = true;                                // tm_net_server_living_flag
            std::thread th_ipc_server_;                                         // th_tm_net_server

            // Arm Listen Node
            //
            //
            IPCClient   ipc_client_ln_;
            bool        ipc_client_ln_flag_ = true;
            std::thread th_ipc_client_ln_;

            // net_w0303_server.h line202   "arm_ln_status"
            yf::data::common::ConnectionStatus    arm_ln_connection_status_;          // tm_ln_connection_status;

            std::mutex mux_Blocking_ln;
            std::condition_variable cv_Blocking_ln;

            // Job
            //
            //
            yf::data::common::MissionStatus schedule_status_;

            // overall control
            //
            bool schedule_flag = true;

            //wait schedule
            //
            // (1) control flag
            bool wait_schedule_flag = true;
            // (2) mutex lock
            std::mutex mux_Blocking_wait_schedule;
            std::condition_variable cv_Blocking_wait_schedule;

            // do schedule
            //
            // (1) control flag..
            bool do_schedule_flag = false;
            // (2) mutex lock
            std::mutex mux_Blocking_do_schedule;
            std::condition_variable cv_Blocking_do_schedule;

            int schedule_number = 0;
            std::deque<yf::data::schedule::Job> schedule{};
            std::deque<int>  all_avaiable_schedule_ids;

        };
    }
}

void yf::arm::tm::Start(std::shared_ptr<IPCServer> server_ptr, std::shared_ptr<yf::status::nw_status> status_ptr, std::shared_ptr<yf::sql::sql_server> sql_ptr)
{
    ipc_server_ptr_ = server_ptr;
    nw_status_ptr_  = status_ptr;
    sql_ptr_ = sql_ptr;

    tm_modbus.Start(tm_ip_address_, modbus_port_no_);
}

void yf::arm::tm::Close()
{
    ipc_server_ptr_ = nullptr;
    nw_status_ptr_  = nullptr;
    sql_ptr_  = nullptr;

    tm_modbus.Close();
}

void yf::arm::tm::ModbusCheckArmStatus()
{
    if(tm_modbus.read_isError())
    {
        // (1) Update nw_sys arm_mission_status
        nw_status_ptr_->arm_mission_status = yf::data::common::MissionStatus::Error;

        // Can also set arm_connection_status = yf::data::common:ConnectionStatus::Disconnected;
        // however, I need to let ipc_sever know cliet arm is disconnected also!
        // how? by sending random msg!
        // after that, there is no arm_connection_id. need to reconnect!

        // (2) Update nw_sys arm_connection_status
        //
        // if still record connected, should update the connection status
        // if not, do nothing...
        if(nw_status_ptr_->arm_connection_status == yf::data::common::ConnectionStatus::Connected)
        {
            // Notify ipc_sever that the arm client has disconnected.
            NetMessageArm("Get Status");
            // Update nw_sys arm_connection_status
            GetConnectionStatus();
        }

    }

    if(tm_modbus.read_isEStop())
    {
        // (1) Update nw_sys arm_mission_status
        nw_status_ptr_->arm_mission_status = yf::data::common::MissionStatus::EStop;

        // Can also set arm_connection_status = yf::data::common:ConnectionStatus::Disconnected;
        // however, I need to let ipc_sever know cliet arm is disconnected also!
        // how? by sending random msg!
        // after that, there is no arm_connection_id. need to reconnect!

        // (2) Update nw_sys arm_connection_status
        //
        // if still record connected, should update the connection status
        // if not, do nothing...
        if(nw_status_ptr_->arm_connection_status == yf::data::common::ConnectionStatus::Connected)
        {
            // Notify ipc_sever that the arm client has disconnected.
            NetMessageArm("Get Status");
            // Update nw_sys arm_connection_status
            GetConnectionStatus();
        }
    }

    if(tm_modbus.read_isPause())
    {
        nw_status_ptr_->arm_mission_status = yf::data::common::MissionStatus::Pause;
    }

    if(!tm_modbus.read_isProjectRunning())
    {
        LOG(INFO) << "robotic arm project is not starting!";

        // Can also set arm_connection_status = yf::data::common:ConnectionStatus::Disconnected;

    }
    else
    {
        // project is running!!!
        // if recorded as disconnected, should I notice the server to wait for new arm connection????
    }
}

void yf::arm::tm::WaitForConnection()
{

    LOG(INFO) << "wait for arm connection...";
    ipc_server_ptr_->WaitConnectionFromArm();
    LOG(INFO) << "arm connected.";

    // update status
    nw_status_ptr_->arm_connection_status = yf::data::common::ConnectionStatus::Connected;

    ModbusCheckArmStatus();
}

void yf::arm::tm::GetConnectionStatus()
{
    nw_status_ptr_->arm_connection_status = ipc_server_ptr_->GetArmConnectionStatus();

    if(nw_status_ptr_->arm_connection_status == yf::data::common::ConnectionStatus::Connected)
    {
        sql_ptr_->UpdateDeviceConnectionStatus("arm",1);
    }
    else
    {
        sql_ptr_->UpdateDeviceConnectionStatus("arm",0);
    }
}

void yf::arm::tm::CheckArmInitMssionStaus()
{
    bool check_flag = true;

    while (check_flag)
    {
        // 1. modbus check and assign the mission status...(Error, E-Stop)
        ModbusCheckArmStatus();

        switch (nw_status_ptr_->arm_mission_status)
        {
            case yf::data::common::MissionStatus::Error:
            {
                LOG(INFO) << "robotic arm is error!";
                LOG(INFO) << "todo: update database(task,schedule)...";

                // The Arm is Error!
                arm_mission_flag_ = false;

                return;
            }

            case yf::data::common::MissionStatus::EStop:
            {
                LOG(INFO) << "robotic arm has been e-stopped!";
                LOG(INFO) << "todo: update database(task,schedule)...";
                LOG(INFO) << "todo: wait 3-min for resume!";

                // The Arm is E-Stop!
                arm_mission_flag_ = false;

                return;
            }
        }

        // if not error
        RetrieveArmCurrentMissionStatus();

        switch (nw_status_ptr_->arm_mission_status)
        {
            case yf::data::common::MissionStatus::Idle:
            {
                // Everything is fine. The arm is waiting any mission to do. Enjoy.
                arm_mission_flag_ = true;

                // No need to check again, break
                check_flag = false;
                break;
            }

            default:
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                break;
            }
        }
    }
}

void yf::arm::tm::AssignArmMission(const std::string &arm_command_str)
{
    // todo: check the mission flag,
    if(arm_mission_flag_)
    {
        // For listen node --- motion control
        // todo: hard code ---> soft code
        //
        if(arm_command_str == "EnterListenNode")
        {
            // 1. First of all, send the enter command.
            NetMessageArm(arm_command_str);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));        //wait 500ms

            // 2. Start Listen node client. (Now hard code) and update the connection status
            ipc_client_ln_.Connect("192.168.2.29",5890);
            arm_ln_connection_status_ = yf::data::common::ConnectionStatus::Connected;

            std::cout << "listen node connection established." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(500));        //wait 500ms

            // 3. retrieve the real_path, send the real_path and then trigger!
            std::string real_path = RetrieveRealPath("EMSD-GF-Handrail");

            ipc_client_ln_.SendMsg(real_path);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));

            // 4. Exit the listen node.

            std::string ln_exit_str = "$TMSCT,16,1,\r\n"
                                      "ScriptExit(),*62\r\n";
            ipc_client_ln_.SendMsg(ln_exit_str);
            std::this_thread::sleep_for(std::chrono::milliseconds(200));

            // 5. Shut down the listen node connection and then update status.
            std::cout << "listen node connection destroyed." << std::endl;
            ipc_client_ln_.Disconnect();
            arm_ln_connection_status_ = yf::data::common::ConnectionStatus::Disconnected;

        }
        else
        {
            // Normal Arm msg, just send it peacefully.
            NetMessageArm(arm_command_str);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }
    else
    {
        // todo: mission deny.
        LOG(INFO) << "AssignArmMission() deny!";
        std::cout << "From AssignArmMission(): Mission deny!" << std::endl;
        return;
    }
}

void yf::arm::tm::UpdateArmCurMissionStatus()
{
    // 1. check whether the project is error or not.
    // 2. if error, update the schedule status. todo: pause all devices.
    // 3. if not error, send "Get Status" to retrieve current mission status.

    bool update_flag = true;

    while (update_flag)
    {
        // ModbusCheckUgvStatus();

        // 1. modbus check and assign the mission status...(Error, E-Stop)
        ModbusCheckArmStatus();

        switch (nw_status_ptr_->arm_mission_status)
        {
            case yf::data::common::MissionStatus::Error:
            {
                // todo: close arm ???? here???

                LOG(INFO) << "ipc knows robotic arm is error!";
                LOG(INFO) << "todo: update database(task,schedule)...";
                return;
            }

            case yf::data::common::MissionStatus::EStop:
            {
                LOG(INFO) << "robotic arm has been e-stopped!";
                LOG(INFO) << "todo: update database(task,schedule)...";
                LOG(INFO) << "todo: wait 3-min for resume!";

                return;
            }
        }

        // if not Error
        // Send "Get Status" to update the mission status
        RetrieveArmCurrentMissionStatus();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        switch (nw_status_ptr_->arm_mission_status)
        {
            case yf::data::common::MissionStatus::Pause:
            {
                // arm has been paused.
                // loop...
                LOG(INFO) << "arm has been Paused.";
                break;
            }
                // exit safely
            case yf::data::common::MissionStatus::Finish:
            {
                LOG(INFO) << "arm has finished.";

                // todo:
                //  1. Need to record current job, task, etc...
                //  2. Report to database....

                // todo:
                //  3. Current arm_task will be regarded as success. just record and finish it.
                update_flag = false;
                arm_mission_flag_ = false;
                break;
            }

                // exit safely
            case yf::data::common::MissionStatus::Idle:
            {
                LOG(INFO) << "arm is idle.";

                // todo:
                //  3. Current arm_task will be regarded as success. just (record and) finish it.
                update_flag = false;
                arm_mission_flag_ = false;   // cur_task_fail_flag
                break;
            }

            case yf::data::common::MissionStatus::Running:
            {
                // arm is working well, just keep checking
                // wait 500ms and loop...
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                break;
            }
        }
    }
}

void yf::arm::tm::RetrieveArmCurrentMissionStatus()
{
    yf::net::message<CustomMsgTypes> msg;
    std::string str = "Get Status";

    msg.body.resize(str.size());
    msg.body.assign(str.begin(),str.end());

    ipc_server_ptr_->MessageClient(ipc_server_ptr_->GetArmClient(), msg);

    // lock the thread.
    // get status and preserve the status in arm_mission_status_
    nw_status_ptr_->arm_mission_status = ipc_server_ptr_->GetArmMissionStatus();
}


void yf::arm::tm::UpdateSQLArmStatus()
{
    int connection_status;
    int mission_status;

    switch (nw_status_ptr_->arm_connection_status)
    {
        case yf::data::common::ConnectionStatus::Connected:
        {
            connection_status = 1;
            break;
        }

        case yf::data::common::ConnectionStatus::Disconnected:
        {
            connection_status = 0;
            break;
        }
    }

    switch (nw_status_ptr_->arm_mission_status)
    {
        case yf::data::common::MissionStatus::Idle:
        {
            mission_status = 1;
            break;
        }

        case yf::data::common::MissionStatus::Running:
        {
            mission_status = 2;
            break;
        }

        case yf::data::common::MissionStatus::Pause:
        {
            mission_status = 3;
            break;
        }

        case yf::data::common::MissionStatus::Finish:
        {
            mission_status = 4;
            break;
        }

        case yf::data::common::MissionStatus::Error:
        {
            mission_status = 5;
            break;
        }

        case yf::data::common::MissionStatus::EStop:
        {
            mission_status = 6;
            break;
        }
    }

    sql_ptr_->UpdateDeviceConnectionStatus(device_name_, connection_status);
    sql_ptr_->UpdateDeviceMissionStatus(device_name_, mission_status);
}

void yf::arm::tm::UpdateSQLScheduleStatus()
{

}

void yf::arm::tm::NetMessageArm(const std::string &str)
{
    yf::net::message<CustomMsgTypes> msg;

    msg.body.resize(str.size());
    msg.body.assign(str.begin(),str.end());

    // input: arm client, and msg.
    ipc_server_ptr_->MessageClient(ipc_server_ptr_->GetArmClient(), msg);

    LOG(INFO) << "[IPC1 ---> Arm]: " << str;

    return;
}

std::string yf::arm::tm::RetrieveRealPath(const std::string &str)
{
    // todo: hard code now
    std::string real_path = "$TMSCT,78,1,\r\n"
                            "float[] targetP1= {-20,-4,72,14,86,-1}\r\n"
                            "PTP(\"JPP\",targetP1,10,200,0,false),*6b\r\n";
    return real_path;
}








