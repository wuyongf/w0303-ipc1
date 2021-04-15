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

//algorithm
#include "../include/al.h"

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

            /// Modbus Method
            // (~) todo: ipc1 check Arm Connection Status via Modbus
            // (2) ipc1 updates arm connection status from net_server to nw_sys
            // (3) ipc1 updates arm connection status from nw_sys to Database
            void GetConnectionStatus();

            /// Net Method
            // (1) ipc1(net_server) ---> arm: ipc1(net sever) requests "Get Status"
            // (2) ipc1(net_server) <--- arm: arm return current status to ipc1(net_server)
            // (3) ipc1 updates arm connection status from net_server to nw_sys
            // (4) ipc1 updates arm connection status from nw_sys to Database
            void GetMissionStatus();

            bool TryReconnectArm();

            //todo: Arm --- --- Check function 1.
            // function: Initial check the status of Arm.
            void CheckArmInitMssionStaus();

            //todo: Arm --- --- Assign Arm Job. Arm Task...
            void AssignArmMission(const std::string& arm_command_str);

            //todo: Arm --- --- Check function 2.
            // function: keep sending "Get Status" and check the stauts.
            void UpdateArmCurMissionStatus();

        public: // for nw_sys: each mission

            bool InitialStatusCheckForMission(const int& timeout_min);

            std::deque<yf::data::arm::MissionConfig> ConfigureArmMission(const int& model_config_id, const int & order);

            yf::data::arm::ModelType GetModelType(const int& model_config_id);

            yf::data::arm::TaskMode GetTaskMode(const int& model_config_id);

            yf::data::arm::OperationArea GetOperationArea(const int& arm_config_id);

            yf::data::arm::Tool GetCurrentTool();

            yf::data::arm::Tool GetMissionTool(const int& model_config_id);

            yf::data::arm::ToolAngle GetToolAngle(const int& arm_mission_config_id);

            yf::data::arm::MotionType GetMotionType(const int& arm_mission_config_id);

            yf::data::arm::Point3d GetStandbyPosition(const int& arm_config_id);

            bool GetLandmarkFlag(const int& arm_mission_config_id);

            yf::data::arm::Point3d GetInitVisionPosition(const int& arm_mission_config_id);

            yf::data::arm::Point3d GetRefLandmarkPosition(const int& arm_mission_config_id);

            yf::data::arm::Point3d GetViaApproachPoint(const int& arm_mission_config_id);

            std::deque<yf::data::arm::Point3d> GetViaPoints(const yf::data::arm::TaskMode& task_mode, const int& arm_mission_config_id, const yf::data::arm::MotionType& motion_type);



        protected:

            void UpdateSQLArmStatus();
            void UpdateSQLScheduleStatus();

            void RetrieveArmCurrentMissionStatus();

            void NetMessageArm(const std::string& str);

            std::string RetrieveRealPath(const std::string& str);

        private:

            // properties
            const char* tm_ip_address_ = "192.168.7.29";
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
            bool  arm_mission_continue_flag_ = false;

            // Algorithm -- Clean Motion
            yf::algorithm::cleaning_motion al_clean_motion;

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

        private:

            // for arm task

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

// Only check the following status:
//   (1) Error (2) TMProjectRunning (3) EStop
//
void yf::arm::tm::ModbusCheckArmStatus()
{
    if(tm_modbus.read_isError())
    {
        // (1) Update nw_sys arm_mission_status
        nw_status_ptr_->arm_mission_status = yf::data::common::MissionStatus::Error;

        // (2) Update nw_sys arm_connection_status
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

    //
    if(!tm_modbus.read_isProjectRunning())
    {
        LOG(INFO) << "robotic arm project is not running!";

        // (1) Update nw_sys arm_mission_status
        nw_status_ptr_->arm_mission_status = yf::data::common::MissionStatus::Error;

        // (2) Update nw_sys arm_connection_status
        if(nw_status_ptr_->arm_connection_status == yf::data::common::ConnectionStatus::Connected)
        {
            // Notify ipc_sever that the arm client has disconnected.
            NetMessageArm("Get Status");
            // Update nw_sys arm_connection_status
            GetConnectionStatus();
        }
    }

    #if 0 //tm_modbus.read_isPause()
    if(tm_modbus.read_isPause())
    {
        nw_status_ptr_->arm_mission_status = yf::data::common::MissionStatus::Pause;
    }
    #endif
}

// todo: 2021/3/1 wait for reconnection

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
                arm_mission_continue_flag_ = false;

                return;
            }

            case yf::data::common::MissionStatus::EStop:
            {
                LOG(INFO) << "robotic arm has been e-stopped!";
                LOG(INFO) << "todo: update database(task,schedule)...";
                LOG(INFO) << "todo: wait 3 minutes for resume!";

                // The Arm is E-Stop!
                arm_mission_continue_flag_ = false;

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
                arm_mission_continue_flag_ = true;

                // No need to check again, break
                check_flag = false;
                break;
            }

            default:
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                break;
            }
        }
    }
}

void yf::arm::tm::AssignArmMission(const std::string &arm_command_str)
{
    // todo: check the mission flag,
    if(arm_mission_continue_flag_)
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
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
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
                LOG(INFO) << "ipc knows robotic arm is error!";

                LOG(INFO) << "arm mission aborted";
                nw_status_ptr_->cur_task_continue_flag = false;

                return;
            }

            case yf::data::common::MissionStatus::EStop:
            {
                LOG(INFO) << "robotic arm has been e-stopped!";

                LOG(INFO) << "arm mission aborted";
                nw_status_ptr_->cur_task_continue_flag = false;

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

                LOG(INFO) << "arm has been Paused.";
                sql_ptr_->UpdateDeviceMissionStatusLog("arm", 4);
                sql_ptr_->UpdateDeviceMissionStatus("arm", 4);
                sql_ptr_->UpdateTaskData(nw_status_ptr_->db_cur_task_id, 6);
                sql_ptr_->UpdateTaskLog(nw_status_ptr_->db_cur_task_id, 6);

                // start counting 10 minutes
                LOG(INFO) << "wait 10 minute for arm to resume its mission.";
                std::string time_future = sql_ptr_->CountdownTime(sql_ptr_->TimeNow(),10);

                // counting...
                while(sql_ptr_->isFutureTime(time_future, sql_ptr_->TimeNow()))
                {

                    ModbusCheckArmStatus();
                    RetrieveArmCurrentMissionStatus();
                    ///TIME
                    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

                    if(nw_status_ptr_->arm_mission_status != data::common::MissionStatus::Pause)
                    {
                        break;
                    }
                }

                // if wait too long
                if(!sql_ptr_->isFutureTime(time_future, sql_ptr_->TimeNow()))
                {
                    LOG(INFO) << "robotic arm has been paused for 10 minutes. arm mission aborted";
                    nw_status_ptr_->cur_task_continue_flag = false;
                    nw_status_ptr_->arm_mission_status = data::common::MissionStatus::Error;
                    return;
                }

                break;
            }
                // just wait for Idle
            case yf::data::common::MissionStatus::Finish:
            {
                LOG(INFO) << "arm has finished.";

                // arm is working well, just keep checking
                // wait 500ms and loop...
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                break;
            }

                // exit safely
            case yf::data::common::MissionStatus::Idle:
            {
                LOG(INFO) << "arm is idle.";

                // todo:
                //  3. Current arm_task will be regarded as success. just (record and) finish it.
                update_flag = false;

                nw_status_ptr_->cur_task_continue_flag = true;

                break;
            }

            case yf::data::common::MissionStatus::Running:
            {
                // arm is working well, just keep checking
                // wait 500ms and loop...
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
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

bool yf::arm::tm::TryReconnectArm()
{
    ipc_server_ptr_->ClearArmConnection();

    NetMessageArm("Get Status");

    if(nw_status_ptr_->arm_connection_status == data::common::ConnectionStatus::Connected)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void yf::arm::tm::GetMissionStatus()
{
    if(nw_status_ptr_->arm_connection_status == data::common::ConnectionStatus::Connected)
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
    else
    {
        LOG(INFO) << "Arm is not connected, GetMissionStatus() fail!";
    }
}

// check arm mission status
//
bool yf::arm::tm::InitialStatusCheckForMission(const int& timeout_min)
{
    // initialization
    bool update_flag = true;

    std::string time_future = sql_ptr_->CountdownTime(sql_ptr_->TimeNow(),timeout_min);

    while (update_flag)
    {
        // get current arm status
        this->UpdateArmCurMissionStatus();

        if (nw_status_ptr_->arm_mission_status != yf::data::common::MissionStatus::Idle)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        }
        else
        {
            // Arm is ready
            update_flag = false;
            break;
        }

        /// IPC1 waits too long.
        if(!sql_ptr_->isFutureTime(time_future, sql_ptr_->TimeNow()))
        {
            LOG(INFO) << "The Arm is not ready.";
            LOG(INFO) << "The Task failed at the initial status check stage.";

            return false;
        }
    }

    return true;

}

std::deque<yf::data::arm::MissionConfig> yf::arm::tm::ConfigureArmMission(const int& model_config_id, const int & order)
{
    std::deque<yf::data::arm::MissionConfig> arm_mission_configs;

    int arm_config_id = sql_ptr_->GetArmConfigId(model_config_id, order);

    // arm_mission_config_num
    std::deque<int> arm_mission_config_ids = sql_ptr_->GetArmMissionConfigIds(arm_config_id);

    int arm_mission_config_num = arm_mission_config_ids.size();

    for (int n = 0; n < arm_mission_config_num; n++)
    {
        int arm_mission_config_id = arm_mission_config_ids[n];

        // start config each arm_mission

        data::arm::MissionConfig mission_config;

        /// 0. model_type
        mission_config.model_type = this->GetModelType(model_config_id);

        /// 1. task_mode
        mission_config.task_mode = this->GetTaskMode(model_config_id);

        /// 2. operation_area
        mission_config.operation_area = this->GetOperationArea(arm_config_id);

        /// 3. current tool
        mission_config.cur_tool = this->GetCurrentTool();

        /// 4. mission_tool
        mission_config.mission_tool = this->GetMissionTool(model_config_id);

        /// 5. tool_angel
        mission_config.tool_angle = this->GetToolAngle(arm_mission_config_id);

        /// 6. motion_type
        mission_config.motion_type = this->GetMotionType(arm_mission_config_id);

        /// 7. standby_position
        mission_config.standby_position = this->GetStandbyPosition(arm_config_id);

        LOG(INFO) << "standby_position x: " << mission_config.standby_position.x ;
        LOG(INFO) << "standby_position y: " << mission_config.standby_position.y ;
        LOG(INFO) << "standby_position z: " << mission_config.standby_position.z ;
        LOG(INFO) << "standby_position rx: " << mission_config.standby_position.rx ;
        LOG(INFO) << "standby_position ry: " << mission_config.standby_position.ry ;
        LOG(INFO) << "standby_position rz: " << mission_config.standby_position.rz;

        /// 8. landmark_flag
        mission_config.landmark_flag = this->GetLandmarkFlag(arm_mission_config_id);

        /// 9.(optional) init_vision_position
        if(mission_config.landmark_flag == true)
        {
            mission_config.init_vision_pos = this->GetInitVisionPosition(arm_mission_config_id);
        }

        /// 10.(optional) ref_landmark_position
        if(mission_config.landmark_flag == true)
        {
            mission_config.ref_landmark_pos = this->GetRefLandmarkPosition(arm_mission_config_id);
        }

        /// 11.(optional) via_approach_point

        mission_config.via_approach_pos = this->GetViaApproachPoint(arm_mission_config_id);

        LOG(INFO) << "via_approach_pos x: " << mission_config.via_approach_pos.x ;
        LOG(INFO) << "via_approach_pos y: " << mission_config.via_approach_pos.y ;
        LOG(INFO) << "via_approach_pos z: " << mission_config.via_approach_pos.z ;
        LOG(INFO) << "via_approach_pos rx: " << mission_config.via_approach_pos.rx ;
        LOG(INFO) << "via_approach_pos ry: " << mission_config.via_approach_pos.ry ;
        LOG(INFO) << "via_approach_pos rz: " << mission_config.via_approach_pos.rz;

        /// 12. via_points.
        mission_config.via_points = this->GetViaPoints(mission_config.task_mode,arm_mission_config_id,mission_config.motion_type);

        /// 13. n_via_points
        mission_config.n_via_points = mission_config.via_points.size();

        /// 14. mission_order
        mission_config.mission_order = n+1;

        arm_mission_configs.push_back(mission_config);
    }

    return arm_mission_configs;
}

yf::data::arm::TaskMode yf::arm::tm::GetTaskMode(const int &model_config_id)
{
    auto task_mode = sql_ptr_->GetTaskMode(model_config_id);

    switch (task_mode)
    {
        case 1:
        {
            return yf::data::arm::TaskMode::Mopping;
        }

        case 2:
        {
            return yf::data::arm::TaskMode::UVCScanning;
        }
    }
}

yf::data::arm::OperationArea yf::arm::tm::GetOperationArea(const int &arm_config_id)
{
    int operation_area = sql_ptr_->GetOperationArea(arm_config_id);

    switch (operation_area)
    {
        case 0:
        {
            return yf::data::arm::OperationArea::Rear;
        }
        case 1:
        {
            return yf::data::arm::OperationArea::Left;
        }
        case 2:
        {
            return yf::data::arm::OperationArea::Right;
        }
        case 3:
        {
            return yf::data::arm::OperationArea::RearRightCorner;
        }
        case 4:
        {
            return yf::data::arm::OperationArea::RightLower;
        }
        case 5:
        {
            return yf::data::arm::OperationArea::RightHigher;
        }
        case 6:
        {
            return yf::data::arm::OperationArea::RearLeftCorner;
        }
        case 7:
        {
            return yf::data::arm::OperationArea::LeftLower;
        }
        case 8:
        {
            return yf::data::arm::OperationArea::LeftHigher;
        }
    }
}

yf::data::arm::Tool yf::arm::tm::GetCurrentTool()
{
    int is_uvc_tool = tm_modbus.get_control_box_DI(13);
    int is_brush_tool = tm_modbus.get_control_box_DI(14);

    if(is_uvc_tool == 1)
    {
        return yf::data::arm::Tool::UvcLed;
    }
    else if (is_brush_tool == 1)
    {
        return yf::data::arm::Tool::Brush;
    }
    else if (is_uvc_tool == 0 && is_brush_tool == 0)
    {
        return yf::data::arm::Tool::None;
    }
}

yf::data::arm::Tool yf::arm::tm::GetMissionTool(const int& model_config_id)
{
    auto task_mode = this->GetTaskMode(model_config_id);

    switch (task_mode)
    {
        case yf::data::arm::TaskMode::UVCScanning:
        {
            return yf::data::arm::Tool::UvcLed;
        }
        case yf::data::arm::TaskMode::Mopping:
        {
            return yf::data::arm::Tool::Brush;
        }
    }
}

yf::data::arm::ToolAngle yf::arm::tm::GetToolAngle(const int &arm_mission_config_id)
{
    int tool_angle = sql_ptr_->GetToolAngle(arm_mission_config_id);

    switch (tool_angle)
    {
        case 0:
        {
            return yf::data::arm::ToolAngle::Zero;
        }
        case 45:
        {
            return yf::data::arm::ToolAngle::FortyFive;
        }
    }
}

yf::data::arm::MotionType yf::arm::tm::GetMotionType(const int &arm_mission_config_id)
{
    int motion_type = sql_ptr_->GetMotionType(arm_mission_config_id);

    switch (motion_type)
    {
        case 1:
        {
            return yf::data::arm::MotionType::PlaneMotion;
        }
        case 2:
        {
            return yf::data::arm::MotionType::LineMotion;
        }
    }
}

yf::data::arm::Point3d yf::arm::tm::GetStandbyPosition(const int &arm_config_id)
{
    int standby_position_id = sql_ptr_->GetStandbyPositionId(arm_config_id);

    yf::data::arm::Point3d standby_position = sql_ptr_->GetArmPoint(standby_position_id);

    return standby_position;
}

bool yf::arm::tm::GetLandmarkFlag(const int &arm_mission_config_id)
{
    int landmark_flag = sql_ptr_->GetLandmarkFlag(arm_mission_config_id);

    if(landmark_flag == 0)
    {
        return false;
    }
    else
    {
        return true;
    }
}

yf::data::arm::Point3d yf::arm::tm::GetInitVisionPosition(const int &arm_mission_config_id)
{
    int init_vision_position_id = sql_ptr_->GetArmMissionPointId(arm_mission_config_id,"vision_p0");

    yf::data::arm::Point3d init_vision_position = sql_ptr_->GetArmPoint(init_vision_position_id);

    return init_vision_position;
}

yf::data::arm::Point3d yf::arm::tm::GetRefLandmarkPosition(const int &arm_mission_config_id)
{
    int ref_landmark_position_id = sql_ptr_->GetArmMissionPointId(arm_mission_config_id, "ref_landmark_pos");

    yf::data::arm::Point3d ref_landmark_position = sql_ptr_->GetArmPoint(ref_landmark_position_id);

    return ref_landmark_position;
}

yf::data::arm::Point3d yf::arm::tm::GetViaApproachPoint(const int &arm_mission_config_id)
{
    int via_approach_point_id = sql_ptr_->GetArmMissionPointId(arm_mission_config_id, "via_approach_point");

    yf::data::arm::Point3d via_approach_point = sql_ptr_->GetArmPoint(via_approach_point_id);

    return via_approach_point;
}

std::deque<yf::data::arm::Point3d>
yf::arm::tm::GetViaPoints(const yf::data::arm::TaskMode& task_mode, const int &arm_mission_config_id, const yf::data::arm::MotionType &motion_type)
{

    std::deque<yf::data::arm::Point3d> init_cleaning_points = sql_ptr_->GetCleanPoints(arm_mission_config_id, motion_type);

    std::deque<yf::data::arm::Point3d> via_points;

    switch (task_mode)
    {
        case data::arm::TaskMode::Mopping:
        {
            via_points = al_clean_motion.get_via_points(motion_type, init_cleaning_points);
            break;
        }
        case data::arm::TaskMode::UVCScanning:
        {
            via_points = al_clean_motion.get_uvc_via_points(motion_type, init_cleaning_points, 0, 0.2);
            break;
        }
    }

    return via_points;

}

yf::data::arm::ModelType yf::arm::tm::GetModelType(const int &model_config_id)
{
    auto model_id = sql_ptr_->GetModelType(model_config_id);

    switch (model_id)
    {
        case 1:
        {
            return data::arm::ModelType::Handrail;
        }
        case 2:
        {
            return data::arm::ModelType::Chair;
        }
        case 4:
        {
            return data::arm::ModelType::Wall;
        }
        case 5:
        {
            return data::arm::ModelType::Handle;
        }
        case 6:
        {
            return data::arm::ModelType::LiftButton;
        }
        case 7:
        {
            return data::arm::ModelType::Desk;
        }
        case 8:
        {
            return data::arm::ModelType::Skirting;
        }
        case 9:
        {
            return data::arm::ModelType::FootPanel;
        }
        case 10:
        {
            return data::arm::ModelType::Windows;
        }
        case 11:
        {
            return data::arm::ModelType::ProtectiveWall;
        }
        case 12:
        {
            return data::arm::ModelType::Sink;
        }
    }
}
