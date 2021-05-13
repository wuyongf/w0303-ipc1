//
// Created by yongf on 2021/5/9.
//
#include "../include/net_w0303_client.h"

// Database
#include "../include/sql.h"

// Status
#include "../include/nw_status.h"
// Arm
#include "../include/arm.h"
// UGV
#include "../include/ugv.h"
// IPC2
#include "../include/ipc2.h"

void thread_IPCServerStartup(std::shared_ptr<IPCServer>& server_ptr, bool& server_flag)
{
    server_ptr->Start();

    while (server_flag)
    {
        server_ptr->Update(-1, true);
    }

    server_ptr->Stop();

    return;
}


int main()
{
    /// Initialiazation
    yf::data::arm::Point3d lm_pos;

    yf::arm::tm tm5;

    std::shared_ptr<IPCServer> ipc_server_ptr_ = std::make_shared<IPCServer>(12345);

    std::shared_ptr<yf::status::nw_status> nw_status_ptr_ = std::make_shared<yf::status::nw_status>();

    std::shared_ptr<yf::sql::sql_server> sql_ptr_ = std::make_shared<yf::sql::sql_server>("SQL Server","192.168.7.27","NW_mobile_robot_sys","sa","NWcadcam2021");

    std::thread th_ipc_server_;

    bool server_flag = true;

    th_ipc_server_ = std::thread(&thread_IPCServerStartup, std::ref(ipc_server_ptr_), std::ref(server_flag));

    tm5.Start(ipc_server_ptr_, nw_status_ptr_, sql_ptr_);

    tm5.WaitForConnection();

    tm5.ArmTask("Post vision_find_landmark");

    tm5.ArmTask("Post get_find_landmark_flag");

    if(tm5.GetFindLandmarkFlag() == true)
    {
        LOG(INFO) << "Find Landmark!";

        // 2.4 get real_landmark_pos
        tm5.ArmTask("Post get_landmark_pos_str");

        lm_pos = tm5.GetRealLandmarkPos();
    }
    else
    {
        std::cerr << "Cannot find Landmark!" << std::endl;
    }

    /// Setup DB
    //
    // @@ input: arm_mission_config_id

    int arm_mission_config_id = 2223;

    sql_ptr_->InsertRefLandmarkPos(arm_mission_config_id,lm_pos);

    std::cout << "Done!!!" << std::endl;

    server_flag = false;

    th_ipc_server_.join();

}