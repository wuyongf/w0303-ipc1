//
// Created by yongf on 2021/4/12.
//

#include <thread>

//glog
#include <glog/logging.h>

//Net
#include "../include/net.h"
#include "../include/net_w0303_server.h"
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

int main()
{
    yf::arm::tm tm5;

    std::shared_ptr<IPCServer> ipc_server_ptr_ = std::make_shared<IPCServer>(12345);

    std::shared_ptr<yf::status::nw_status> nw_status_ptr_ = std::make_shared<yf::status::nw_status>();
//    std::shared_ptr<yf::sql::sql_server> sql_ptr = std::make_shared<yf::sql::sql_server>("SQL Server","192.168.7.84","NW_mobile_robot_sys","sa","wuyongfeng1334");
    std::shared_ptr<yf::sql::sql_server> sql_ptr_ = std::make_shared<yf::sql::sql_server>("SQL Server","192.168.7.27","NW_mobile_robot_sys","sa","NWcadcam2021");

    tm5.Start(ipc_server_ptr_,nw_status_ptr_,sql_ptr_);

    auto landmark_flag = tm5.GetLandmarkFlag(1);

    auto tool_angle = tm5.GetToolAngle(1);

    auto via_points = tm5.GetViaPoints(1,yf::data::arm::MotionType::PlaneMotion);

    auto via_points_1 = tm5.GetViaPoints(2,yf::data::arm::MotionType::LineMotion);


    yf::data::arm::Point3d standby_pos = tm5.GetViaApproachPoint(1);


    std::cout << "x: " << standby_pos.x <<std::endl;
    std::cout << "y: " << standby_pos.y <<std::endl;
    std::cout << "z: " << standby_pos.z <<std::endl;
    std::cout << "rx: " << standby_pos.rx <<std::endl;
    std::cout << "ry: " << standby_pos.ry <<std::endl;
    std::cout << "rz: " << standby_pos.rz <<std::endl;



}
