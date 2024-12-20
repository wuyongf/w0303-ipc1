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

#include "../include/al.h"


void TF_duration_test(yf::arm::tm& tm5)
{


    auto tcp_offset_info = tm5.GetTcpOffsetInfo("d455");
    //   1.2. get all ref_tcp_pos.
    auto ref_tcp_pos_ids = tm5.GetRefTcpPosIds(11167);

    yf::algorithm::Timer timer;
    // 2d vector for TF
    auto ref_tcp_pos_tfs = tm5.GetRefTcpPosTFs(ref_tcp_pos_ids,tcp_offset_info);

    return;
}

int main()
{


    yf::arm::tm tm5;

    std::shared_ptr<IPCServer> ipc_server_ptr_ = std::make_shared<IPCServer>(12345);

    std::shared_ptr<yf::status::nw_status> nw_status_ptr_ = std::make_shared<yf::status::nw_status>();

///    rog in office
//    std::shared_ptr<yf::sql::sql_server> sql_ptr = std::make_shared<yf::sql::sql_server>("SQL Server","192.168.7.84","NW_mobile_robot_sys","sa","wuyongfeng1334");
///    ipc1
//
    std::shared_ptr<yf::sql::sql_server> sql_ptr_ = std::make_shared<yf::sql::sql_server>("SQL Server","192.168.7.127","NW_mobile_robot_sys","sa","NWcadcam2021");
///    localhost
//
//    std::shared_ptr<yf::sql::sql_server> sql_ptr_ = std::make_shared<yf::sql::sql_server>("ODBC Driver 17 for SQL Server","localhost","NW_mobile_robot_sys","sa","wuyongfeng1334");

    tm5.Start(ipc_server_ptr_,nw_status_ptr_,sql_ptr_);

    TF_duration_test(tm5);


    auto ids = tm5.GetFeatureTypeIds(11167);


    while (true)
    {
        tm5.ModbusCheckArmStatus();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }



    /// 2d vector for Transformation Matrix
    std::vector<std::vector<Eigen::Matrix4f>> ref_tcp_pos_tfs;
    /// 2d vector for ref_pc_file_name
    std::vector<std::vector<std::string>> ref_pc_file_names;
    /// 2d vector for real_pc_file_name
    std::vector<std::vector<std::string>> real_pc_file_names;
    /// 1d vector for feature type
    std::vector<int> feature_type_ids;

//    auto landmark_flag = tm5.GetLandmarkFlag(1);
//
//    auto tool_angle = tm5.GetToolAngle(1);
//
//    auto via_points = tm5.GetViaPoints(1,yf::data::arm::MotionType::PlaneMotion);
//
//    auto via_points_1 = tm5.GetViaPoints(2,yf::data::arm::MotionType::LineMotion);


    yf::data::arm::Point3d ref_landmark_pos = tm5.GetRefLandmarkPos(3011);

    yf::data::arm::Point3d real_landmark_pos;

    real_landmark_pos.x = -255.33;
    real_landmark_pos.y = -1117.83;
    real_landmark_pos.z = 409.13;
    real_landmark_pos.rx = -88.66;
    real_landmark_pos.ry = -0.07;
    real_landmark_pos.rz = -171.59;

    std::deque<yf::data::arm::Point3d> original_path;

    yf::data::arm::Point3d ori_point_1;
    ori_point_1.x = -358.8814;
    ori_point_1.y = -73.76439;
    ori_point_1.z = 358.9901;
    ori_point_1.rx = -173.859;
    ori_point_1.ry = 0.1401029;
    ori_point_1.rz = -87.89309;

    original_path.push_back(ori_point_1);

    auto real_path = tm5.GetRealViaPointsByLM(original_path, ref_landmark_pos, real_landmark_pos);





}
