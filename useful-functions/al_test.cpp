//
// Created by yongf on 2021/4/28.
//

#include "../include/al.h"

#include <filesystem>

int main()
{
    yf::algorithm::arm_path al;

    yf::data::arm::Point3d ref_landmark_pos;
    yf::data::arm::Point3d real_landmark_pos;

    ref_landmark_pos.x = -555.3;
    ref_landmark_pos.y = -214.55;
    ref_landmark_pos.z = 2.74;
    ref_landmark_pos.rx = 178.9;
    ref_landmark_pos.ry = 7.16;
    ref_landmark_pos.rz = 95.59;

    real_landmark_pos.x = -624.77;
    real_landmark_pos.y = -39.14;
    real_landmark_pos.z = 0.21;
    real_landmark_pos.rx = 179.97;
    real_landmark_pos.ry = 0.98;
    real_landmark_pos.rz = 94.44;

    Eigen::Matrix4f T_ref_tag;      //T1
    Eigen::Matrix4f T_real_tag;     //T4

    T_ref_tag.setZero();
    T_real_tag.setZero();

    T_ref_tag = al.points2TMat(ref_landmark_pos);      // T1
    T_real_tag = al.points2TMat(real_landmark_pos);    // T4

    Eigen::Matrix4f T_deltaa = T_real_tag * T_ref_tag.inverse() ;

    std::cout << "T_deltaa" << std::endl << T_deltaa << std::endl;

/// demo handle testing
    std::string real_pc_file = "..\\data\\point_clouds\\real\\arm_mission_config_11167\\task_group_6224\\point_cloud\\6224-11167-1-1-planar.pcd";
    std::string ref_pos_tf_file = "..\\data\\point_clouds\\real\\arm_mission_config_11167\\task_group_6224\\tf\\6224-11167-1-1-planar-tf.txt";

//    auto TMat1 = al.Phase2GetTMat4Handle(real_pc_file,ref_pos_tf_file);

    Eigen::Matrix4f TMat1;
    TMat1 <<
    0.991383, 0.130997, 0, 11.0602,
    -0.130997, 0.991383, 0, 173.274,
    0, 0, 1, 0,
    0, 0, 0, 1;

    Eigen::Matrix4f TMat_delta;
    TMat_delta <<
    0, 0, 0, -47.5,
    0, 0, 0, 125.23,
    0, 0, 0, 0,
    0, 0, 0, 0;

//    TMat1 = TMat1 - TMat_delta;

    yf::data::arm::Point3d pos_via;
    pos_via.x = -607.0776;
    pos_via.y = -83.38239;
    pos_via.z = 686.2543;
    pos_via.rx = 128.3404;
    pos_via.ry = 2.061164;
    pos_via.rz = -70.3384;

    yf::data::arm::Point3d pos_1;
    pos_1.x = -641.461;
    pos_1.y = -86.476;
    pos_1.z = 645.393;
    pos_1.rx = 130.573;
    pos_1.ry = 2.056;
    pos_1.rz = -71.029;

    yf::data::arm::Point3d pos_2;
    pos_2.x = -670.471;
    pos_2.y = 17.467;
    pos_2.z = 632.146;
    pos_2.rx = 126.768;
    pos_2.ry = 2.234;
    pos_2.rz = -75.474;

    auto real_pos_via = al.GetRealPointByRS(TMat1,pos_via);
    auto real_pos_1 = al.GetRealPointByRS(TMat1,pos_1);
    auto real_pos_2 = al.GetRealPointByRS(TMat1,pos_2);

    std::cout << "real_pos_via: "<< std::endl;
    std::cout << "x: " << real_pos_via.x << std::endl;
    std::cout << "y: " << real_pos_via.y << std::endl;
    std::cout << "z: " << real_pos_via.z << std::endl;
    std::cout << "rx: " << real_pos_via.rx << std::endl;
    std::cout << "ry: " << real_pos_via.ry << std::endl;
    std::cout << "rz: " << real_pos_via.rz << std::endl;

    std::cout << "real_pos_1: "<< std::endl;
    std::cout << "x: " << real_pos_1.x << std::endl;
    std::cout << "y: " << real_pos_1.y << std::endl;
    std::cout << "z: " << real_pos_1.z << std::endl;
    std::cout << "rx: " << real_pos_1.rx << std::endl;
    std::cout << "ry: " << real_pos_1.ry << std::endl;
    std::cout << "rz: " << real_pos_1.rz << std::endl;

    std::cout << "real_pos_2: "<< std::endl;
    std::cout << "x: " << real_pos_2.x << std::endl;
    std::cout << "y: " << real_pos_2.y << std::endl;
    std::cout << "z: " << real_pos_2.z << std::endl;
    std::cout << "rx: " << real_pos_2.rx << std::endl;
    std::cout << "ry: " << real_pos_2.ry << std::endl;
    std::cout << "rz: " << real_pos_2.rz << std::endl;


//    al.RecordCurRealPC("c:", "123");

    // T_1: matrix input
    Eigen::Matrix4f T_1;
    Eigen::Matrix4f T_2;
    Eigen::Matrix3f R_1;


    T_1.setZero();
    T_2.setZero();
    R_1.setZero();

    float x,y,z;
    std::vector<float> rpy;

    T_1 <<
    -0.994480442272671, 0.104750838243843, 0.00599264752553959, 0.1193786,
    0.02254707923126, 0.157577670571579, 0.987249161536931, 0.1422883,
    0.102470869788893, 0.981935099497134, -0.159069736971746, 0.9545875,
    0, 0, 0, 1;


    std::filesystem::create_directory("c:/folder2/");

    std::ofstream myfile ("c:/folder2/tf1.txt");
    if (myfile.is_open())
    {
        for (int i = 0 ; i <= 3 ; i++)
        {
            for (int j = 0 ; j <= 3 ; j++)
            {
                myfile << T_1(i,j) ;

                if(i!=3 || j!=3)
                {
                    myfile << ", ";
                }
            }
            myfile << std::endl;
        }
        myfile.close();
    }
    else
    {
        std::cout << "Unable to open file";
        return false;
    }



    T_2 <<
    -0.862408753516547, -0.0436989246263717, -0.504322858736876, 0.02973238,
    -0.50200642512407, 0.201991620943158, 0.840945262310757, 0.2537608,
    0.065120588082327, 0.978411870867915, -0.196136482971827, 0.9530663,
    0, 0, 0, 1;

    T_1 = T_1 - T_2;

    std::cout << "T_1" << std::endl << T_1 << std::endl;

    R_1 = T_1.block(0,0 ,3,3);

    //std::cout << "R_1" << std::endl << R_1 << std::endl;

    rpy = al.R2rpy(R_1);

    x =  T_1(0, 3) * 1000;
    y =  T_1(1, 3) * 1000;
    z =  T_1(2, 3) * 1000;

    std::cout << "x: " << x << std::endl;
    std::cout << "y: " << y << std::endl;
    std::cout << "z: " << z << std::endl;
    std::cout << "rx: " << rpy[0] << std::endl;
    std::cout << "ry: " << rpy[1] << std::endl;
    std::cout << "rz: " << rpy[2] << std::endl;

    yf::data::arm::Point3d p1;
    yf::data::arm::Point3d p2;
    yf::data::arm::Point3d p3;

    p1.x = 119.3786;  p1.y = 142.2883; p1.z = 954.5875; p1.rx = 99.20176; p1.ry = -5.881472; p1.rz = 178.7012;
    p2.x = 29.73238;    p2.y = 253.7608 ; p2.z = 953.0663; p2.rx = 101.3355; p2.ry = -3.733777; p2.rz = -149.7964;

    p3.x  = p2.x  - p1.x ;
    p3.y  = p2.y  - p1.y ;
    p3.z  = p2.z  - p1.z ;
    p3.rx = p2.rx - p1.rx;
    p3.ry = p2.ry - p1.ry;
    p3.rz = p2.rz - p1.rz;

    auto T_delta = al.points2TMat(p3);

    std::cout << "T_delta: " << std::endl << T_delta <<std::endl;

#if 0
    yf::algorithm::cleaning_motion al_motion;
    yf::algorithm::arm_path al_path;

    yf::data::arm::Point3d p1;
    yf::data::arm::Point3d p2;
    yf::data::arm::Point3d p3;

//    p1.x = -292.85;  p1.y = 832.61; p1.z = -280.7; p1.rx = 63.11; p1.ry = -6.73; p1.rz = -178.71;
    p2.x = -342.19;    p2.y = 937.58; p2.z = 199.02; p2.rx = -138.35; p2.ry = 1.93; p2.rz = 3.74;
//    p3.x = -179;   p3.y = 835.21; p3.z = -294.12; p3.rx = 63.1; p3.ry = -6.72; p3.rz = -178.71;

    p1.x = 524.307;  p1.y = 176.7380; p1.z = 531.3431;
    p1.rx = 179.59; p1.ry = 1.577; p1.rz = -67.24;
    p3.x = 976.2513;   p3.y = -368.1517; p3.z = -61.53578;
    p3.rx = -73.52872; p3.ry = -0.9297386; p3.rz = -129.0411;

    auto T_up = al_path.points2TMat(p1);

//    auto T_down = al_path.points2TMat(p2);

    auto T_up2 = al_path.points2TMat(p3);

    std::cout << "T_up: " << std::endl << T_up <<std::endl;
//    std::cout << "T2" << T_down <<std::endl;
    std::cout << "T_up2: " << std::endl <<  T_up2 <<std::endl;


    std::deque<yf::data::arm::Point3d> points;
    std::deque<yf::data::arm::Point3d> via_points;
    std::deque<yf::data::arm::Point3d> via_points_a;
    std::deque<yf::data::arm::Point3d> via_points_b;

    via_points.clear();
    via_points_a.clear();
    via_points_b.clear();

    points.push_back(p1);
    points.push_back(p2);
    points.push_back(p3);

    al_motion.CenterFit3d(points);

    auto center_point_ = al_motion.get_center();
    auto radius_ = al_motion.get_radius();
    auto v1n_ = al_motion.get_v1n();
    auto v2nb_ = al_motion.get_v2nb();

    // via_points, with center_point_ and radius_
    for(int i = 1; i <= 361; i = i+10)
    {
        yf::data::arm::Point3d via_point;

        float a = i/180.0*PI;
        via_point.x = center_point_.x + sin(a) * radius_ * v1n_(0) + cos(a) * radius_ * v2nb_(0);
        via_point.y = center_point_.y + sin(a) * radius_ * v1n_(1) + cos(a) * radius_ * v2nb_(1);
        via_point.z = center_point_.z + sin(a) * radius_ * v1n_(2) + cos(a) * radius_ * v2nb_(2);
        via_point.rx = center_point_.rx;
        via_point.ry = center_point_.ry;
        via_point.rz = center_point_.rz;

        via_points.push_back(via_point);
    }

    int index = 0;

    float delta_distance =  (points[0].x-via_points[0].x) * (points[0].x-via_points[0].x) +
                            (points[0].y-via_points[0].y) * (points[0].y-via_points[0].y) +
                            (points[0].z-via_points[0].z) * (points[0].z-via_points[0].z);

    for (int n = 0; n < via_points.size(); n++)
    {
        float delta_distance_temp = (points[0].x-via_points[n].x) * (points[0].x-via_points[n].x) +
                                    (points[0].y-via_points[n].y) * (points[0].y-via_points[n].y) +
                                    (points[0].z-via_points[n].z) * (points[0].z-via_points[n].z);
        if(delta_distance > delta_distance_temp)
        {
            delta_distance = delta_distance_temp;

            index = n;
        }
    }

    if(index != 0)
    {
        via_points_a.insert(via_points_a.begin(),via_points.begin()+index,via_points.end());

        via_points_b.insert(via_points_b.begin(), via_points.begin(),via_points.begin()+index-1);

        via_points.clear();

        via_points.insert(via_points.end(),via_points_a.begin(),via_points_a.end());
        via_points.insert(via_points.end(),via_points_b.begin(),via_points_b.end());
    }


#endif
}