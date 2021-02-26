//
// Created by yf on 2021/1/21.
//
// TODO:
//  1. read .log file, obtain original path.
//  2. input "ref_landmark_position" and "real_landmark_position", convert it to real path, stored in vector.
//  3. convert it to listen node command.

#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "w0303_ln_msg.h"

const float PI = std::atan(1.0)*4;

float d2r(float degree)
{
    float radian = PI/180* degree;
    return radian;
}

float r2d(float radian)
{
  return radian*180/PI;
}

Eigen::Matrix3f ryp2RMat(const float& roll, const float& pitch, const float& yaw)
{
    // roll  -- x axis --- gamma
    // yaw   -- y axis --- beta
    // pitch -- z axis --- alpha

    float gamma = d2r(roll);
    float beta  = d2r(pitch);
    float alpha = d2r(yaw);

    auto ca = cos(alpha);
    auto cb = cos(beta);
    auto cg = cos(gamma);
    auto sa = sin(alpha);
    auto sb = sin(beta);
    auto sg = sin(gamma);

    // Rx -- Rg
    Eigen::Matrix3f Rg;

    Rg << 1,   0,    0,
          0,  cg,  -sg,
          0,  sg,   cg;

    // Ry -- Rb
    Eigen::Matrix3f Rb;

    Rb << cb,  0,  sb,
           0,  1,   0,
         -sb,  0,  cb;

    // Rz -- Ra
    Eigen::Matrix3f Ra;

    Ra << ca,  -sa,  0,
          sa,   ca,  0,
           0,    0,  1;

    return Ra*Rb*Rg;

}

Eigen::Matrix4f points2TMat(std::vector<float>& point)
{

    float arrMembers[16] = {};

    Eigen::MatrixXf TMat =
            Eigen::Map<Eigen::Matrix<float, 4, 4>>(arrMembers);

    Eigen::RowVectorXf vpoint = Eigen::Map<Eigen::Matrix<float, 1, 6> >(point.data());;

    TMat(0, 3) = vpoint(0);
    TMat(1, 3) = vpoint(1);
    TMat(2, 3) = vpoint(2);
    TMat(3, 3) = 1;

    auto RMat = ryp2RMat(vpoint(3), vpoint(4), vpoint(5));

    TMat(0, 0) = RMat(0, 0);
    TMat(0, 1) = RMat(0, 1);
    TMat(0, 2) = RMat(0, 2);
    TMat(1, 0) = RMat(1, 0);
    TMat(1, 1) = RMat(1, 1);
    TMat(1, 2) = RMat(1, 2);
    TMat(2, 0) = RMat(2, 0);
    TMat(2, 1) = RMat(2, 1);
    TMat(2, 2) = RMat(2, 2);

    return TMat;
}

std::vector<float> R2rpy(Eigen::Matrix3f& RMat)
{
    std::vector<float> rpy;
    rpy.clear();

    float beta(0);
    float alpha(0);
    float gamma(0);

    auto r11 = RMat(0, 0);
    auto r12 = RMat(0, 1);
    auto r13 = RMat(0, 2);
    auto r21 = RMat(1, 0);
    auto r22 = RMat(1, 1);
    auto r23 = RMat(1, 2);
    auto r31 = RMat(2, 0);
    auto r32 = RMat(2, 1);
    auto r33 = RMat(2, 2);

    beta = atan2(-r31, sqrt(r11 * r11 + r21 * r21));

    if (beta > d2r(89.99)) {
        beta = d2r(89.99);
        alpha = 0;
        gamma = atan2(r12, r22);
    } else if (beta < -d2r(89.99)) {
        beta = -d2r(89.99);
        alpha = 0;
        gamma = -atan2(r12, r22);
    } else {
        auto cb = cos(beta);
        alpha = atan2(r21 / cb, r11 / cb);
        gamma = atan2(r32 / cb, r33 / cb);
    }

    rpy.push_back(r2d(gamma));
    rpy.push_back(r2d(beta));
    rpy.push_back(r2d(alpha));

    return rpy;
}

// Transformation Matrix
// todo: Transformation Matrix
std::vector<std::vector<float>> Convert2RealPathByLM(std::vector<std::vector<float>>& original_path,
                                                     std::vector<float>& ref_tag_position,
                                                     std::vector<float>& real_tag_positon)
{
    std::vector<std::vector<float>> real_path;

    real_path.clear();

    Eigen::Matrix4f T_ref_tag;      //T1
    Eigen::Matrix4f T_real_tag;     //T4

    T_ref_tag.setZero();
    T_real_tag.setZero();

    T_ref_tag = points2TMat(ref_tag_position);    // T1
    T_real_tag = points2TMat(real_tag_positon);    // T4

    for(int i = 0; i < original_path.size(); i++)
    {
      std::vector<float> real_point;
      std::vector<float> real_point_rpy;
      real_point.clear();

      auto T_n = points2TMat(original_path[i]);

      Eigen::Matrix4f T_real = T_real_tag * T_ref_tag.inverse() * T_n;
      Eigen::MatrixXf Translation_real = T_real.block(0,3 ,3,1);

      Eigen::Matrix3f R_real = T_real.block(0,0 ,3,3);

      real_point_rpy = R2rpy(R_real);

      // x,y,z
      real_point.push_back(Translation_real(0));
      real_point.push_back(Translation_real(1));
      real_point.push_back(Translation_real(2));
      // rx,ry,rz
      real_point.push_back(real_point_rpy[0]);
      real_point.push_back(real_point_rpy[1]);
      real_point.push_back(real_point_rpy[2]);

      real_path.push_back(real_point);
    }

    return real_path;
}

// ----------------------------------------------------------------------------------------------------------- //

// Listen Node:
// todo: checksum function. compute the value between '$' and ','
std::string ChecksumStr(const std::string& str)
{
    uint16_t checksum = 0;

    unsigned int index = 1; // Skip $ character at beginning
    const unsigned int length = str.length();
    for (; (index < length); ++index)
    {
        const char16_t c = str[index];
        if (c == '*')
            break;
        checksum = checksum ^ c;
    }

    std::stringstream stream;
    stream << std::hex <<checksum;

    std::string result( stream.str() );

    return result;
}

// split string "-523.332275,-132.192795,363.023712,-174.692902,-1.481504,-92.383774" by delimiter ","
std::vector<float> SplitStrPoint (const std::string& s, char delim)
{
    std::vector<float> result;
    std::stringstream ss (s);
    std::string item;

    while (getline (ss, item, delim)) {
        result.push_back (std::stof(item));
    }

    return result;
}

// read log file line by line.
// find delimiter "{" and "}"
// convert to original path
std::vector<std::vector<float>> ReadOriginalPath(const std::string& text)
{
    std::vector<std::vector<float>> original_path;

    std::fstream file;
    std::string path1 ="C:\\Dev\\w0303\\Arm_Control_Module_v1.0\\useful-functions\\" ;
    std::string path = path1 + text;
    file.open(path);

    if (file)
    {
        std::cout<< "i've managed to open the file!"<< "\n";
    }

    std::string line;

    while (getline(file, line))
    {
        auto char_index1 = line.find("{");
        auto char_index2 = line.find("}");

        if(char_index1 != std::string::npos)
        {
//            std::cout << "char_index1 is: " << char_index1 << std::endl;
//            std::cout << "index1: " << char_index1 << "index2: " << char_index2 << std::endl;

            auto str_point = line.substr(char_index1+1,char_index2-char_index1-1);
//            std::cout << "sub string is: " << str_point << std::endl;

            char delim = ',';
            auto v_point = SplitStrPoint(str_point, delim);

            original_path.push_back(v_point);
        }
//        std::cout << line << "\n";
    }

    return original_path;
}

#if 0
std::vector<std::string> LinesForTargetPoints(const std::vector<std::vector<float>>& original_path , const LnMotionMessage& msg)
{


    std::vector<std::string> lines_for_point;
    // construct lines for point.
    for (int i = 0; i < original_path.size(); i++)
    {
         std::string line_for_point = "float[] targetP" + std::to_string(i) + " = {" +
                 std::to_string(original_path[i][0]) + "," +
                 std::to_string(original_path[i][1]) + "," +
                 std::to_string(original_path[i][2]) + "," +
                 std::to_string(original_path[i][3]) + "," +
                 std::to_string(original_path[i][4]) + "," +
                 std::to_string(original_path[i][5]) + "}\r\n";

         lines_for_point.push_back(line_for_point);
    }

    return lines_for_point;
}

std::vector<std::string> LinesForTargetPointsMotion(const std::vector<std::vector<float>>& original_path , const LnMotionMessage& msg)
{
    std::vector<std::string> lines_for_motion;

    // construct lines for motion
    for (int i = 0; i < original_path.size(); i++)
    {
        if(i!= original_path.size()-1)
        {
            std::string line_for_motion = msg.motion_type + "(\"" +
                msg.motion_structure + "\",targetP" + std::to_string(i) + "," +
                std::to_string(msg.speed) + "," +
                std::to_string(msg.time_to_top_speed) + "," +
                std::to_string(msg.blending_percentage) + ")\r\n";

            lines_for_motion.push_back(line_for_motion);
//            std::cout << "line1 for motion: " << line_for_motion << std::endl;
        }
        else
        {
            std::string line_for_motion = msg.motion_type + "(\"" +
                                          msg.motion_structure + "\",targetP" + std::to_string(i) + "," +
                                          std::to_string(msg.speed) + "," +
                                          std::to_string(msg.time_to_top_speed) + "," +
                                          std::to_string(msg.blending_percentage) + ")";

            lines_for_motion.push_back(line_for_motion);
//            std::cout << "line2 for motion: " << line_for_motion << std::endl;
        }
    }

    return lines_for_motion;
}
#endif

std::vector<std::string> LinesForDirectMotion(const std::vector<std::vector<float>>& original_path , const LnMotionMessage& msg)
{
    std::vector<std::string> lines_for_motion;

    // construct lines for motion
    for (int i = 0; i < original_path.size(); i++)
    {
        std::string line_for_point = std::to_string(original_path[i][0]) + "," +
                                     std::to_string(original_path[i][1]) + "," +
                                     std::to_string(original_path[i][2]) + "," +
                                     std::to_string(original_path[i][3]) + "," +
                                     std::to_string(original_path[i][4]) + "," +
                                     std::to_string(original_path[i][5]);

        if(i!= original_path.size()-1)
        {
            std::string line_for_motion = msg.motion_type + "(\"" +
                                          msg.motion_structure + "\"," +
                                          line_for_point + "," +
                                          std::to_string(msg.speed) + "," +
                                          std::to_string(msg.time_to_top_speed) + "," +
                                          std::to_string(msg.blending_percentage) + ")\r\n";

            lines_for_motion.push_back(line_for_motion);
//            std::cout << "line1 for motion: " << line_for_motion << std::endl;
        }
        else
        {
            std::string line_for_motion = msg.motion_type + "(\"" +
                                          msg.motion_structure + "\"," +
                                          line_for_point + "," +
                                          std::to_string(msg.speed) + "," +
                                          std::to_string(msg.time_to_top_speed) + "," +
                                          std::to_string(msg.blending_percentage) + ")";

            lines_for_motion.push_back(line_for_motion);
//            std::cout << "line2 for motion: " << line_for_motion << std::endl;
        }
    }

    return lines_for_motion;
}

#if 0
// script id, "pose", lines_for_points, lines_for_motion
std::string ListenNodeTargetScript(const std::vector<std::vector<float>>& original_path ,
                               const LnMotionMessage& motion_msg, LnMessage& ln_msg)
{
    // script body
    std::string aline_for_points("");
    std::string aline_for_motion("");

    std::vector<std::string> lines_for_points = LinesForTargetPoints(original_path, motion_msg);
    std::vector<std::string> lines_for_motion = LinesForTargetPointsMotion(original_path, motion_msg);

    for(int i = 0 ; i < lines_for_points.size(); i++)
    {
        aline_for_points += lines_for_points[i];
    }

    for(int i = 0 ; i < lines_for_motion.size(); i++)
    {
        aline_for_motion += lines_for_motion[i];
    }

    //construct the body script
    std::string body_lines("");
    body_lines = std::to_string(ln_msg.data.script_id)+",\r\n"+aline_for_points+aline_for_motion;

//    std::cout << "body_lines is: " << body_lines << std::endl;
//    std::cout << "body_lines size is: " << body_lines.size() << std::endl;

    // construct the final script
    ln_msg.length = body_lines.size();

    // construct the final script : for checksum value
    std::string checksum_body_str("");
    checksum_body_str = ln_msg.header+std::to_string(ln_msg.length)+","+body_lines+",*";

    ln_msg.checksum = ChecksumStr(checksum_body_str);

    std::string lines = checksum_body_str+ln_msg.checksum+"\r\n";

    return lines;

}

//
std::string LnDefineTargetPoints(const std::vector<std::vector<float>>& original_path ,
                               const LnMotionMessage& motion_msg, LnMessage& ln_msg)
{
    // script body
    std::string aline_for_points("");

    std::vector<std::string> lines_for_points = LinesForTargetPoints(original_path);
    std::vector<std::string> lines_for_motion = LinesForTargetPointsMotion(original_path, motion_msg);

    for(int i = 0 ; i < lines_for_points.size(); i++)
    {
        aline_for_points += lines_for_points[i];
    }

    //construct the body script
    std::string body_lines("");
    body_lines = std::to_string(ln_msg.data.script_id)+",\r\n"+aline_for_points;

//    std::cout << "body_lines is: " << body_lines << std::endl;
//    std::cout << "body_lines size is: " << body_lines.size() << std::endl;

    // construct the final script
    ln_msg.length = body_lines.size();

    // construct the final script : for checksum value
    std::string checksum_body_str("");
    checksum_body_str = ln_msg.header+std::to_string(ln_msg.length)+","+body_lines+",*";

    ln_msg.checksum = ChecksumStr(checksum_body_str);

    std::string lines = checksum_body_str+ln_msg.checksum+"\r\n";

    return lines;
}
//
std::string LnExecuteTargetPoints(const std::vector<std::vector<float>>& original_path ,
                            const LnMotionMessage& motion_msg, LnMessage& ln_msg)
{
    // script body

    std::string aline_for_motion("");

    std::vector<std::string> lines_for_motion = LinesForTargetPointsMotion(original_path, motion_msg);


    for(int i = 0 ; i < lines_for_motion.size(); i++)
    {
        aline_for_motion += lines_for_motion[i];
    }

    //construct the body script
    std::string body_lines("");
    body_lines = std::to_string(ln_msg.data.script_id)+",\r\n"+aline_for_points+aline_for_motion;

//    std::cout << "body_lines is: " << body_lines << std::endl;
//    std::cout << "body_lines size is: " << body_lines.size() << std::endl;

    // construct the final script
    ln_msg.length = body_lines.size();

    // construct the final script : for checksum value
    std::string checksum_body_str("");
    checksum_body_str = ln_msg.header+std::to_string(ln_msg.length)+","+body_lines+",*";

    ln_msg.checksum = ChecksumStr(checksum_body_str);

    std::string lines = checksum_body_str+ln_msg.checksum+"\r\n";

    return lines;
}
#endif

std::string ListenNodeDirectScript(const std::vector<std::vector<float>>& original_path ,
                                 const LnMotionMessage& motion_msg, LnMessage& ln_msg)
{
    std::string aline_for_motion("");

    std::vector<std::string> lines_for_motion = LinesForDirectMotion(original_path, motion_msg);

    for(int i = 0 ; i < lines_for_motion.size(); i++)
    {
        aline_for_motion += lines_for_motion[i];
    }

    //construct the body script
    std::string body_lines("");
    body_lines = std::to_string(ln_msg.data.script_id)+",\r\n"+aline_for_motion;

    // construct the final script
    ln_msg.length = body_lines.size();

    // construct the final script : for checksum value
    std::string checksum_body_str("");
    checksum_body_str = ln_msg.header+std::to_string(ln_msg.length)+","+body_lines+",*";

    ln_msg.checksum = ChecksumStr(checksum_body_str);
    std::string lines = checksum_body_str+ln_msg.checksum+"\r\n";

    return lines;
}

int main()
{
    auto str = std::string("1,\r\nfloat[] targetP1= {-90,0,90,0,90,0}\r\nPTP(\"JPP\",targetP1,10,200,0,false)");

    std::cout << "str size is: " << str.size() << std::endl;

    // todo:
    //  1. read original path from .txt file and then store it in std::vector<std::vector<float>>
    //  2. convert it to real path, stored in vector. ***
    //  3. convert to listen node command.

    //  1. read original path from .txt file.  the file should only consist of points..
    auto original_path = ReadOriginalPath("yf_20210118_landmark_ref-2021-01-22.log");
    std::vector<std::vector<float>> original_path2 = {{-358.8814,-73.76439,358.9901,-173.859,0.1401029,-87.89309},{-523.3323,-132.1928,363.0237,-174.6929,-1.481504,-92.38377}};

    //  2. input "ref_landmark_position" and "real_landmark_position", convert it to real path, stored in vector.
    std::vector<float> ref_tag_position = {-443.66,-66.02,-15.34,179.67,0.33,90.87};
    std::vector<float> real_tag_position = {-336.7,185.08,-15.37,-179.69,0.62,1.12};

    std::vector<std::vector<float>> real_path = Convert2RealPathByLM(original_path, ref_tag_position, real_tag_position);

    //  3. convert to listen node command.

    LnMotionMessage motion_msg;
    motion_msg.target_point_name   = "EMSD_6F_DESK_01_P";
    motion_msg.motion_type         = "PLine";
    motion_msg.motion_structure    = "CAP";
    motion_msg.speed               = 100;
    motion_msg.time_to_top_speed   = 250;
    motion_msg.blending_percentage = 80;

    LnMessage ln_msg;
    ln_msg.header = "$TMSCT,";
    ln_msg.data.script_id = 1;

    //define target points and then execute them. (Note: the target points can only be defined once.)
//    auto lines = ListenNodeTargetScript(real_path, motion_msg, ln_msg);

//    auto def_script = LnDefineTargetPoints(real_path, motion_msg, ln_msg);
//    auto exec_script = LnExecuteTargetPoints();

    auto lines = ListenNodeDirectScript(real_path, motion_msg, ln_msg);

    std::cout << lines << std::endl;
}
