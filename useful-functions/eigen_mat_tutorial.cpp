// arm
// eigen
// transformation matrix

#include <iostream>
#include <string>
#include <sstream>

#include <Eigen/Core>
#include <Eigen/Geometry>

//void points2TMat(float* point)
//{
//  float arrMembers[16] = {1,2,3,4,5};
//
//  Eigen::MatrixXf TMat =
//      Eigen::Map<Eigen::Matrix<float, 4, 4>>(arrMembers);
//
//  Eigen::RowVectorXf vpoint = Eigen::Map < Eigen::Matrix <float , 1 , 6 > > ( point );
//
//  std::cout << "vpoint is: " << vpoint << std::endl;
//
//
//  std::cout << "TMat is: " << TMat << std::endl;
//  std::cout << "TMat's [0,0] is: " << TMat(0,0) << std::endl;
//
//}

const float PI = std::atan(1.0)*4;

float& d2r(float degree)
{
  float radian = PI/180* degree;
  return radian;
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

Eigen::Matrix4f points2TMat(float* point)
{

  float arrMembers[16] = {};

  Eigen::MatrixXf TMat =
      Eigen::Map<Eigen::Matrix<float, 4, 4>>(arrMembers);

  Eigen::RowVectorXf vpoint = Eigen::Map < Eigen::Matrix <float , 1 , 6 > > ( point );;

  TMat(0,3) = vpoint(0);
  TMat(1,3) = vpoint(1);
  TMat(2,3) = vpoint(2);
  TMat(3,3) = 1;

  auto RMat = ryp2RMat(vpoint(3),vpoint(4),vpoint(5));

  TMat(0,0) = RMat(0,0);
  TMat(0,1) = RMat(0,1);
  TMat(0,2) = RMat(0,2);
  TMat(1,0) = RMat(1,0);
  TMat(1,1) = RMat(1,1);
  TMat(1,2) = RMat(1,2);
  TMat(2,0) = RMat(2,0);
  TMat(2,1) = RMat(2,1);
  TMat(2,2) = RMat(2,2);


  return TMat;

}

struct rs_point
{
    float x;
    float y;
    float z; // depth
};


int main() {

    // ref_pc_whole: reference point clouds.

    // ref_arm_pos1, ref_arm_pos2, ref_arm_pos3, ... ref_arm_posN

    // 1 arm_config_id --- n ref_pos_ID

    // 1 ref_pos_id --- 1 point_cloud

#if 0
    std::vector<rs_point> pc_part1;

    rs_point point1;

    point1.x = 0;
    point1.y = 1;
    point1.z = 2;

    pc_part1.push_back(point1);

#endif

    // --->input:  arm_config_id. & post rs_ref_arm_pos



    float arr_ref_tag[6] = { -365.90, 576.41, 330.63, 146.09, -3.33, -177.89 };

    Eigen::Matrix4f a = points2TMat(arr_ref_tag);

    std::cout << "T is: " << std::endl <<  a << std::endl;



#if 0

    Eigen::Matrix3f Ra;

    Ra << 1,   0,    0,
          0,   1,    3,
          0,   1,    2;

    std::cout << Ra << std::endl;

    float arrVertices[16] = {};

    Eigen::MatrixXf mVertices = Eigen::Map < Eigen::Matrix <float , 4 , 4 > > ( arrVertices );

    std::cout << "mVertices" << std::endl;
    std::cout << mVertices << std::endl;

    Eigen::RowVectorXf ref_tag_position;

    float arr_ref_tag[6] = { -443.66, -66.02, -15.34, 179.67, 0.33,90.87};

    Eigen::Matrix4f a = points2TMat(arr_ref_tag);

    std::vector<std::vector<float>> original_path2 = {{-358.8814,-73.76439,358.9901,-173.859,0.1401029,-87.89309},{-523.3323,-132.1928,363.0237,-174.6929,-1.481504,-92.38377}};

    std::cout << "original_path2 size is: " << original_path2[0][0] << std::endl;

    Eigen::Matrix4f T_real;

    T_real << 1,2,3,4,
              5,6,7,8,
              9,10,11,12,
              13,14,15,16;

    std::cout << "T_real is: " << T_real << std::endl;

    Eigen::MatrixXf R_real = T_real.block(0,3 ,3,1);


    std::cout << "R_real is: "  << R_real(1) << std::endl;

#endif

    return 1;
}
