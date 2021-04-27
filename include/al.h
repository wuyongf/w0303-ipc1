#pragma once

#include "al_common.h"
#include "data.h"

#include "sql.h"


namespace yf
{
    namespace algorithm
    {
        class cleaning_motion
        {
        public:
            cleaning_motion(){};
            virtual ~cleaning_motion(){};

        public:
            void Start(std::shared_ptr<yf::sql::sql_server> sql_ptr);

            void set_plain_edge_points(const data::arm::Point3d& p1, const data::arm::Point3d& p2,
                                       const data::arm::Point3d& p3, const data::arm::Point3d& p4);

            std::deque<yf::data::arm::Point3d> get_via_points(const yf::data::arm::MotionType &motion_type, std::deque<yf::data::arm::Point3d> init_cleaning_points);

            std::deque<yf::data::arm::Point3d> get_uvc_via_points(const yf::data::arm::MotionType &motion_type,
                                                                  std::deque<yf::data::arm::Point3d> init_cleaning_points,
                                                                  const int& layer,
                                                                  const float& step_ratio_horizontal);

            void set_layer(const int& layer_no);

        private:

            // shared database
            std::shared_ptr<yf::sql::sql_server> sql_ptr_;

            // properties

            int layer_          = 1; // at least 1
            int sample_points_  = 40;

            data::arm::Point3d edge_p1_;
            data::arm::Point3d edge_p2_;
            data::arm::Point3d edge_p3_;
            data::arm::Point3d edge_p4_;

            std::vector<data::arm::Point3d> plain_clean_path_;

        };

        class degree_related
        {
        public:
            degree_related(){}
            virtual ~degree_related(){}

            float d2r(const float& degree);
            float r2d(const float& radian);

            Eigen::Matrix3f ryp2RMat(const float& roll, const float& pitch, const float& yaw);
            Eigen::Matrix4f points2TMat(std::vector<float>& point);

            std::vector<float> R2rpy(Eigen::Matrix3f& RMat);
        };

        class arm_path
        {
        public:
            arm_path(){}
            virtual ~arm_path(){}

            void RecordRefPath();

            std::deque<yf::data::arm::Point3d> ExportRealPathByLM(  const std::deque<yf::data::arm::Point3d>& original_via_points,
                                                                    const yf::data::arm::Point3d& ref_landmark_pos,
                                                                    const yf::data::arm::Point3d& real_landmark_pos);

            yf::data::arm::Point3d ExportRealPointByLM(  const yf::data::arm::Point3d& original_via_points,
                                                         const yf::data::arm::Point3d& ref_landmark_pos,
                                                         const yf::data::arm::Point3d& real_landmark_pos);


        private:

            float d2r(const float& degree);
            float r2d(const float& radian);

            Eigen::Matrix3f ryp2RMat(const float& roll, const float& pitch, const float& yaw);

            Eigen::Matrix4f points2TMat(std::vector<float>& point);
            Eigen::Matrix4f points2TMat(const yf::data::arm::Point3d& point);

            std::vector<float> R2rpy(Eigen::Matrix3f& RMat);

            std::vector<std::vector<float>> Convert2RealPathByLM(std::vector<std::vector<float>>& original_path,
                                                                 std::vector<float>& ref_tag_position,
                                                                 std::vector<float>& real_tag_positon);
        };
    }
}

void yf::algorithm::arm_path::RecordRefPath()
{
    //todo:
    // 1. read ref_path from .txt file.
    // 2. record the LM1 position.
    // 3. record the transformation T2. T3("ref_path" to base) = T1(LM1 to base)* T2;
    return;
}

std::deque<yf::data::arm::Point3d>
        yf::algorithm::arm_path::ExportRealPathByLM(const std::deque<yf::data::arm::Point3d>& original_via_points,
                                                    const yf::data::arm::Point3d& ref_landmark_pos,
                                                    const yf::data::arm::Point3d& real_landmark_pos)
{
    //todo: Find real_path T6 = T4*inv(T1)*T3
    // T1: ref_landmark_pos
    // T3: ref_path
    // T4: real_landmark_pos

    //@@ input:
    // 1. via_points (original_path)
    // 2. ref_landmark_pos(ref_TF)
    // 3. real_landmark_pos(real_TF)
    //@@ output:
    // 1. new_via_points (real_path)

    std::deque<yf::data::arm::Point3d> real_path;

    Eigen::Matrix4f T_ref_tag;      //T1
    Eigen::Matrix4f T_real_tag;     //T4

    T_ref_tag.setZero();
    T_real_tag.setZero();

    T_ref_tag = points2TMat(ref_landmark_pos);      // T1
    T_real_tag = points2TMat(real_landmark_pos);    // T4

    for(int i = 0; i < original_via_points.size(); i++)
    {
        yf::data::arm::Point3d real_point;
        std::vector<float> real_point_rpy;


        auto T_n = points2TMat(original_via_points[i]);

        Eigen::Matrix4f T_real = T_real_tag * T_ref_tag.inverse() * T_n;
        Eigen::MatrixXf Translation_real = T_real.block(0,3 ,3,1);

        Eigen::Matrix3f R_real = T_real.block(0,0 ,3,3);

        real_point_rpy = R2rpy(R_real);

        // x,y,z
        real_point.x = Translation_real(0);
        real_point.y = Translation_real(1);
        real_point.z = Translation_real(2);
        // rx,ry,rz
        real_point.rx = real_point_rpy[0];
        real_point.ry = real_point_rpy[1];
        real_point.rz = real_point_rpy[2];

        real_path.push_back(real_point);
    }

    return real_path;
}

float yf::algorithm::arm_path::d2r(const float &degree)
{
    float radian = PI/180* degree;
    return radian;
}

float yf::algorithm::arm_path::r2d(const float &radian)
{
    return radian*180/PI;
}

Eigen::Matrix3f yf::algorithm::arm_path::ryp2RMat(const float &roll, const float &pitch, const float &yaw)
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

Eigen::Matrix4f yf::algorithm::arm_path::points2TMat(std::vector<float> &point)
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

std::vector<float> yf::algorithm::arm_path::R2rpy(Eigen::Matrix3f &RMat)
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

std::vector<std::vector<float>>
yf::algorithm::arm_path::Convert2RealPathByLM(std::vector<std::vector<float>> &original_path,
                                              std::vector<float> &ref_tag_position,
                                              std::vector<float> &real_tag_positon)
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

Eigen::Matrix4f yf::algorithm::arm_path::points2TMat(const yf::data::arm::Point3d &point)
{
    std::vector<float> vec_point;

    vec_point.clear();

    vec_point.push_back(point.x);
    vec_point.push_back(point.y);
    vec_point.push_back(point.z);
    vec_point.push_back(point.rx);
    vec_point.push_back(point.ry);
    vec_point.push_back(point.rz);

    float arrMembers[16] = {};

    Eigen::MatrixXf TMat =
            Eigen::Map<Eigen::Matrix<float, 4, 4>>(arrMembers);

    Eigen::RowVectorXf vpoint = Eigen::Map<Eigen::Matrix<float, 1, 6> >(vec_point.data());;

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

yf::data::arm::Point3d yf::algorithm::arm_path::ExportRealPointByLM(const yf::data::arm::Point3d &original_via_points,
                                                                    const yf::data::arm::Point3d &ref_landmark_pos,
                                                                    const yf::data::arm::Point3d &real_landmark_pos)
{
    //todo: Find real_path T6 = T4*inv(T1)*T3
    // T1: ref_landmark_pos
    // T3: ref_path
    // T4: real_landmark_pos

    //@@ input:
    // 1. via_points (original_path)
    // 2. ref_landmark_pos(ref_TF)
    // 3. real_landmark_pos(real_TF)
    //@@ output:
    // 1. new_via_points (real_path)

    yf::data::arm::Point3d real_point;

    Eigen::Matrix4f T_ref_tag;      //T1
    Eigen::Matrix4f T_real_tag;     //T4

    T_ref_tag.setZero();
    T_real_tag.setZero();

    T_ref_tag = points2TMat(ref_landmark_pos);      // T1
    T_real_tag = points2TMat(real_landmark_pos);    // T4

    std::vector<float> real_point_rpy;

    auto T_n = points2TMat(original_via_points);

    Eigen::Matrix4f T_real = T_real_tag * T_ref_tag.inverse() * T_n;
    Eigen::MatrixXf Translation_real = T_real.block(0,3 ,3,1);

    Eigen::Matrix3f R_real = T_real.block(0,0 ,3,3);

    real_point_rpy = R2rpy(R_real);

    // x,y,z
    real_point.x = Translation_real(0);
    real_point.y = Translation_real(1);
    real_point.z = Translation_real(2);
    // rx,ry,rz
    real_point.rx = real_point_rpy[0];
    real_point.ry = real_point_rpy[1];
    real_point.rz = real_point_rpy[2];

    return real_point;
}

void yf::algorithm::cleaning_motion::Start(std::shared_ptr<yf::sql::sql_server> sql_ptr)
{
    sql_ptr_ = sql_ptr;
}

void yf::algorithm::cleaning_motion::set_plain_edge_points(const yf::data::arm::Point3d &p1,
                                                           const yf::data::arm::Point3d &p2,
                                                           const yf::data::arm::Point3d &p3,
                                                           const yf::data::arm::Point3d &p4)
{

}

std::deque<yf::data::arm::Point3d> yf::algorithm::cleaning_motion::get_via_points(const yf::data::arm::MotionType &motion_type,
                                                                                  std::deque<yf::data::arm::Point3d> init_cleaning_points)
{
    std::deque<yf::data::arm::Point3d> via_points;

    switch (motion_type)
    {
        case yf::data::arm::MotionType::PlaneMotion: // plain_cleaning
        {
            /// 1. Initialization
            edge_p1_ = init_cleaning_points[0];
            edge_p2_ = init_cleaning_points[1];
            edge_p3_ = init_cleaning_points[2];
            edge_p4_ = init_cleaning_points[3];


            int motion_line = layer_ + 1;
            int occupied_points = motion_line * 2;
            int free_points = sample_points_ - occupied_points;
            int points_no_each_line = std::floor(free_points/motion_line);
            int totoal_points = occupied_points + motion_line * points_no_each_line;

            float delta_x = abs(edge_p2_.x - edge_p1_.x);
            float delta_z = abs(edge_p3_.z - edge_p2_.z);

            float delta_x_each_point = delta_x / ( points_no_each_line + 1.0);
            float delta_z_each_line = delta_z / layer_;

            float step_ratio_horizontal = abs( delta_x_each_point / delta_x );
            float step_ratio_vertical = 1.0 / layer_;

            bool even_flag = true;
            int line_counter = 0;

            int point_no = 0;

            for (float u = 0.0; u <= 1.0; u = u + step_ratio_vertical)
            {
                // count no. func
                // if the number is even, we should count the point from left to right
                // for 1st line, line_counter == 0
                //
                if( line_counter % 2 == 0 )
                {
                    even_flag = true;
                }
                else
                {
                    even_flag = false;
                }

                // current line point no.
                //
                int cur_line_point_no = 0;

                // temp deque for each line.
                //
                std::deque<yf::data::arm::Point3d> temp_points;
                temp_points.clear();

                //
                step_ratio_horizontal = 1.0;

                for(float v = 0.0 ; v <= 1.0; v = v + step_ratio_horizontal)
                {
                    yf::data::arm::Point3d via_point;

                    // count point_no.
                    point_no ++;

                    // current line point no.
                    cur_line_point_no ++;

                    via_point.x = (1-u)*(1-v)*edge_p1_.x + v*(1-u)*edge_p2_.x + u*v*edge_p3_.x + u*(1-v)*edge_p4_.x;
                    via_point.y = (1-u)*(1-v)*edge_p1_.y + v*(1-u)*edge_p2_.y + u*v*edge_p3_.y + u*(1-v)*edge_p4_.y;
                    via_point.z = (1-u)*(1-v)*edge_p1_.z + v*(1-u)*edge_p2_.z + u*v*edge_p3_.z + u*(1-v)*edge_p4_.z;
                    via_point.rx = (1-u)*(1-v)*edge_p1_.rx + v*(1-u)*edge_p2_.rx + u*v*edge_p3_.rx + u*(1-v)*edge_p4_.rx;
                    via_point.ry = (1-u)*(1-v)*edge_p1_.ry + v*(1-u)*edge_p2_.ry + u*v*edge_p3_.ry + u*(1-v)*edge_p4_.ry;
                    via_point.rz = (1-u)*(1-v)*edge_p1_.rz + v*(1-u)*edge_p2_.rz + u*v*edge_p3_.rz + u*(1-v)*edge_p4_.rz;

                    temp_points.push_back(via_point);
                }

                if(even_flag == true)
                {
                    for(int n = 0; n < temp_points.size(); n++)
                    {
                        via_points.push_back(temp_points[n]);
                    }
                }
                else
                {
                    std::reverse(temp_points.begin(),temp_points.end());

                    for(int n = 0; n < temp_points.size(); n++)
                    {
                        via_points.push_back(temp_points[n]);
                    }
                }

                line_counter ++;
            }

            break;
        }

        case yf::data::arm::MotionType::LineMotion: // line_cleaning
        {
            /// 1. Initialization
            edge_p1_ = init_cleaning_points[0];
            edge_p2_ = init_cleaning_points[1];

            via_points.push_back(edge_p1_);
            via_points.push_back(edge_p2_);

            break;
        }
    }

    auto via_points_reverse = via_points;

    std::reverse(via_points_reverse.begin(),via_points_reverse.end());

    for(int n = 0 ; n < via_points_reverse.size() ; n++)
    {
        via_points.push_back(via_points_reverse[n]);
    }

    return via_points;
}

std::deque<yf::data::arm::Point3d>
yf::algorithm::cleaning_motion::get_uvc_via_points(const yf::data::arm::MotionType &motion_type,
                                                   std::deque<yf::data::arm::Point3d> init_cleaning_points,
                                                   const int& layer,
                                                   const float& step_ratio_horizontal)
{
    std::deque<yf::data::arm::Point3d> via_points;

    switch (motion_type)
    {
        case yf::data::arm::MotionType::PlaneMotion: // plain_cleaning
        {
            LOG(INFO) << "wrong motion type!";
            break;
        }

        case yf::data::arm::MotionType::LineMotion: // line_cleaning
        {
            /// 1. Initialization
            edge_p1_ = init_cleaning_points[0];
            edge_p2_ = init_cleaning_points[1];

            std::deque<yf::data::arm::Point3d> temp_points;

            for(float v = 0.0 ; v <= 1.0; v = v + step_ratio_horizontal)
            {
                yf::data::arm::Point3d via_point;

                via_point.x = (1-v)*edge_p1_.x + v*edge_p2_.x;
                via_point.y = (1-v)*edge_p1_.y + v*edge_p2_.y;
                via_point.z = (1-v)*edge_p1_.z + v*edge_p2_.z;
                via_point.rx = (1-v)*edge_p1_.rx + v*edge_p2_.rx;
                via_point.ry = (1-v)*edge_p1_.ry + v*edge_p2_.ry;
                via_point.rz = (1-v)*edge_p1_.rz + v*edge_p2_.rz;

                temp_points.push_back(via_point);
            }

            via_points = temp_points;

            break;
        }
    }

#if 0

    auto via_points_reverse = via_points;

    std::reverse(via_points_reverse.begin(),via_points_reverse.end());

    for(int n = 0 ; n < via_points_reverse.size() ; n++)
    {
        via_points.push_back(via_points_reverse[n]);
    }

#endif

    return via_points;
}

void yf::algorithm::cleaning_motion::set_layer(const int &layer_no)
{
    layer_ = layer_no;
    return;
}
