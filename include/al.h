#pragma once

#include <iostream>
#include <Windows.h>
#include <stdio.h>

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

            std::deque<yf::data::arm::Point3d> get_mop_via_points(  const yf::data::arm::ModelType& model_type,
                                                                    const yf::data::arm::MotionType &motion_type,
                                                                    const std::deque<yf::data::arm::Point3d>& ref_path_init_points,
                                                                    const int& layer,
                                                                    float& step_ratio_horizontal);

            std::deque<yf::data::arm::Point3d> get_uvc_via_points(const yf::data::arm::MotionType& motion_type,
                                                                  const std::deque<yf::data::arm::Point3d>& ref_path_init_points,
                                                                  const int& layer,
                                                                  const float& step_ratio_horizontal);

        private:

            std::deque<yf::data::arm::Point3d> GetCircleEachLayerViaPoints(const float& radius, const float& step_ratio_horizontal, const yf::data::arm::Point3d& init_point);

            double GetCurveOffsetDistance(const int& layer_no,
                                             const std::deque<yf::data::arm::Point3d>& ref_init_points);

        public:

            yf::data::arm::Point3d get_center();
            float get_radius();
            Eigen::RowVectorXf get_v1n();
            Eigen::RowVector3f get_v2nb();

        private:

            // shared database
            std::shared_ptr<yf::sql::sql_server> sql_ptr_;

            // properties
            //
            data::arm::Point3d init_p1_;
            data::arm::Point3d init_p2_;
            data::arm::Point3d init_p3_;
            data::arm::Point3d init_p4_;

            /// for motion type: Plane
            int sample_points_  = 60;

            /// for motion type: Circle
            yf::data::arm::Point3d center_point_;
            float radius_;

            Eigen::RowVectorXf v1n_;
            Eigen::RowVector3f v2nb_;

            float offset_distance_;

        public:

            void CenterFit3d(const std::deque<yf::data::arm::Point3d>& init_cleaning_points);

            std::vector<float> Point3d_2_Vector(const yf::data::arm::Point3d& point);

            void set_offset_distance(const float& distance);
        };

        class degree_related
        {
        public:
            degree_related(){}
            virtual ~degree_related(){}

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


        public:

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

        class TimeSleep
        {
        public:
            TimeSleep(){};
            virtual ~TimeSleep(){};
        public:
            void ms(const int& ms);
            void sec(const int& sec);
            void minute(const int& min);
        };

        struct Timer
                {
            std::chrono::time_point<std::chrono::steady_clock> start,end;
            std::chrono::duration<float> duration;

            Timer()
            {
                start = std::chrono::high_resolution_clock::now();
            }

            ~Timer()
            {
                end = std::chrono::high_resolution_clock::now();
                duration = end - start;

                float ms = duration.count() * 1000.0f;
                std::cout << "Timer took " << ms << "ms" << std::endl;
            }
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

std::deque<yf::data::arm::Point3d>
yf::algorithm::cleaning_motion::get_mop_via_points( const yf::data::arm::ModelType& model_type,
                                                    const yf::data::arm::MotionType& motion_type,
                                                    const std::deque<yf::data::arm::Point3d>& ref_path_init_points,
                                                    const int& layer,
                                                    float& step_ratio_horizontal)
{
    // Initialization
    std::deque<yf::data::arm::Point3d> via_points;
    via_points.clear();

    switch (motion_type)
    {
        case yf::data::arm::MotionType::Plane:
        {
            //@@ input: 4 points

            /// 1. Initialization
            init_p1_ = ref_path_init_points[0];
            init_p2_ = ref_path_init_points[1];
            init_p3_ = ref_path_init_points[2];
            init_p4_ = ref_path_init_points[3];

            int motion_line = layer + 1;
            int occupied_points = motion_line * 2;
            int free_points = sample_points_ - occupied_points;
            int points_no_each_line = std::floor(free_points/motion_line);
            int totoal_points = occupied_points + motion_line * points_no_each_line;

            float delta_x = abs(init_p2_.x - init_p1_.x);
            float delta_z = abs(init_p3_.z - init_p2_.z);

            float delta_x_each_point = delta_x / ( points_no_each_line + 1.0);
            float delta_z_each_line = delta_z / layer;

            float step_ratio_vertical = 1.0 / layer;

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

                    via_point.x = (1-u) * (1-v) * init_p1_.x + v * (1 - u) * init_p2_.x + u * v * init_p3_.x + u * (1 - v) * init_p4_.x;
                    via_point.y = (1-u) * (1-v) * init_p1_.y + v * (1 - u) * init_p2_.y + u * v * init_p3_.y + u * (1 - v) * init_p4_.y;
                    via_point.z = (1-u) * (1-v) * init_p1_.z + v * (1 - u) * init_p2_.z + u * v * init_p3_.z + u * (1 - v) * init_p4_.z;
                    via_point.rx = (1-u) * (1-v) * init_p1_.rx + v * (1 - u) * init_p2_.rx + u * v * init_p3_.rx + u * (1 - v) * init_p4_.rx;
                    via_point.ry = (1-u) * (1-v) * init_p1_.ry + v * (1 - u) * init_p2_.ry + u * v * init_p3_.ry + u * (1 - v) * init_p4_.ry;
                    via_point.rz = (1-u) * (1-v) * init_p1_.rz + v * (1 - u) * init_p2_.rz + u * v * init_p3_.rz + u * (1 - v) * init_p4_.rz;

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

            /// 2. add on reverse via points
            switch (model_type)
            {
                case yf::data::arm::ModelType::Skirting:
                {
                    break;
                }
                case yf::data::arm::ModelType::Windows:
                {
                    break;
                }
                default:
                {
                    auto via_points_reverse = via_points;

                    std::reverse(via_points_reverse.begin(),via_points_reverse.end());

                    for(int n = 0 ; n < via_points_reverse.size() ; n++)
                    {
                        via_points.push_back(via_points_reverse[n]);
                    }

                    break;
                }
            }

            break;
        }

        case yf::data::arm::MotionType::Line:
        {
            //@@ input: 2 points

            /// 1. Initialization
            init_p1_ = ref_path_init_points[0];
            init_p2_ = ref_path_init_points[1];

            via_points.push_back(init_p1_);
            via_points.push_back(init_p2_);

            /// 2. add on reverse via points

            auto via_points_reverse = via_points;

            std::reverse(via_points_reverse.begin(),via_points_reverse.end());

            for(int n = 0 ; n < via_points_reverse.size() ; n++)
            {
                via_points.push_back(via_points_reverse[n]);
            }

            break;
        }

        case yf::data::arm::MotionType::CircleFull:
        {
            // get center_point_, radius_, v1_, v2_
            this->CenterFit3d(ref_path_init_points);

            // delta_radius
            float delta_radius = radius_/static_cast<float>(layer);

            //
            for (int n = 0; n < layer; n++)
            {
                auto radius = radius_  - n * delta_radius;
                auto step_ratio = step_ratio_horizontal * (n+1);

                std::deque<yf::data::arm::Point3d> via_points_each_layer;

                via_points_each_layer = this->GetCircleEachLayerViaPoints(radius,step_ratio,ref_path_init_points[0]);

                via_points.insert(via_points.end(),via_points_each_layer.begin(),via_points_each_layer.end());
            }

            break;
        }

        case yf::data::arm::MotionType::Curve:
        {
            via_points.insert(via_points.end(), ref_path_init_points.begin(), ref_path_init_points.end());

            using namespace std;

            struct robot_path_data
            {
                int no_of_point;
                double z;
                double point[500][3];

                robot_path_data()
                {
                    no_of_point = 0;
                    z = 0;

                    for (int i = 0; i < 500; i++)
                    {
                        for(int j = 0; j < 3; j++)
                        {
                            point[i][j] = 0;
                        }
                    }
                };
            };

            typedef int(*getoffsetpath)(robot_path_data*,double,robot_path_data*[]);

            double  offset = this->GetCurveOffsetDistance(layer, ref_path_init_points);

            /// ref_init_points ---> double array
            double ptt[500][3];

            // array initialization
            for (auto & i : ptt)
            {
                for(double & j : i)
                {
                    j = 0;
                }
            }

            // ref_init_points ---> array
            for(int n = 0; n < ref_path_init_points.size(); n++)
            {
                ptt[n][0] = ref_path_init_points[n].x;
                ptt[n][1] = ref_path_init_points[n].y;
                ptt[n][2] = 0;
            }

            /// Offset Method input:
            int i,j,no_of_offset_path;

            robot_path_data init_path,*offset_path[50];

            getoffsetpath get_offset_path;

            //HINSTANCE hinstLib = LoadLibrary(TEXT("C:\\Dev\\w0303\\Arm_Control_Module_v1.0\\lib\\polylineoffset.dll"));
            HINSTANCE hinstLib = LoadLibrary(TEXT("C:\\dev\\w0303-ipc1\\lib\\polylineoffset.dll"));

            /// Offset Method input: point size
            init_path.no_of_point = ref_path_init_points.size();

            /// Offset Method output:
            for (i = 0; i < init_path.no_of_point; i++)
            {   for (j=0;j<3;j++)
                    init_path.point[i][j] = ptt[i][j];
            }
            get_offset_path=(getoffsetpath)GetProcAddress(hinstLib, "find_offset_paths");
            no_of_offset_path=get_offset_path(&init_path,offset,offset_path);

            /// yf
            //
            for(int n = 0 ; n < no_of_offset_path; n++)
            {
                std::deque<yf::data::arm::Point3d> ref_path_each_layer;

                /// For Each Layer, Find All Points
                auto each_layer_path = offset_path[n];

                // 1. find each layer's point number.
                int each_layer_points_number = each_layer_path->no_of_point;

                // assign to std container
                for (int m = 0; m < each_layer_points_number; m++)
                {
                    yf::data::arm::Point3d point;

                    point.x = each_layer_path->point[m][0];
                    point.y = each_layer_path->point[m][1];

                    //
                    point.z  = ref_path_init_points[0].z;
                    point.rx = ref_path_init_points[0].rx;
                    point.ry = ref_path_init_points[0].ry;
                    point.rz = ref_path_init_points[0].rz;

                    ref_path_each_layer.push_back(point);
                }

                // rearrange path. check even
                if(n % 2 == 0)
                {
                    std::reverse(ref_path_each_layer.begin(), ref_path_each_layer.end());
                }

                // insert to ref_paths
                via_points.insert(via_points.end(), ref_path_each_layer.begin(), ref_path_each_layer.end());
            }

            break;
        }

        default:
        {
            via_points = ref_path_init_points;
            break;
        }
    }

    return via_points;
}

std::deque<yf::data::arm::Point3d>
yf::algorithm::cleaning_motion::get_uvc_via_points(const yf::data::arm::MotionType &motion_type,
                                                   const std::deque<yf::data::arm::Point3d>& ref_path_init_points,
                                                   const int& layer,
                                                   const float& step_ratio_horizontal)
{
    std::deque<yf::data::arm::Point3d> via_points;

    switch (motion_type)
    {
        case yf::data::arm::MotionType::Plane:
        {
            //@@ input: 4 points

            /// 1. Initialization
            init_p1_ = ref_path_init_points[0];
            init_p2_ = ref_path_init_points[1];
            init_p3_ = ref_path_init_points[2];
            init_p4_ = ref_path_init_points[3];

            int motion_line = layer + 1;
            int occupied_points = motion_line * 2;
            int free_points = sample_points_ - occupied_points;
            int points_no_each_line = std::floor(free_points/motion_line);
            int totoal_points = occupied_points + motion_line * points_no_each_line;

            float delta_x = abs(init_p2_.x - init_p1_.x);
            float delta_z = abs(init_p3_.z - init_p2_.z);

            float delta_x_each_point = delta_x / ( points_no_each_line + 1.0);
            float delta_z_each_line = delta_z / layer;

            float step_ratio_vertical = 1.0 / layer;

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

                for(float v = 0.0 ; v <= 1.0; v = v + step_ratio_horizontal)
                {
                    yf::data::arm::Point3d via_point;

                    // count point_no.
                    point_no ++;

                    // current line point no.
                    cur_line_point_no ++;

                    via_point.x = (1-u) * (1-v) * init_p1_.x + v * (1 - u) * init_p2_.x + u * v * init_p3_.x + u * (1 - v) * init_p4_.x;
                    via_point.y = (1-u) * (1-v) * init_p1_.y + v * (1 - u) * init_p2_.y + u * v * init_p3_.y + u * (1 - v) * init_p4_.y;
                    via_point.z = (1-u) * (1-v) * init_p1_.z + v * (1 - u) * init_p2_.z + u * v * init_p3_.z + u * (1 - v) * init_p4_.z;
                    via_point.rx = (1-u) * (1-v) * init_p1_.rx + v * (1 - u) * init_p2_.rx + u * v * init_p3_.rx + u * (1 - v) * init_p4_.rx;
                    via_point.ry = (1-u) * (1-v) * init_p1_.ry + v * (1 - u) * init_p2_.ry + u * v * init_p3_.ry + u * (1 - v) * init_p4_.ry;
                    via_point.rz = (1-u) * (1-v) * init_p1_.rz + v * (1 - u) * init_p2_.rz + u * v * init_p3_.rz + u * (1 - v) * init_p4_.rz;

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

        case yf::data::arm::MotionType::Line:
        {
            /// 1. Initialization
            init_p1_ = ref_path_init_points[0];
            init_p2_ = ref_path_init_points[1];

            std::deque<yf::data::arm::Point3d> temp_points;

            for(float v = 0.0 ; v <= 1.0; v = v + step_ratio_horizontal)
            {
                yf::data::arm::Point3d via_point;

                via_point.x = (1-v) * init_p1_.x + v * init_p2_.x;
                via_point.y = (1-v) * init_p1_.y + v * init_p2_.y;
                via_point.z = (1-v) * init_p1_.z + v * init_p2_.z;
                via_point.rx = (1-v) * init_p1_.rx + v * init_p2_.rx;
                via_point.ry = (1-v) * init_p1_.ry + v * init_p2_.ry;
                via_point.rz = (1-v) * init_p1_.rz + v * init_p2_.rz;

                temp_points.push_back(via_point);
            }

            via_points = temp_points;

            break;
        }

        case yf::data::arm::MotionType::CircleFull:
        {

            // get center_point_, radius_, v1_, v2_
            this->CenterFit3d(ref_path_init_points);

            // delta_radius
            float delta_radius = radius_/static_cast<float>(layer);

            //
            for (int n = 0; n < layer; n++)
            {
                auto radius = radius_  - n * delta_radius;
                auto step_ratio = step_ratio_horizontal * (n+1);

                std::deque<yf::data::arm::Point3d> via_points_each_layer;

                via_points_each_layer = this->GetCircleEachLayerViaPoints(radius,step_ratio,ref_path_init_points[0]);

                via_points.insert(via_points.end(),via_points_each_layer.begin(),via_points_each_layer.end());
            }

            break;
        }

        case yf::data::arm::MotionType::Curve:
        {
            via_points.insert(via_points.end(), ref_path_init_points.begin(), ref_path_init_points.end());

            using namespace std;

            struct robot_path_data
            {
                int no_of_point;
                double z;
                double point[500][3];

                robot_path_data()
                {
                    no_of_point = 0;
                    z = 0;

                    for (int i = 0; i < 500; i++)
                    {
                        for(int j = 0; j < 3; j++)
                        {
                            point[i][j] = 0;
                        }
                    }
                };
            };

            typedef int(*getoffsetpath)(robot_path_data*,double,robot_path_data*[]);

            double  offset = this->GetCurveOffsetDistance(layer, ref_path_init_points);

            /// ref_init_points ---> double array
            double ptt[500][3];

            // array initialization
            for (auto & i : ptt)
            {
                for(double & j : i)
                {
                    j = 0;
                }
            }

            // ref_init_points ---> array
            for(int n = 0; n < ref_path_init_points.size(); n++)
            {
                ptt[n][0] = ref_path_init_points[n].x;
                ptt[n][1] = ref_path_init_points[n].y;
                ptt[n][2] = 0;
            }

            /// Offset Method input:
            int i,j,no_of_offset_path;

            robot_path_data init_path,*offset_path[50];

            getoffsetpath get_offset_path;

            //HINSTANCE hinstLib = LoadLibrary(TEXT("C:\\Dev\\w0303\\Arm_Control_Module_v1.0\\lib\\polylineoffset.dll"));
            HINSTANCE hinstLib = LoadLibrary(TEXT("C:\\dev\\w0303-ipc1\\lib\\polylineoffset.dll"));

            /// Offset Method input: point size
            init_path.no_of_point = ref_path_init_points.size();

            /// Offset Method output:
            for (i = 0; i < init_path.no_of_point; i++)
            {   for (j=0;j<3;j++)
                    init_path.point[i][j] = ptt[i][j];
            }
            get_offset_path=(getoffsetpath)GetProcAddress(hinstLib, "find_offset_paths");
            no_of_offset_path=get_offset_path(&init_path,offset,offset_path);

            /// yf
            //
            for(int n = 0 ; n < no_of_offset_path; n++)
            {
                std::deque<yf::data::arm::Point3d> ref_path_each_layer;

                /// For Each Layer, Find All Points
                auto each_layer_path = offset_path[n];

                // 1. find each layer's point number.
                int each_layer_points_number = each_layer_path->no_of_point;

                // assign to std container
                for (int m = 0; m < each_layer_points_number; m++)
                {
                    yf::data::arm::Point3d point;

                    point.x = each_layer_path->point[m][0];
                    point.y = each_layer_path->point[m][1];

                    //
                    point.z  = ref_path_init_points[0].z;
                    point.rx = ref_path_init_points[0].rx;
                    point.ry = ref_path_init_points[0].ry;
                    point.rz = ref_path_init_points[0].rz;

                    ref_path_each_layer.push_back(point);
                }

                // rearrange path. check even
                if(n % 2 == 0)
                {
                    std::reverse(ref_path_each_layer.begin(), ref_path_each_layer.end());
                }

                // insert to ref_paths
                via_points.insert(via_points.end(), ref_path_each_layer.begin(), ref_path_each_layer.end());
            }

            break;
        }

        default:
        {
            via_points = ref_path_init_points;
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

void yf::algorithm::cleaning_motion::CenterFit3d(const std::deque<yf::data::arm::Point3d> &init_cleaning_points)
{
    // 1. input points
    std::vector<float> v_p1 = this->Point3d_2_Vector(init_cleaning_points[0]);
    std::vector<float> v_p2 = this->Point3d_2_Vector(init_cleaning_points[1]);
    std::vector<float> v_p3 = this->Point3d_2_Vector(init_cleaning_points[2]);

    // 2. center_point. fix rx,ry,rz
    center_point_.rx = init_cleaning_points[0].rx;
    center_point_.ry = init_cleaning_points[0].ry;
    center_point_.rz = init_cleaning_points[0].rz;

    // 3. Start calculation
    Eigen::RowVectorXf p1 = Eigen::Map<Eigen::Matrix<float, 1, 3> >(v_p1.data());
    Eigen::RowVectorXf p2 = Eigen::Map<Eigen::Matrix<float, 1, 3> >(v_p2.data());
    Eigen::RowVectorXf p3 = Eigen::Map<Eigen::Matrix<float, 1, 3> >(v_p3.data());

    // v1, v2 describe the vectors from p1 to p2 and p3, resp.
    Eigen::RowVector3f v1 = p2 - p1;
    Eigen::RowVector3f v2 = p3 - p1;

    // l1, l2 describe the lengths of those vectors
    float l1 = std::sqrtf(v1(0) * v1(0) + v1(1) * v1(1) + v1(2) * v1(2));
    float l2 = std::sqrtf(v2(0) * v2(0) + v2(1) * v2(1) + v2(2) * v2(2));

    // v1n, v2n describe the normalized vectors v1 and v2
    v1n_ = v1 / l1;
    Eigen::RowVectorXf v2n = v2 / l2;

    // nv describes the normal vector on the plane of the circle
    std::vector<float> v_nv;

    float nv_x = v1n_(1) * v2n(2) - v1n_(2) * v2n(1);
    float nv_y = v1n_(2) * v2n(0) - v1n_(0) * v2n(2);
    float nv_z = v1n_(0) * v2n(1) - v1n_(1) * v2n(0);

    v_nv.push_back(nv_x);
    v_nv.push_back(nv_y);
    v_nv.push_back(nv_z);

    Eigen::RowVectorXf nv = Eigen::Map<Eigen::Matrix<float, 1, 3> >(v_nv.data());

    // v2nb: orthogonalization of v2n against v1n
    float dotp = v2n(0) * v1n_(0) + v2n(1) * v1n_(1) + v2n(2) * v1n_(2);
    v2nb_ = v2n;
    for (int n = 0; n <= 2 ; n++)
    {
        v2nb_(n) = v2nb_(n) - dotp * v1n_(n);
    }
    // normalize v2nb
    float l2nb = std::sqrtf((v2nb_(0) * v2nb_(0) + v2nb_(1) * v2nb_(1) + v2nb_(2) * v2nb_(2)));

    v2nb_ = v2nb_ / l2nb;

    // remark: the circle plane will now be discretized as follows
    //
    // origin: p1                    normal vector on plane: nv
    // first coordinate vector: v1n  second coordinate vector: v2nb
    //
    // calculate 2d coordinates of points in the plane
    // p1_2d = zeros(n,2); % set per construction
    // p2_2d = zeros(n,2);p2_2d(:,1) = l1; % set per construction

    // p3_2d = zeros(n,2); // has to be calculated
    Eigen::RowVector2f p3_2d; p3_2d(0) = 0; p3_2d(1) = 0;

    for (int n = 0; n <= 2 ; n++)
    {
        p3_2d(0) = p3_2d(0) + v2(n) * v1n_(n);
        p3_2d(1) = p3_2d(1) + v2(n) * v2nb_(n);
    }

    // calculate the fitting circle
    // due to the special construction of the 2d system this boils down to solving
    // q1 = [0,0], q2 = [a,0], q3 = [b,c] (points on 2d circle)
    // crossing perpendicular bisectors, s and t running indices:
    // solve [a/2,s] = [b/2 + c*t, c/2 - b*t]
    // solution t = (a-b)/(2*c)

    float a = l1; float b = p3_2d(0); float c = p3_2d(1);
    float t = 0.5*(a-b)/c;
    float scale1 = b/2 + c*t;
    float scale2 = c/2 - b*t;

    // center_point. x,y,z
    center_point_.x = p1(0) + scale1 * v1n_(0) + scale2 * v2nb_(0);
    center_point_.y = p1(1) + scale1 * v1n_(1) + scale2 * v2nb_(1);
    center_point_.z = p1(2) + scale1 * v1n_(2) + scale2 * v2nb_(2);

    // radius
    radius_ = std::sqrtf((center_point_.x-p1(0))*(center_point_.x-p1(0)) +
                             (center_point_.y-p1(1))*(center_point_.y-p1(1)) +
                             (center_point_.z-p1(2))*(center_point_.z-p1(2)));

    return;
}

std::vector<float> yf::algorithm::cleaning_motion::Point3d_2_Vector(const yf::data::arm::Point3d &point)
{
    std::vector<float> v_point;
    v_point.clear();

    v_point.push_back(point.x); v_point.push_back(point.y); v_point.push_back(point.z);
    v_point.push_back(point.rx); v_point.push_back(point.ry); v_point.push_back(point.rz);

    return v_point;
}

void yf::algorithm::cleaning_motion::set_offset_distance(const float &distance)
{
    offset_distance_ = distance;
    return;
}

yf::data::arm::Point3d yf::algorithm::cleaning_motion::get_center()
{
    return center_point_;
}

float yf::algorithm::cleaning_motion::get_radius()
{
    return radius_;
}

Eigen::RowVectorXf yf::algorithm::cleaning_motion::get_v1n()
{
    return v1n_;
}

Eigen::RowVector3f yf::algorithm::cleaning_motion::get_v2nb()
{
    return v2nb_;
}

std::deque<yf::data::arm::Point3d> yf::algorithm::cleaning_motion::GetCircleEachLayerViaPoints(const float& radius,
                                                                                               const float& step_ratio_horizontal,
                                                                                               const yf::data::arm::Point3d& init_point)
{
    std::deque<yf::data::arm::Point3d> via_points_layer;

    std::deque<yf::data::arm::Point3d> via_points_a;
    std::deque<yf::data::arm::Point3d> via_points_b;

    via_points_a.clear();
    via_points_b.clear();

    yf::data::arm::Point3d via_point_end;

    via_point_end.x = center_point_.x + sin(1/180.0*PI) * radius * v1n_(0) + cos(1/180.0*PI) * radius * v2nb_(0);
    via_point_end.y = center_point_.y + sin(1/180.0*PI) * radius * v1n_(1) + cos(1/180.0*PI) * radius * v2nb_(1);
    via_point_end.z = center_point_.z + sin(1/180.0*PI) * radius * v1n_(2) + cos(1/180.0*PI) * radius * v2nb_(2);
    via_point_end.rx = center_point_.rx;
    via_point_end.ry = center_point_.ry;
    via_point_end.rz = center_point_.rz;

    // via_points, with center_point_ and radius_
    for(int i = 1; i <= 361; i = i+step_ratio_horizontal*360)
    {
        yf::data::arm::Point3d via_point;

        float a = i/180.0*PI;
        via_point.x = center_point_.x + sin(a) * radius * v1n_(0) + cos(a) * radius * v2nb_(0);
        via_point.y = center_point_.y + sin(a) * radius * v1n_(1) + cos(a) * radius * v2nb_(1);
        via_point.z = center_point_.z + sin(a) * radius * v1n_(2) + cos(a) * radius * v2nb_(2);
        via_point.rx = center_point_.rx;
        via_point.ry = center_point_.ry;
        via_point.rz = center_point_.rz;

        via_points_layer.push_back(via_point);
    }

    // search via_p0

    int index = 0;

    float delta_distance = (init_point.x - via_points_layer[0].x) * (init_point.x - via_points_layer[0].x) +
                           (init_point.y - via_points_layer[0].y) * (init_point.y - via_points_layer[0].y) +
                           (init_point.z - via_points_layer[0].z) * (init_point.z - via_points_layer[0].z);

    for (int n = 0; n < via_points_layer.size(); n++)
    {
        float delta_distance_temp = (init_point.x - via_points_layer[n].x) * (init_point.x - via_points_layer[n].x) +
                                    (init_point.y - via_points_layer[n].y) * (init_point.y - via_points_layer[n].y) +
                                    (init_point.z - via_points_layer[n].z) * (init_point.z - via_points_layer[n].z);
        if(delta_distance > delta_distance_temp)
        {
            delta_distance = delta_distance_temp;

            index = n;
        }
    }

    if(index != 0)
    {
        via_points_a.insert(via_points_a.begin(),via_points_layer.begin()+index,via_points_layer.end());
        via_points_b.insert(via_points_b.begin(), via_points_layer.begin(),via_points_layer.begin()+index-1);

        via_points_layer.clear();

        via_points_layer.insert(via_points_layer.end(),via_points_a.begin(),via_points_a.end());
        via_points_layer.insert(via_points_layer.end(),via_points_b.begin(),via_points_b.end());
    }

    via_points_layer.push_back(via_point_end);

    return via_points_layer;
}

double yf::algorithm::cleaning_motion::GetCurveOffsetDistance(const int &layer_no,
                                                              const std::deque<yf::data::arm::Point3d> &ref_init_points)
{
    auto start_point = ref_init_points[0];
    auto end_point = ref_init_points[ref_init_points.size()-1];

    float   vec_max_magnitude = 0;
    int     farthest_point_no;

    for(int n = 1; n < ref_init_points.size()-1 ; n ++)
    {
        auto vec_1_x = start_point.x - ref_init_points[n].x;
        auto vec_1_y = start_point.y - ref_init_points[n].y;

        auto vec_2_x = end_point.x - ref_init_points[n].x;
        auto vec_2_y = end_point.y - ref_init_points[n].y;

        auto vec_3_x = (vec_1_x+vec_2_x)/2;
        auto vec_3_y = (vec_1_y+vec_2_y)/2;

        auto vec_3_magnitude = std::sqrtf(vec_3_x*vec_3_x + vec_3_y*vec_3_y);

        if(vec_max_magnitude < vec_3_magnitude)
        {
            vec_max_magnitude = vec_3_magnitude;
            farthest_point_no = n+1;
        }
    }

    return vec_max_magnitude/(layer_no+1);
}

void yf::algorithm::TimeSleep::ms(const int &ms)
{
    ///TIME
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

void yf::algorithm::TimeSleep::minute(const int &min)
{
    auto min_r = min * 1000 * 60;
    ///TIME
    std::this_thread::sleep_for(std::chrono::milliseconds(min_r));
}

void yf::algorithm::TimeSleep::sec(const int &sec)
{
    auto result = sec * 1000;
    ///TIME
    std::this_thread::sleep_for(std::chrono::milliseconds(result));
}
