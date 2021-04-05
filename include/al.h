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

            std::deque<yf::data::arm::Point3d> get_via_points(const int& motion_type, std::deque<yf::data::arm::Point3d> init_cleaning_points);

        private:

            // shared database
            std::shared_ptr<yf::sql::sql_server> sql_ptr_;

            // properties

            int layer_          = 5; // at least 1
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

            void ExportRealPathByLM()
            {
                //todo: Find T6 = T4*inv(T1)*T3
                // 1. retrieve T2
                // 2. Find LM2 position. T4
                // 3. calculate the real_path.

            }


        };
    }
}

void yf::algorithm::arm_path::RecordRefPath()
{
    //todo:
    // 1. read ref_path from .txt file.
    // 2. record the LM1 position.
    // 3. record the transformation T2. T3(ref_path to base) = T1(LM1 to base)* T2;
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

std::deque<yf::data::arm::Point3d> yf::algorithm::cleaning_motion::get_via_points(const int &motion_type,
                                                                                  std::deque<yf::data::arm::Point3d> init_cleaning_points)
{
    std::deque<yf::data::arm::Point3d> via_points;

    switch (motion_type)
    {
        case 1: // plain_cleaning
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

        case 2: // line_cleaning
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