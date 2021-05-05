//
// Created by yongf on 2021/4/28.
//

#include "../include/al.h"

int main()
{
    yf::algorithm::cleaning_motion al_motion;

    yf::data::arm::Point3d p1;
    yf::data::arm::Point3d p2;
    yf::data::arm::Point3d p3;

    p1.x = 0.8143;  p1.y = 0.2435; p1.z = 0.9293; p1.rx = -162.17; p1.ry = -2.01; p1.rz = -89.74;
    p2.x = 0.35;    p2.y = 0.1966; p2.z = 0.2511; p2.rx = -163.02; p2.ry = -2.01; p2.rz = -89.58;
    p3.x = 0.616;   p3.y = 0.4733; p3.z = 0.3517; p3.rx = -161.62; p3.ry = -1.62; p3.rz = -93.82;

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



}