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

    p1.x = 0.8143;  p1.y = 0.2435; p1.z = 0.9293; p1.rx = 0; p1.ry = 0; p1.rz = 0;
    p2.x = 0.35;    p2.y = 0.1966; p2.z = 0.2511; p2.rx = 0; p2.ry = 0; p2.rz = 0;
    p3.x = 0.616;   p3.y = 0.4733; p3.z = 0.3517; p3.rx = 0; p3.ry = 0; p3.rz = 0;

    std::deque<yf::data::arm::Point3d> points;

    points.push_back(p1);
    points.push_back(p2);
    points.push_back(p3);

    al_motion.CenterFit3d(points);
}