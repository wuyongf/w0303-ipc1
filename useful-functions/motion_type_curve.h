//
// Created by yongf on 2021/5/4.
//
#include "../include/data_arm.h"
#include "../include/al_common.h"

#pragma once

std::deque<yf::data::arm::Point3d> CurveFit3d(const std::deque<yf::data::arm::Point3d>& init_points);