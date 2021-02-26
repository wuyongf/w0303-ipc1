//
// Created by yongf on 2021/1/21.
//
#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>

struct ScriptData
{
    int script_id = 1;
    std::string script;
};

struct LnMessage
{
    std::string header = "$TMSCT";

    int length;

    ScriptData data;

    std::string checksum;
};

struct LnMotionMessage
{
    std::string target_point_name;

    std::string motion_type         = "PLine";         // e.g. "PLine" or "PTP"
    std::string motion_structure    = "CAP";           // e.g. "CAP" --> coordinate ; "JAP" --> joints

    int speed                       = 100;             // mm/s
    int time_to_top_speed           = 200;             // ms
    int blending_percentage         = 100;             // 100? 50?
};























