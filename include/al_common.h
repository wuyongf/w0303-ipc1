#pragma once

#include <memory>
#include <thread>
#include <mutex>
#include <deque>
#include <optional>
#include <vector>

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cmath>

//
//#include "../include/boost_1_75_0/boost/asio.hpp"
//#include "../include/boost_1_75_0/boost/asio/ts/buffer.hpp"
//#include "../include/boost_1_75_0/boost/asio/ts/internet.hpp"

const float PI = std::atan(1.0)*4;

/// uint32_t to float

float unit32_to_float(const uint16_t& high_16, const uint16_t& low_16)
{
    union uuf
    {
        uint32_t x;
        float f;
    };

    uint16_t data[2]={high_16,low_16};

    uuf u;

    u.x = (((unsigned long)data[0] << 16) | data[1]);

    float result = u.f;

    return result;
}
