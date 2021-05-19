//
// Created by yongf on 2021/4/14.
//

#include "../include/sql.h"

#include "../include/al.h"

#include <iostream>
#include <chrono>
#include <iomanip>

template <typename Clock, typename Duration>
std::ostream& operator<<(std::ostream& os, const std::chrono::time_point<Clock, Duration>& timep)
{
    auto converted_timep = Clock::to_time_t(timep);
    os << std::put_time(std::localtime(&converted_timep), "%Y %b %d %H:%M:%S");
    return os;
}

auto convert_to_timepoint(int years, int months, int days, int hours, int mins, int secs)
{
    years -= 1900; //epoch
    std::tm date = {};
    date.tm_year = years;
    date.tm_mon = months;
    date.tm_mday = days;
    date.tm_hour = hours;
    date.tm_min = mins;
    date.tm_sec = secs;

    return std::chrono::system_clock::from_time_t(std::mktime(&date));
}

template <typename Clock, typename Duration>
auto add_days(const std::chrono::time_point<Clock, Duration>& timepoint, int days_to_add)
{
    constexpr std::time_t seconds_in_day = 60 * 60 * 24;
    //                                     mm   hh   dd

    std::time_t days = seconds_in_day * days_to_add;
    auto date = Clock::to_time_t(timepoint);
    return Clock::from_time_t(date + days);
}

template <typename Clock, typename Duration>
auto add_seconds(const std::chrono::time_point<Clock, Duration>& timepoint, int seconds_to_add)
{
    std::time_t seconds =  seconds_to_add;
    auto date = Clock::to_time_t(timepoint);
    return Clock::from_time_t(date + seconds);
}

template <typename Clock, typename Duration>
std::string convert_to_db_time(const std::chrono::time_point<Clock, Duration>& timep)
{
    auto converted_timep = Clock::to_time_t(timep);

    std::tm date = *std::localtime(&converted_timep);

    std::string db_time =   std::to_string(date.tm_year + 1900) + "-" +
                            std::to_string(date.tm_mon + 1) + "-" +
                            std::to_string(date.tm_mday) + " " +
                            std::to_string(date.tm_hour) + ":" +
                            std::to_string(date.tm_min) + ":" +
                            std::to_string(date.tm_sec) + ".000";

    std::cout << "db_time: " << db_time << std::endl;

    return db_time;
}

int main()
{

    /// For IPC DB
    std::shared_ptr<yf::sql::sql_server> sql_ptr = std::make_shared<yf::sql::sql_server>("SQL Server","192.168.7.27","NW_mobile_robot_sys","sa","NWcadcam2021");

    /// For localhost DB
//    std::shared_ptr<yf::sql::sql_server> sql_ptr = std::make_shared<yf::sql::sql_server>("ODBC Driver 17 for SQL Server","localhost","NW_mobile_robot_sys","sa","wuyongfeng1334");

    sql_ptr->OSGetScheduleReady(5026,10);

    /// get model_type
//    std::cout << "model_type: " << sql_ptr->GetModelType(3)<< std::endl;

    return 1;
}