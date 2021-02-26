#include <iostream>
#include <string>
#include <stdio.h>
#include <time.h>
#include <deque>
#include <vector>
#include <chrono>


// Get current date/time, format is YYYY-MM-DD.HH:mm:ss
const std::string currentDateTime() {

    struct tm newtime;

    __time64_t long_time;

    errno_t err;

    // Get time as 64-bit integer.
    _time64( &long_time );
    // Convert to local time.
    err = _localtime64_s( &newtime, &long_time );
    if (err)
    {
        printf("Invalid argument to _localtime64_s.");
        exit(1);
    }

    newtime.tm_year = 1900 + newtime.tm_year;

    std::string year = std::to_string(newtime.tm_year);
    std::string month;
    std::string day;

    if(newtime.tm_mon < 10)
    {
        month = "0"+std::to_string(newtime.tm_mon);
    }
    else
    {
        month = std::to_string(newtime.tm_mon);
    }

    if(newtime.tm_mday < 10)
    {
        day = "0"+std::to_string(newtime.tm_mday);
    }
    else
    {
        day = std::to_string(newtime.tm_mday);
    }

    std::string hour = std::to_string(newtime.tm_hour);
    std::string min = std::to_string(newtime.tm_min);
    std::string sec = std::to_string(newtime.tm_sec);
    /// "2021-01-28 18:02:00.000"

    std::string  time = year + "-" + month + "-" + day + " " + hour + ":" + min + ":" + sec+ ".000";

    return time;
}

std::vector<float> time_str2vector(std::string str)
{
    std::vector<float> time;

    auto char_index1 = str.find("-");
    auto char_index2 = str.find_last_of("-");

    auto char_index3 = str.find(" ");

    auto char_index4 = str.find(":");
    auto char_index5 = str.find_last_of(":");

    auto char_index6 = str.find(".");

    auto str_year = str.substr(0,char_index1);
    auto str_month = str.substr(char_index1+1, char_index2-char_index1-1);
    auto str_day = str.substr(char_index2+1, char_index3-char_index2-1);
    auto str_hour = str.substr(char_index3+1, char_index4-char_index3-1);
    auto str_min = str.substr(char_index4+1, char_index5-char_index4-1);
    auto str_second = str.substr(char_index5+1, char_index6-char_index5-1);

    std::cout << str_year << " " << str_month << " "<< str_day << " "<< str_hour << " "<< str_min << " "<< str_second << std::endl;

    time.push_back(std::stof(str_year));
    time.push_back(std::stof(str_month));
    time.push_back(std::stof(str_day));
    time.push_back(std::stof(str_hour));
    time.push_back(std::stof(str_min));
    time.push_back(std::stof(str_second));

    return time;
}

bool IsLatestSchedule(std::string time_from_database, std::string time_now)
{
    std::vector<float> time_db;
    std::vector<float> time_current;

    time_db = time_str2vector(time_from_database);
    time_current = time_str2vector(time_now);

    if(time_current[0] > time_db[0])
    {
        return false;
    }
    else
        if(time_current[0] < time_db[0])
        {
            return true;
        }
        else
        {
            if(time_current[1] > time_db[1])
            {
                return false;
            }
            else if(time_current[1] < time_db[1])
            {
                return true;
            }
            else
                if(time_current[2] > time_db[2])
                    {
                        return false;
                    }
                else if (time_current[2] < time_db[2])
                {
                    return true;
                }
                else
                    if(time_current[3] > time_db[3])
                    {
                        return false;
                    }
                    else if (time_current[3] < time_db[3])
                    {
                        return true;
                    }
                    else
                        if(time_current[4] > time_db[4])
                        {
                            return false;
                        }
                        else if(time_current[4] < time_db[4])
                        {
                            return true;
                        }
                        else
                            if(time_current[5] >=  time_db[5])
                            {
                                return false;
                            }
                            else
                            {
                                return true;
                            }
        }

}

std::string GetCurTime()
{
    struct tm newtime;
    char am_pm[] = "AM";
    __time64_t long_time;
    char timebuf[26];
    errno_t err;

    // Get time as 64-bit integer.
    _time64( &long_time );
    // Convert to local time.
    err = _localtime64_s( &newtime, &long_time );
    if (err)
    {
        printf("Invalid argument to _localtime64_s.");
        exit(1);
    }


    newtime.tm_year = 1900 + newtime.tm_year;
    newtime.tm_mon = 1 + newtime.tm_mon;

    std::string year = std::to_string(newtime.tm_year);
    std::string month;
    std::string day;

    if(newtime.tm_mon < 10)
    {
        month = "0"+std::to_string(newtime.tm_mon);
    }
    else
    {
        month = std::to_string(newtime.tm_mon);
    }

    if(newtime.tm_mday < 10)
    {
        day = "0"+std::to_string(newtime.tm_mday);
    }
    else
    {
        day = std::to_string(newtime.tm_mday);
    }

    std::string hour = std::to_string(newtime.tm_hour);
    std::string min = std::to_string(newtime.tm_min);
    std::string sec = std::to_string(newtime.tm_sec);
    /// "2021-01-28 18:02:00.000"

    std::string  time = year + "-" + month + "-" + day + " " + hour + ":" + min + ":" + sec+ ".000";

    return time;
}

/// "2021-01-28 18:02:00.000"

int main() {

    std::cout << "currentDateTime()=" << currentDateTime() << std::endl;

    std::deque<double> time_split;

    std::string time_from_database  = "2021-02-03 16:58:50.000";
    std::string time_now = currentDateTime();

    std::cout << "time_now: " << time_now << std::endl;

    auto time_flag = IsLatestSchedule(time_from_database,time_now);

    std::cout << "is latest schedule? " << time_flag << std::endl;

    getchar();  // wait for keyboard input

    auto s = GetCurTime();

    std::cout << "qweqwe: "<< s << std::endl;
}