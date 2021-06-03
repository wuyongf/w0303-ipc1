#pragma once

#include <iostream>
#include <string>
#include <stdio.h>
#include <time.h>
#include <deque>
#include <vector>
#include <chrono>
#include <exception>
#include <thread>
#include <iomanip>

//glog
#include <glog/logging.h>

#include <nanodbc/nanodbc.h>

#include "data.h"

namespace yf
{
    namespace sql
    {
        class sql_server
        {
        public:
            sql_server();
            sql_server(const std::string& driver,const std::string& server,const std::string& database,
                       const std::string& Uid, const std::string& Pwd );
            ~sql_server();

        public:
            // setter
            void setODBCConnectionStr(const std::string &driver, const std::string &server, const std::string &database,
                                      const std::string &Uid, const std::string &Pwd);

            // getter
            std::string getODBCConnectionStr();

            std::string get_execute_time();

            // properties
            bool isConnected();

            void Connect();

            void Disconnect();

        private:
            std::mutex muxConn;

        public:
            /// time method
            //
            std::string TimeNow();
            void UpdateTime();

            // time getter
            std::string get_time_element(const std::string& element);

            std::vector<float> Time_str2vector(std::string str);

            bool isFutureTime(std::string time_from_database, std::string time_now);

            std::string CountdownTime(const std::string& time_now, const int& countdown_min);

            std::string CountdownTimeSec(const int& countdown_sec);

            void WaitForExecute(const std::string &execute_time);

        public:
            //
            //methods
            //
            // Schedules
            // dataReader

            std::deque<int> GetAvailableScheuldesId();

            std::deque<int> GetJobsId(const int& cur_schedule_id);

            std::deque<int> GetTasksId(const int& cur_job_id);

            void GetScheduleExecTime(const int& id);

            // Filter
            // int GetTaskMode(const int& model_config_id);

            // Schedule
            //
            void UpdateScheduleData(const int& schedule_id, const int& schedule_status);

            void UpdateScheduleLog(const int& schedule_id, const int& schedule_status);

            // Job
            //
            void UpdateJobData(const int& cur_job_id, const int& job_status);
            void UpdateJobLog(const int& cur_job_id, const int& job_status);

            // Task
            //
            int GetTaskCommand(const int& cur_task_id);

            void UpdateTaskData(const int& cur_task_id, const int& task_status);
            void UpdateTaskLog(const int& cur_task_id, const int& task_status);

            // Device Status
            //
            void UpdateDeviceConnectionStatus(const std::string& device_name, const int& connection_status);

            void UpdateDeviceMissionStatusLog(const std::string& device_name, const int& mission_status);
            void UpdateDeviceMissionStatus(const std::string& device_name, const int& mission_status);

            void UpdateDeviceBatteryCapacity(const std::string& device_name, const float& battery_capacity);
            void UpdateDeviceUgvCurPosition(const float& x, const float& y,const float& theta);

            /// Sys Status
            //
            // sys control status
            int GetSysControlMode();
            int GetScheduleCommand(const int& id);


            // Model_config
            int GetModelConfigId(const int& cur_job_id);

            int GetModelConfigElement(const int& model_config_id, const std::string& element);

            /// Arm_mission_config
            //
            int GetArmConfigId(const int& model_config_id, const int& cur_order);
            int GetArmConfigIsValid(const int& arm_config_id);

            std::deque<int> GetArmMissionConfigIds(const int& arm_config_id);

            int GetArmMissionPointId(const int& arm_mission_config_id, const std::string& arm_point_name);

            int GetArmMotionType(const int& arm_mission_config_id);
            int GetTaskMode(const int& model_config_id);
            int GetOperationArea(const int& arm_config_id);
            int GetToolAngle(const int& arm_mission_config_id);
            int GetMotionType(const int& arm_mission_config_id);

            // retrieve point(x,y,z,rx,ry,rz) from table "data_arm_points"
            int GetStandbyPositionId(const int& arm_config_id);
            yf::data::arm::Point3d GetArmPoint(const int& point_id);
            float GetArmPointElement(const int &point_id, const std::string &point_element);

            // retrieve data from table "data_arm_mc_ref_landmark_pos"
            int GetArmRefLMPosId(const int& arm_mission_config_id);
            yf::data::arm::Point3d GetArmRefLMPos(const int& pos_id);
            float GetArmRefLMPosElement(const int &pos_id, const std::string &pos_element);  // private

            // Landmark Flag
            int GetLandmarkFlag(const int &arm_mission_config_id);

            // for ref_path_init_point
            // (1) table "data_arm_points"
            // (2) table "data_arm_mc_ref_path_init_points"
            std::deque<yf::data::arm::Point3d> GetRefPathInitPoints(const int& arm_mission_config_id);

            std::deque<int> GetRefPathInitPointIds(const int& arm_mission_config_id);
            yf::data::arm::Point3d GetRefPathInitPoint(const int& point_id);
            float GetRefPathInitPointElement(const int &point_id, const std::string &point_element);

            int GetRefPathLayerNo(const int &arm_mission_config_id);
            float GetRefPathStepRatioHorizontal(const int &arm_mission_config_id);

            // Ref Landmark Pos Configuration
            void InsertRefLandmarkPos(const int& arm_mission_config_id, const yf::data::arm::Point3d& pos);


            /// Ugv_mission_config
            int GetUgvMissionConfigNum(const int& model_config_id);

            std::vector<int> GetArmConfigValidResultQueue(const int& model_config_id);
            int GetFirstValidOrder(const int& model_config_id);
            int GetLastValidOrder(const int& model_config_id);
            std::vector<int> GetValidIndexes(const int& model_config_id);

            std::deque<std::string> GetUgvMissionConfigPositionNames(const int& model_config_id);

            int GetModelId(const int& model_config_id);
            int GetModelType(const int& model_config_id);
            std::string GetModelName(const int& model_config_id);

            int GetMapIdFromModelId(const int& model_id);
            std::string GetMapElement(const int& map_id, const std::string& map_element);
            std::string GetSiteInfo(const int& model_config_id);
            std::string GetBuildingInfo(const int& model_config_id);
            std::string GetFloorInfo(const int& model_config_id);

            std::string GetMapNameFromMapStatus();

            int GetUgvInitPositionIdFromMapStatus();
            float GetUgvInitPositionElement(const int& id, const std::string& element);
            std::vector<float> GetUgvInitPositionFromMapStatus();

            /// Ugv Initialization for Each Schedule
            std::string GetActivatedMapName();


        private:
            bool static IsOne(int x){return x == 1;}
        public:

            /// Onsite setup
            void OSGetScheduleReady(const int& schedule_id, const int& seconds_later_from_now);


        private: /// Onsite Setup: method 1: stl time ---> db time

            std::string get_future_db_time(const int& seconds_later_from_now);

            auto convert_to_timepoint(int years, int months, int days, int hours, int mins, int secs);
            template <typename Clock, typename Duration>
            auto add_seconds(const std::chrono::time_point<Clock, Duration>& timepoint, int seconds_to_add);
            template <typename Clock, typename Duration>
            std::string convert_to_db_time(const std::chrono::time_point<Clock, Duration>& timep);

            // Onsite setup: method 2: set relevant job
        private:


            /// properties
        private:

            nanodbc::connection conn_;

            // connection info
            std::string ODBCConnectionStr_ = "" ;
            std::string driver_ = "SQL Server" ;
            std::string server_ = "192.168.7.27";
            std::string database_ = "NW_mobile_robot_sys";
            std::string Uid_ = "sa";
            std::string Pwd_ = "NWcadcam2021";

            // properties
            std::string time_now_;
            std::string time_year_;
            std::string time_month_;
            std::string time_day_;
            std::string time_hour_;
            std::string time_minute_;
            std::string time_sec_;

            std::deque<int> available_jobs;
            std::deque<int> available_tasks;

            std::string schedule_id_execute_time;

            std::vector<int> GetFirstAndLastValidOrderNo(const int &model_config_id);
        };
    }
}

yf::sql::sql_server::sql_server()
{
    //"Driver={SQL Server};Server=192.168.0.8;Database=NW_mobile_robot_system;Uid=sa;Pwd=Willsonic2010"
    ODBCConnectionStr_ = "Driver={"+driver_+"};Server="+server_+";Port=1433;Database="+database_+";Uid="+Uid_+";Pwd="+Pwd_;
}

yf::sql::sql_server::sql_server(const std::string &driver, const std::string &server, const std::string &database,
                                const std::string &Uid, const std::string &Pwd)
        : driver_(driver),  server_(server), database_(database), Uid_(Uid), Pwd_(Pwd)
{
    ODBCConnectionStr_ = "Driver={"+driver_+"};Server="+server_+";Port=1433;Database="+database_+";Uid="+Uid_+";Pwd="+Pwd_;
}

yf::sql::sql_server::~sql_server()
{

}

void yf::sql::sql_server::setODBCConnectionStr(const std::string &driver, const std::string &server, const std::string &database,
                                               const std::string &Uid, const std::string &Pwd)
{
    ODBCConnectionStr_ = "Driver={"+driver+"};Server="+server+";Database="+database+";Uid="+Uid+";Pwd="+Pwd;
}

std::string yf::sql::sql_server::getODBCConnectionStr()
{
    return ODBCConnectionStr_;
}

bool yf::sql::sql_server::isConnected()
{
    return conn_.connected();
}

void yf::sql::sql_server::Connect()
{

    try
    {
        std::unique_lock lock(muxConn);
        nanodbc::connection connection(ODBCConnectionStr_);
        conn_ = connection;
    }
    catch (std::exception& e)
    {  
        std::cerr << e.what() << std::endl;
        std::cerr << "EXIT_FAILURE: " << EXIT_FAILURE << std::endl;
    }
}

void yf::sql::sql_server::Disconnect()
{
    try
    {
        conn_.disconnect();
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << "EXIT_FAILURE: " << EXIT_FAILURE << std::endl;
    }
}

std::string yf::sql::sql_server::TimeNow()
{
    struct tm newtime;

//    char am_pm[] = "AM";

    __time64_t long_time;

//    char timebuf[26];

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

    time_now_ = year + "-" + month + "-" + day + " " + hour + ":" + min + ":" + sec+ ".000";

    return time_now_;
}

void yf::sql::sql_server::UpdateTime()
{
    struct tm newtime;

//    char am_pm[] = "AM";

    __time64_t long_time;

//    char timebuf[26];

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

    time_year_ = year;
    time_month_ = month;
    time_day_ = day;
    time_hour_ = hour;
    time_minute_ = min;
    time_sec_ = sec;

    return ;
}

// @@ input: year/month/day/hour/min
std::string yf::sql::sql_server::get_time_element(const std::string &element)
{
    if(element == "year")
    {
        return time_year_;
    }
    else if (element == "month")
    {
        return time_month_;
    }
    else if (element == "day")
    {
        return time_day_;
    }
    else if (element == "hour")
    {
        return time_hour_;
    }
    else if (element == "minute")
    {
        return time_minute_;
    }
    else if (element == "sec")
    {
        return time_sec_;
    }
}

void yf::sql::sql_server::UpdateScheduleData(const int& schedule_id, const int& schedule_status)
{
    std::string status_str = std::to_string(schedule_status);
    std::string id_str = std::to_string(schedule_id);
    std::string query_update;

    try
    {
        Connect();

        switch (schedule_status)
        {
            case 2: // in processing
            {
                query_update = "UPDATE sys_schedule SET status = " + status_str + ", actual_start='"+TimeNow()+"' WHERE ID = "+ id_str;
                break;
            }

            case 3: // finish
            {
                query_update = "UPDATE sys_schedule SET status = " + status_str + ", actual_end='"+TimeNow()+"' WHERE ID = "+ id_str;
                break;
            }

            case 4: // cancel
            {
                query_update = "UPDATE sys_schedule SET status = " + status_str + ", actual_end='"+TimeNow()+"' WHERE ID = "+ id_str;
                break;
            }

            case 5: // error
            {
                query_update = "UPDATE sys_schedule SET status = " + status_str + ", actual_end='"+TimeNow()+"' WHERE ID = "+ id_str;
                break;
            }

            case 6: // pause
            {
                query_update = "UPDATE sys_schedule SET status = " + status_str + ", actual_end='"+TimeNow()+"' WHERE ID = "+ id_str;
                break;
            }

        }

        nanodbc::execute(conn_,query_update);

        Disconnect();
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << "EXIT_FAILURE: " << EXIT_FAILURE << std::endl;
    }
}

void yf::sql::sql_server::UpdateScheduleLog(const int &schedule_id, const int &schedule_status)
{
    std::string status_str = std::to_string(schedule_status);
    std::string schedule_id_str = std::to_string(schedule_id);
    std::string query_update;

    try
    {
        Connect();

        switch (schedule_status)
        {
            case 2: // in processing
            {
                query_update = "INSERT INTO sys_schedule_log(schedule_id, actual_start, status) VALUES (" +
                               schedule_id_str + ",'" + TimeNow() + "'," + status_str + ")";
                break;
            }

            default:
            {
                query_update = "UPDATE sys_schedule_log SET status = " + status_str + ", actual_end='" + TimeNow() + "' WHERE schedule_id = " + schedule_id_str + " AND status=2";
                break;
            }

        }

        nanodbc::execute(conn_,query_update);

        Disconnect();
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << "EXIT_FAILURE: " << EXIT_FAILURE << std::endl;
    }
}

std::deque<int> yf::sql::sql_server::GetJobsId(const int& cur_schedule_id)
{
    std::string query_update;

    std::string schedule_id_str = std::to_string(cur_schedule_id);

    //"SELECT ID FROM schedule_table where status=1 AND planned_start > '2021-02-06 11:10:08.000'"
    try
    {
        Connect();

        query_update = "SELECT job_id FROM sys_schedule_job where schedule_id = " + schedule_id_str + " ORDER BY job_order" ;

        auto result = nanodbc::execute(conn_,query_update);

        // if there are new schedules available, sql module will mark down all the available schedule ids
        while(result.next())
        {
            std::string id_str = result.get<std::string>(0, "null");

            int id = std::stoi(id_str);

            available_jobs.push_back(id);

        };

        Disconnect();

        std::deque<int> q_jobs_id = available_jobs;

        available_jobs.clear();

        return q_jobs_id;
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << "EXIT_FAILURE: " << EXIT_FAILURE << std::endl;
    }
}

std::deque<int> yf::sql::sql_server::GetTasksId(const int& cur_job_id)
{
    std::string query_update;

    std::string job_id_str = std::to_string(cur_job_id);

    //"SELECT ID FROM schedule_table where status=1 AND planned_start > '2021-02-06 11:10:08.000'"
    try
    {
        Connect();

        query_update = "SELECT ID FROM sys_schedule_job_task where job_id = " + job_id_str + " ORDER BY task_order" ;

        auto result = nanodbc::execute(conn_,query_update);

        // if there are new schedules available, sql module will mark down all the available schedule ids
        while(result.next())
        {
            std::string id_str = result.get<std::string>(0, "null");

            int id = std::stoi(id_str);

            available_tasks.push_back(id);

        };

        Disconnect();

        std::deque<int> q_tasks_id = available_tasks;

        available_tasks.clear();

        return q_tasks_id;
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << "EXIT_FAILURE: " << EXIT_FAILURE << std::endl;
    }


}

void yf::sql::sql_server::GetScheduleExecTime(const int &id)
{
    std::string query_update;

    //"SELECT ID FROM schedule_table where status=1 AND planned_start > '2021-02-06 11:10:08.000'"
    try
    {
        Connect();

        query_update = "SELECT planned_start FROM sys_schedule where ID = "+std::to_string(id)+" AND status=1 AND planned_start > '"+ TimeNow() +"'";

        auto result = nanodbc::execute(conn_,query_update);

        while(result.next())
        {
            std::string time_str = result.get<std::string>(0, "null");
            auto index = time_str.find("+");
            time_str = time_str.substr(0,index-1)+".000";
            schedule_id_execute_time = time_str;
        };

        Disconnect();
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << "EXIT_FAILURE: " << EXIT_FAILURE << std::endl;
    }
}

std::string yf::sql::sql_server::get_execute_time()
{
    return schedule_id_execute_time;
}

void yf::sql::sql_server::UpdateDeviceConnectionStatus(const std::string &device_name, const int &connection_status)
{
    std::string connection_status_str = std::to_string(connection_status);
    std::string query_update;

    try
    {
        Connect();

        query_update = "UPDATE sys_status_device SET connection_status = " + connection_status_str + ", modified_date='" + TimeNow() + "' WHERE device = '"+device_name+"'" ;

        nanodbc::execute(conn_,query_update);

        Disconnect();
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << "EXIT_FAILURE: " << EXIT_FAILURE << std::endl;
    }
}

void yf::sql::sql_server::UpdateDeviceMissionStatus(const std::string &device_name, const int &mission_status)
{
    std::string mission_status_str = std::to_string(mission_status);
    std::string query_update;

    try
    {
        Connect();

        query_update = "UPDATE sys_status_device SET  mission_status = " + mission_status_str + ", modified_date='" + TimeNow() + "' WHERE device = '" + device_name +"'" ;

        nanodbc::execute(conn_,query_update);

        Disconnect();
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << "EXIT_FAILURE: " << EXIT_FAILURE << std::endl;
    }
}

void yf::sql::sql_server::UpdateDeviceBatteryCapacity(const std::string &device_name, const float& battery_capacity)
{
    std::string battery_capacity_str = std::to_string(battery_capacity);
    std::string query_update;

    try
    {
        Connect();

        query_update = "UPDATE sys_status_device SET battery_capacity = " + battery_capacity_str + ", modified_date='" + TimeNow() + "' WHERE device = '"+device_name+"'" ;

        nanodbc::execute(conn_,query_update);

        Disconnect();
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << "EXIT_FAILURE: " << EXIT_FAILURE << std::endl;
    }
}

std::vector<float> yf::sql::sql_server::Time_str2vector(std::string str)
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

bool yf::sql::sql_server::isFutureTime(std::string time_from_database, std::string time_now)
{
    std::vector<float> time_db;
    std::vector<float> time_current;

    time_db = Time_str2vector(time_from_database);
    time_current = Time_str2vector(time_now);

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

void yf::sql::sql_server::WaitForExecute(const std::string &execute_time)
{
    // wait for execute time...
    //
    auto time_now = TimeNow();
    auto time_temp = time_now;
    auto time_scheduled = execute_time;

    // wait for execute time..
    while (isFutureTime(time_scheduled, time_temp))
    {
        time_temp = TimeNow();
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    }
}

int yf::sql::sql_server::GetTaskMode(const int &model_config_id)
{
    std::string query_update;

    //"SELECT ID FROM schedule_table where status=1 AND planned_start > '2021-02-06 11:10:08.000'"
    try
    {
        Connect();

        query_update = "SELECT task_mode FROM data_model_config where ID = "+std::to_string(model_config_id);

        int task_mode;

        auto result = nanodbc::execute(conn_,query_update);

        while(result.next())
        {
            task_mode = result.get<int>(0);
        };

        Disconnect();

        return task_mode;
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << "EXIT_FAILURE: " << EXIT_FAILURE << std::endl;
        return 0;
    }
}

int yf::sql::sql_server::GetTaskCommand(const int &cur_task_id)
{
    std::string query_update;

    //"SELECT ID FROM schedule_table where status=1 AND planned_start > '2021-02-06 11:10:08.000'"
    try
    {
        Connect();

        query_update = "SELECT command FROM sys_schedule_job_task where ID = "+std::to_string(cur_task_id)+" AND status=2 ";

        std::string task_command;

        auto result = nanodbc::execute(conn_,query_update);

        while(result.next())
        {
            task_command = result.get<std::string>(0, "null");
        };

        Disconnect();

        return std::stoi(task_command);
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << "EXIT_FAILURE: " << EXIT_FAILURE << std::endl;
    }
}

void yf::sql::sql_server::UpdateTaskLog(const int &cur_task_id, const int &task_status)
{
    std::string status_str = std::to_string(task_status);
    std::string task_id_str = std::to_string(cur_task_id);
    std::string query_update;

    try
    {
        Connect();

        switch (task_status) {
            case 2: // in processing
            {
                query_update = "INSERT INTO sys_schedule_job_task_log(task_id, actual_start, status) VALUES (" +
                               task_id_str + ",'" + TimeNow() + "'," + status_str + ")";
                break;
            }

            case 3: // finish
            {
                query_update =
                        "UPDATE sys_schedule_job_task_log SET status = " + status_str + ", actual_end='" + TimeNow() +
                        "' WHERE task_id = " + task_id_str + " AND status=2";
                break;
            }

            case 4: // cancel
            {
                query_update =
                        "UPDATE sys_schedule_job_task_log SET status = " + status_str + ", actual_end='" + TimeNow() +
                        "' WHERE task_id = " + task_id_str + " AND status=2";
                break;
            }

                // todo:
                // case 5: // pause
        }

        nanodbc::execute(conn_,query_update);

        Disconnect();
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << "EXIT_FAILURE: " << EXIT_FAILURE << std::endl;
    }
}

std::string yf::sql::sql_server::CountdownTime(const std::string& time_now, const int &countdown_min)
{
    auto v_time = Time_str2vector(time_now);

    int countdown_sec = countdown_min * 60;

    return this->get_future_db_time(countdown_sec);
}

void yf::sql::sql_server::UpdateTaskData(const int &cur_task_id, const int &task_status)
{
    std::string status_str = std::to_string(task_status);
    std::string id_str = std::to_string(cur_task_id);
    std::string query_update;

    try
    {
        Connect();

        switch (task_status)
        {
            case 2: // in processing
            {
                query_update = "UPDATE sys_schedule_job_task SET status = " + status_str + ", actual_start='"+TimeNow()+"' WHERE ID = "+ id_str;
                break;
            }

            case 3: // finish
            {
                query_update = "UPDATE sys_schedule_job_task SET status = " + status_str + ", actual_end='"+TimeNow()+"' WHERE ID = "+ id_str;
                break;
            }

            case 4: // cancel
            {
                query_update = "UPDATE sys_schedule_job_task SET status = " + status_str + ", actual_end='"+TimeNow()+"' WHERE ID = "+ id_str;
                break;
            }

            case 5: // error
            {
                query_update = "UPDATE sys_schedule_job_task SET status = " + status_str + ", actual_end='"+TimeNow()+"' WHERE ID = "+ id_str;
                break;
            }

            case 6: // pause
            {
                query_update = "UPDATE sys_schedule_job_task SET status = " + status_str + ", actual_end='"+TimeNow()+"' WHERE ID = "+ id_str;
                break;
            }

        }

        nanodbc::execute(conn_,query_update);

        Disconnect();
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << "EXIT_FAILURE: " << EXIT_FAILURE << std::endl;
    }
}

void yf::sql::sql_server::UpdateDeviceMissionStatusLog(const std::string &device_name, const int &mission_status)
{
    // (1) get current device mission status : std::string original_mission_status
    std::string query_update_1;

    std::string original_mission_status;

    //"SELECT ID FROM schedule_table where status=1 AND planned_start > '2021-02-06 11:10:08.000'"
    try
    {
        Connect();

        query_update_1 = "SELECT mission_status FROM sys_status_device where device = '"+ device_name +"'";



        auto result = nanodbc::execute(conn_,query_update_1);

        while(result.next())
        {
            original_mission_status = result.get<std::string>(0, "null");
        };

        Disconnect();

    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << "EXIT_FAILURE: " << EXIT_FAILURE << std::endl;
    }

    // (2) update to status log

    std::string new_mission_status = std::to_string(mission_status);
    std::string query_update_2;

    try
    {
        Connect();

        query_update_2 = "INSERT INTO sys_status_device_log(device_name, original_mission_status, new_mission_status, "
                         "created_time) VALUES ('" + device_name + "'," + original_mission_status +"," + new_mission_status +",'"+ TimeNow() + "'";

        nanodbc::execute(conn_,query_update_2);

        Disconnect();
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << "EXIT_FAILURE: " << EXIT_FAILURE << std::endl;
    }
}

void yf::sql::sql_server::UpdateJobData(const int &cur_job_id, const int &job_status)
{
    std::string status_str = std::to_string(job_status);
    std::string id_str = std::to_string(cur_job_id);
    std::string query_update;

    try
    {
        Connect();

        switch (job_status)
        {
            case 2: // in processing
            {
                query_update = "UPDATE sys_schedule_job SET status = " + status_str + ", actual_start='"+TimeNow()+"' WHERE job_id = "+ id_str;
                break;
            }

            case 3: // finish
            {
                query_update = "UPDATE sys_schedule_job SET status = " + status_str + ", actual_end='"+TimeNow()+"' WHERE job_id = "+ id_str;
                break;
            }

            case 4: // cancel
            {
                query_update = "UPDATE sys_schedule_job SET status = " + status_str + ", actual_end='"+TimeNow()+"' WHERE job_id = "+ id_str;
                break;
            }

            case 5: // error
            {
                query_update = "UPDATE sys_schedule_job SET status = " + status_str + ", actual_end='"+TimeNow()+"' WHERE job_id = "+ id_str;
                break;
            }

            case 6: // pause
            {
                query_update = "UPDATE sys_schedule_job SET status = " + status_str + ", actual_end='"+TimeNow()+"' WHERE job_id = "+ id_str;
                break;
            }

        }

        nanodbc::execute(conn_,query_update);

        Disconnect();
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << "EXIT_FAILURE: " << EXIT_FAILURE << std::endl;
    }
}

void yf::sql::sql_server::UpdateJobLog(const int &cur_job_id, const int &job_status)
{
    std::string status_str = std::to_string(job_status);
    std::string job_id_str = std::to_string(cur_job_id);
    std::string query_update;

    try
    {
        Connect();

        switch (job_status) {
            case 2: // in processing
            {
                query_update = "INSERT INTO sys_schedule_job_log(job_id, actual_start, status) VALUES (" +
                               job_id_str + ",'" + TimeNow() + "'," + status_str + ")";
                break;
            }

            case 3: // finish
            {
                query_update =
                        "UPDATE sys_schedule_job_log SET status = " + status_str + ", actual_end='" + TimeNow() +
                        "' WHERE job_id = " + job_id_str + " AND status=2";
                break;
            }

            case 4: // cancel
            {
                query_update =
                        "UPDATE sys_schedule_job_log SET status = " + status_str + ", actual_end='" + TimeNow() +
                        "' WHERE job_id = " + job_id_str + " AND status=2";
                break;
            }

            case 5: // error
            {
                query_update = "UPDATE sys_schedule_job_log SET status = " + status_str + ", actual_end='" + TimeNow() +
                               "' WHERE job_id = " + job_id_str ;
                break;
            }

            case 6: // pause
            {
                query_update = query_update = "UPDATE sys_schedule_job_log SET status = " + status_str + ", actual_end='" + TimeNow() +
                                              "' WHERE job_id = " + job_id_str ;
                break;
            }
        }

        nanodbc::execute(conn_,query_update);

        Disconnect();
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << "EXIT_FAILURE: " << EXIT_FAILURE << std::endl;
    }
}

int yf::sql::sql_server::GetSysControlMode()
{
    std::string query_update;

    //"SELECT ID FROM schedule_table where status=1 AND planned_start > '2021-02-06 11:10:08.000'"
    try
    {
        Connect();

        query_update = "SELECT control_mode FROM sys_status where ID = 1 AND name = 'nw_sys' ";

        std::string sys_control_mode;

        auto result = nanodbc::execute(conn_,query_update);

        while(result.next())
        {
            sys_control_mode = result.get<std::string>(0, "null");
        };

        Disconnect();

        return std::stoi(sys_control_mode);
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << "EXIT_FAILURE: " << EXIT_FAILURE << std::endl;

        return -999;
    }
}

std::deque<int> yf::sql::sql_server::GetAvailableScheuldesId()
{
    std::string query_update;

    std::deque<int> available_schedules;

    //"SELECT ID FROM schedule_table where status=1 AND planned_start > '2021-02-06 11:10:08.000'"
    try
    {
        Connect();

        query_update = "SELECT ID FROM sys_schedule where status=1 AND planned_start > '"+ TimeNow() +"'";

        auto result = nanodbc::execute(conn_,query_update);

        // if there are new schedules available, sql module will mark down all the available schedule ids
        while(result.next())
        {
            std::string id_str = result.get<std::string>(0, "null");

            int id = std::stoi(id_str);

            available_schedules.push_back(id);
        };

        Disconnect();

        return available_schedules;
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << "EXIT_FAILURE: " << EXIT_FAILURE << std::endl;

        return available_schedules;
    }
}

int yf::sql::sql_server::GetArmMotionType(const int& arm_mission_config_id)
{
    std::string mission_id_str = std::to_string(arm_mission_config_id);

    //motion types
    std::string query_motion_type;

    int motion_type;

    try
    {
        Connect();

        query_motion_type = "SELECT motion_type FROM data_arm_mission_config where ID = " + mission_id_str;

        auto result = nanodbc::execute(conn_, query_motion_type);

        while(result.next())
        {
            motion_type = result.get<int>(0);
        };

        Disconnect();

        return motion_type;

    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << "EXIT_FAILURE: " << EXIT_FAILURE << std::endl;

        return 0;
    }

}

int yf::sql::sql_server::GetArmMissionPointId(const int &arm_mission_config_id, const std::string &arm_point_name)
{
    std::string mission_id_str = std::to_string(arm_mission_config_id);

    //motion types
    std::string query;

    int point_id;

    try
    {
        Connect();

        query = "SELECT " + arm_point_name + " FROM data_arm_mission_config where ID = " + mission_id_str;

        auto result = nanodbc::execute(conn_, query);

        while(result.next())
        {
            point_id = result.get<int>(0);
        };

        Disconnect();

        return point_id;

    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << "EXIT_FAILURE: " << EXIT_FAILURE << std::endl;

        return 1;
    }

}

yf::data::arm::Point3d yf::sql::sql_server::GetArmPoint(const int &point_id)
{

    yf::data::arm::Point3d point3d;

    point3d.x = this->GetArmPointElement(point_id,"x");
    point3d.y = this->GetArmPointElement(point_id,"y");
    point3d.z = this->GetArmPointElement(point_id,"z");
    point3d.rx = this->GetArmPointElement(point_id,"rx");
    point3d.ry = this->GetArmPointElement(point_id,"ry");
    point3d.rz = this->GetArmPointElement(point_id,"rz");

    return point3d;
}

float yf::sql::sql_server::GetArmPointElement(const int &point_id, const std::string &point_element)
{
    std::string query_update;

    std::string point_id_str = std::to_string(point_id);

    float element_value;

    //"SELECT ID FROM schedule_table where status=1 AND planned_start > '2021-02-06 11:10:08.000'"
    try
    {
        Connect();

        query_update = "SELECT " + point_element + " FROM data_arm_points where ID = " + point_id_str;

        auto result = nanodbc::execute(conn_,query_update);

        // if there are new schedules available, sql module will mark down all the available schedule ids
        while(result.next())
        {
            element_value = result.get<float>(point_element);
        };

        Disconnect();

        return element_value;
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << "EXIT_FAILURE: " << EXIT_FAILURE << std::endl;

        return 0;
    };
}

int yf::sql::sql_server::GetModelConfigId(const int &cur_job_id)
{
    std::string query_update;

    std::string cur_job_id_str = std::to_string(cur_job_id);

    int model_config_id;

    //"SELECT ID FROM schedule_table where status=1 AND planned_start > '2021-02-06 11:10:08.000'"
    try
    {
        Connect();

        query_update = "SELECT model_config_id FROM sys_schedule_job where job_id = " + cur_job_id_str;

        auto result = nanodbc::execute(conn_,query_update);

        // if there are new schedules available, sql module will mark down all the available schedule ids
        while(result.next())
        {
            model_config_id = result.get<int>(0);
        };

        Disconnect();

        return model_config_id;
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << "EXIT_FAILURE: " << EXIT_FAILURE << std::endl;

        return 0;
    };
}

int yf::sql::sql_server::GetModelConfigElement(const int &model_config_id, const std::string &element)
{
    std::string query_update;

    std::string model_config_id_str = std::to_string(model_config_id);

    int element_value;

    //"SELECT ID FROM schedule_table where status=1 AND planned_start > '2021-02-06 11:10:08.000'"
    try
    {
        Connect();

        query_update = "SELECT " + element + " FROM data_model_config where ID = " + model_config_id_str;

        auto result = nanodbc::execute(conn_,query_update);

        // if there are new schedules available, sql module will mark down all the available schedule ids
        while(result.next())
        {
            element_value = result.get<int>(0);
        };

        Disconnect();

        return element_value;
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << "EXIT_FAILURE: " << EXIT_FAILURE << std::endl;

        return 0;
    };
}

std::deque<int> yf::sql::sql_server::GetArmMissionConfigIds(const int &arm_config_id)
{
    // query string
    std::string query_update;

    // input
    std::string arm_config_id_str = std::to_string(arm_config_id);

    // output
    std::deque<int> arm_mission_config_ids;

    //"SELECT ID FROM schedule_table where status=1 AND planned_start > '2021-02-06 11:10:08.000'"
    try
    {
        Connect();

        query_update = "SELECT ID FROM data_arm_mission_config where arm_config_id = " + arm_config_id_str + " ORDER BY mission_order";

        auto result = nanodbc::execute(conn_,query_update);

        // if there are new schedules available, sql module will mark down all the available schedule ids
        while(result.next())
        {
            auto mission_config_id = result.get<int>(0);

            arm_mission_config_ids.push_back(mission_config_id);
        };

        Disconnect();

        return arm_mission_config_ids;
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << "EXIT_FAILURE: " << EXIT_FAILURE << std::endl;

        return arm_mission_config_ids;
    };
}

int yf::sql::sql_server::GetUgvMissionConfigNum(const int &model_config_id)
{
    // query string
    std::string query_update;

    // input
    std::string model_config_id_str = std::to_string(model_config_id);

    // output
    int mission_config_num;

    //"SELECT max(mission_order) AS mission_num FROM data_ugv_mission_config where model_config_id = 1"
    try
    {
        Connect();

        query_update = "SELECT max(mission_order) AS mission_config_num FROM data_ugv_mission_config where model_config_id = " + model_config_id_str ;

        auto result = nanodbc::execute(conn_,query_update);

        // if there are new schedules available, sql module will mark down all the available schedule ids
        while(result.next())
        {
            mission_config_num = result.get<int>(0);
        };

        Disconnect();

        return mission_config_num;
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << "EXIT_FAILURE: " << EXIT_FAILURE << std::endl;

        return 0;
    };
}

//@@ input: mission_order == plc_002_value
//
int yf::sql::sql_server::GetArmConfigId(const int &model_config_id, const int &cur_order)
{
    // query string
    std::string query_update;

    // input
    std::string model_config_id_str = std::to_string(model_config_id);
    std::string cur_order_str = std::to_string(cur_order);

    // output
    int arm_config_id;

    //"SELECT arm_config_id FROM data_ugv_mission_config where model_config_id = 1"
    try
    {
        Connect();

        query_update = "SELECT arm_config_id FROM data_ugv_mission_config where model_config_id = " + model_config_id_str + "AND mission_order = " + cur_order_str ;

        auto result = nanodbc::execute(conn_,query_update);

        // if there are new schedules available, sql module will mark down all the available schedule ids
        while(result.next())
        {
            arm_config_id = result.get<int>(0);
        };

        Disconnect();

        return arm_config_id;
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << "EXIT_FAILURE: " << EXIT_FAILURE << std::endl;

        return 0;
    };
}

std::deque<std::string> yf::sql::sql_server::GetUgvMissionConfigPositionNames(const int &model_config_id)
{
    // query string
    std::string query_update;

    // input
    std::string model_config_id_str = std::to_string(model_config_id);

    // output
    std::deque<std::string> position_names;

    //"SELECT position_name FROM data_ugv_mission_config where model_config_id = 1 ORDER BY mission_order"
    try
    {
        Connect();

        query_update = "SELECT position_name FROM data_ugv_mission_config where model_config_id = " + model_config_id_str + " ORDER BY mission_order" ;

        auto result = nanodbc::execute(conn_,query_update);

        // if there are new schedules available, sql module will mark down all the available schedule ids
        while(result.next())
        {
            auto position_name = result.get<std::string>(0);

            position_names.push_back(position_name);
        };

        Disconnect();

        return position_names;
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << "EXIT_FAILURE: " << EXIT_FAILURE << std::endl;

        return position_names;
    };
}

int yf::sql::sql_server::GetModelId(const int &model_config_id)
{
    // query string
    std::string query_update;

    // input
    std::string model_config_id_str = std::to_string(model_config_id);

    // output
    int model_id;

    //"SELECT position_name FROM data_ugv_mission_config where model_config_id = 1 ORDER BY mission_order"
    try
    {
        Connect();

        query_update = "SELECT model_id FROM data_model_config where ID = " + model_config_id_str ;

        auto result = nanodbc::execute(conn_,query_update);

        // if there are new schedules available, sql module will mark down all the available schedule ids
        while(result.next())
        {
            model_id = result.get<int>(0);
        };

        Disconnect();

        return model_id;
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << "EXIT_FAILURE: " << EXIT_FAILURE << std::endl;

        return model_id;
    };
}

std::string yf::sql::sql_server::GetModelName(const int &model_config_id)
{
    auto model_id = this->GetModelId(model_config_id);

    // query string
    std::string query_update;

    // input
    std::string model_id_str = std::to_string(model_id);

    // output
    std::string model_name;

    //"SELECT position_name FROM data_ugv_mission_config where model_config_id = 1 ORDER BY mission_order"
    try
    {
        Connect();

        query_update = "SELECT name FROM data_model where ID = " + model_id_str ;

        auto result = nanodbc::execute(conn_,query_update);

        // if there are new schedules available, sql module will mark down all the available schedule ids
        while(result.next())
        {
            model_name = result.get<std::string>(0);
        };

        Disconnect();

        return model_name;
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << "EXIT_FAILURE: " << EXIT_FAILURE << std::endl;

        return model_name;
    };
}

int yf::sql::sql_server::GetMapIdFromModelId(const int &model_id)
{
    // query string
    std::string query_update;

    // input
    std::string model_id_str = std::to_string(model_id);

    // output
    int map_id;

    //"SELECT position_name FROM data_ugv_mission_config where model_config_id = 1 ORDER BY mission_order"
    try
    {
        Connect();

        query_update = "SELECT map_id FROM data_model where ID = " + model_id_str ;

        auto result = nanodbc::execute(conn_,query_update);

        // if there are new schedules available, sql module will mark down all the available schedule ids
        while(result.next())
        {
            map_id = result.get<int>(0);
        };

        Disconnect();

        return map_id;
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << "EXIT_FAILURE: " << EXIT_FAILURE << std::endl;

        return map_id;
    };
}

std::string yf::sql::sql_server::GetMapElement(const int &map_id, const std::string &map_element)
{
    // query string
    std::string query_update;

    // input
    std::string map_id_str = std::to_string(map_id);

    // output
    std::string element_value_str;

    //"SELECT position_name FROM data_ugv_mission_config where model_config_id = 1 ORDER BY mission_order"
    try
    {
        Connect();

        query_update = "SELECT " + map_element + " FROM data_ugv_map where ID = " + map_id_str ;

        auto result = nanodbc::execute(conn_,query_update);

        // if there are new schedules available, sql module will mark down all the available schedule ids
        while(result.next())
        {
            element_value_str = result.get<std::string>(0);
        };

        Disconnect();

        return element_value_str;
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << "EXIT_FAILURE: " << EXIT_FAILURE << std::endl;

        return element_value_str;
    };
}

std::string yf::sql::sql_server::GetSiteInfo(const int &model_config_id)
{
    /// model_config_id -> model_id -> map_id -> map_guid / session_id / building_id / floor_id
    //
    // 1. get model_id
    auto model_id = this->GetModelId(model_config_id);

    // 2. get map_id
    auto map_id = this->GetMapIdFromModelId(model_id);

    // 3. get location_site_id
    auto location_site_id_str = this->GetMapElement(map_id, "location_site_id");

    // 4. based on location_site_id, get site name.

    // query string
    std::string query_update;

    // input


    // output
    std::string site_str;

    //"SELECT position_name FROM data_ugv_mission_config where model_config_id = 1 ORDER BY mission_order"
    try
    {
        Connect();

        query_update = "SELECT location_site FROM data_Location_Site where ID = " + location_site_id_str ;

        auto result = nanodbc::execute(conn_,query_update);

        // if there are new schedules available, sql module will mark down all the available schedule ids
        while(result.next())
        {
            site_str = result.get<std::string>(0);
        };

        Disconnect();

        return site_str;
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << "EXIT_FAILURE: " << EXIT_FAILURE << std::endl;

        return site_str;
    };
}

std::string yf::sql::sql_server::GetBuildingInfo(const int &model_config_id)
{
    /// model_config_id -> model_id -> map_id -> map_guid / session_id / building_id / floor_id
    //
    // 1. get model_id
    auto model_id = this->GetModelId(model_config_id);

    // 2. get map_id
    auto map_id = this->GetMapIdFromModelId(model_id);

    // 3. get location_site_id
    auto location_building_id_str = this->GetMapElement(map_id, "location_building_id");

    // 4. based on location_site_id, get site name.

    // query string
    std::string query_update;

    // input


    // output
    std::string building_str;

    //"SELECT position_name FROM data_ugv_mission_config where model_config_id = 1 ORDER BY mission_order"
    try
    {
        Connect();

        query_update = "SELECT building_name FROM data_Location_Site_Building where ID = " + location_building_id_str ;

        auto result = nanodbc::execute(conn_,query_update);

        // if there are new schedules available, sql module will mark down all the available schedule ids
        while(result.next())
        {
            building_str = result.get<std::string>(0);
        };

        Disconnect();

        return building_str;
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << "EXIT_FAILURE: " << EXIT_FAILURE << std::endl;

        return building_str;
    };
}

std::string yf::sql::sql_server::GetFloorInfo(const int &model_config_id)
{
    /// model_config_id -> model_id -> map_id -> map_guid / session_id / building_id / floor_id
    //
    // 1. get model_id
    auto model_id = this->GetModelId(model_config_id);

    // 2. get map_id
    auto map_id = this->GetMapIdFromModelId(model_id);

    // 3. get location_site_id
    auto location_floor_id_str = this->GetMapElement(map_id, "location_floor_id");

    // 4. based on location_floor_id, get floor info.

    // query string
    std::string query_update;

    // input


    // output
    std::string floor_str;

    //"SELECT position_name FROM data_ugv_mission_config where model_config_id = 1 ORDER BY mission_order"
    try
    {
        Connect();

        query_update = "SELECT floor_no FROM data_Location_Site_Building_Floor where ID = " + location_floor_id_str ;

        auto result = nanodbc::execute(conn_,query_update);

        // if there are new schedules available, sql module will mark down all the available schedule ids
        while(result.next())
        {
            floor_str = result.get<std::string>(0);
        };

        Disconnect();

        return floor_str;
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << "EXIT_FAILURE: " << EXIT_FAILURE << std::endl;

        return floor_str;
    };
}

int yf::sql::sql_server::GetOperationArea(const int &arm_config_id)
{
    std::string query_update;

    //"SELECT ID FROM schedule_table where status=1 AND planned_start > '2021-02-06 11:10:08.000'"
    try
    {
        Connect();

        query_update = "SELECT operation_area FROM data_arm_config where ID = "+ std::to_string(arm_config_id);

        int operation_area;

        auto result = nanodbc::execute(conn_,query_update);

        while(result.next())
        {
            operation_area = result.get<int>(0);
        };

        Disconnect();


        return operation_area;
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << "EXIT_FAILURE: " << EXIT_FAILURE << std::endl;
        return 0;
    }
}

int yf::sql::sql_server::GetToolAngle(const int &arm_mission_config_id)
{
    std::string query_update;

    //"SELECT ID FROM schedule_table where status=1 AND planned_start > '2021-02-06 11:10:08.000'"
    try
    {
        Connect();

        query_update = "SELECT tool_angle FROM data_arm_mission_config where ID = "+ std::to_string(arm_mission_config_id);

        int tool_angle;

        auto result = nanodbc::execute(conn_,query_update);

        while(result.next())
        {
            tool_angle = result.get<int>(0);
        };

        Disconnect();


        return tool_angle;
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << "EXIT_FAILURE: " << EXIT_FAILURE << std::endl;
        return 0;
    }
}

int yf::sql::sql_server::GetMotionType(const int &arm_mission_config_id)
{
    std::string query_update;

    //"SELECT ID FROM schedule_table where status=1 AND planned_start > '2021-02-06 11:10:08.000'"
    try
    {
        Connect();

        query_update = "SELECT motion_type FROM data_arm_mission_config where ID = "+ std::to_string(arm_mission_config_id);

        int motion_type;

        auto result = nanodbc::execute(conn_,query_update);

        while(result.next())
        {
            motion_type = result.get<int>(0);
        };

        Disconnect();

        return motion_type;
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << "EXIT_FAILURE: " << EXIT_FAILURE << std::endl;
        return 0;
    }
}

int yf::sql::sql_server::GetStandbyPositionId(const int &arm_config_id)
{
    std::string query_update;

    //"SELECT ID FROM schedule_table where status=1 AND planned_start > '2021-02-06 11:10:08.000'"
    try
    {
        Connect();

        query_update = "SELECT standby_position_id FROM data_arm_config where ID = "+ std::to_string(arm_config_id);

        int standby_position_id;

        auto result = nanodbc::execute(conn_,query_update);

        while(result.next())
        {
            standby_position_id = result.get<int>(0);
        };

        Disconnect();

        return standby_position_id;
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << "EXIT_FAILURE: " << EXIT_FAILURE << std::endl;
        return 0;
    }
}

int yf::sql::sql_server::GetLandmarkFlag(const int &arm_mission_config_id)
{
    std::string query_update;

    //"SELECT ID FROM schedule_table where status=1 AND planned_start > '2021-02-06 11:10:08.000'"
    try
    {
        Connect();

        query_update = "SELECT landmark_flag FROM data_arm_mission_config where ID = "+ std::to_string(arm_mission_config_id);

        int landmark_flag;

        auto result = nanodbc::execute(conn_,query_update);

        while(result.next())
        {
            landmark_flag = result.get<int>(0);
        };

        Disconnect();

        return landmark_flag;
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << "EXIT_FAILURE: " << EXIT_FAILURE << std::endl;
        return 0;
    }
}

void yf::sql::sql_server::OSGetScheduleReady(const int &schedule_id, const int &seconds_later_from_now)
{
    std::string query;

    // 1. get schedule job ready
    auto job_ids = this->GetJobsId(schedule_id);

    for(int n = 0; n < job_ids.size() ; n++)
    {
        std::string job_id = std::to_string(job_ids[n]);

        try
        {
            Connect();

            query = "UPDATE sys_schedule_job SET status = 1 WHERE job_id = " + job_id ;

            nanodbc::execute(conn_, query);

            Disconnect();
        }
        catch (std::exception& e)
        {
            std::cerr << e.what() << std::endl;
            std::cerr << "EXIT_FAILURE: " << EXIT_FAILURE << std::endl;
        }
    }

    // 2. get schedule ready

    auto future_date = this->get_future_db_time(seconds_later_from_now);

    try
    {
        Connect();

        query = "UPDATE sys_schedule SET status = 1, planned_start='" + future_date + "' WHERE ID = " + std::to_string(schedule_id) ;

        nanodbc::execute(conn_, query);

        Disconnect();
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << "EXIT_FAILURE: " << EXIT_FAILURE << std::endl;
    }
}

auto yf::sql::sql_server::convert_to_timepoint(int years, int months, int days, int hours, int mins, int secs)
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

template<typename Clock, typename Duration>
auto yf::sql::sql_server::add_seconds(const std::chrono::time_point<Clock, Duration> &timepoint, int seconds_to_add)
{
    std::time_t seconds =  seconds_to_add;
    auto date = Clock::to_time_t(timepoint);
    return Clock::from_time_t(date + seconds);
}

template<typename Clock, typename Duration>
std::string yf::sql::sql_server::convert_to_db_time(const std::chrono::time_point<Clock, Duration> &timep)
{
    auto converted_timep = Clock::to_time_t(timep);

    std::tm date = *std::localtime(&converted_timep);

    std::string db_time =   std::to_string(date.tm_year + 1900) + "-" +
                            std::to_string(date.tm_mon + 1) + "-" +
                            std::to_string(date.tm_mday) + " " +
                            std::to_string(date.tm_hour) + ":" +
                            std::to_string(date.tm_min) + ":" +
                            std::to_string(date.tm_sec) + ".000";

    return db_time;
}

std::string yf::sql::sql_server::get_future_db_time(const int &seconds_later_from_now)
{
    this->UpdateTime();
    auto year = this->get_time_element("year");
    auto month = this->get_time_element("month");
    auto day = this->get_time_element("day");
    auto hour = this->get_time_element("hour");
    auto min = this->get_time_element("minute");
    auto sec = this->get_time_element("sec");

    auto date = convert_to_timepoint(std::stoi(year), std::stoi(month)-1, std::stoi(day),std::stoi(hour),std::stoi(min),std::stoi(sec));

    auto date_future = add_seconds(date, seconds_later_from_now);

    return convert_to_db_time(date_future);
}

int yf::sql::sql_server::GetModelType(const int &model_config_id)
{
    auto model_id = this->GetModelId(model_config_id);

    // query string
    std::string query_update;

    // input
    std::string model_id_str = std::to_string(model_id);

    // output
    int model_type;

    //"SELECT position_name FROM data_ugv_mission_config where model_config_id = 1 ORDER BY mission_order"
    try
    {
        Connect();

        query_update = "SELECT model_type FROM data_model where ID = " + model_id_str ;

        auto result = nanodbc::execute(conn_,query_update);

        // if there are new schedules available, sql module will mark down all the available schedule ids
        while(result.next())
        {
            model_type = result.get<int>(0);
        };

        Disconnect();

        return model_type;
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << "EXIT_FAILURE: " << EXIT_FAILURE << std::endl;

        return 0;
    };
}

int yf::sql::sql_server::GetArmRefLMPosId(const int &arm_mission_config_id)
{
    std::string mission_id_str = std::to_string(arm_mission_config_id);

    //ref_landmark_pos_id
    //
    std::string query;

    int point_id;

    try
    {
        Connect();

        query = "SELECT ID FROM data_arm_mc_ref_landmark_pos where arm_mission_config_id = " + mission_id_str;

        auto result = nanodbc::execute(conn_, query);

        while(result.next())
        {
            point_id = result.get<int>(0);
        };

        Disconnect();

        return point_id;

    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << "EXIT_FAILURE: " << EXIT_FAILURE << std::endl;

        return 1;
    }
}

float yf::sql::sql_server::GetArmRefLMPosElement(const int &pos_id, const std::string &pos_element)
{
    std::string query_update;

    std::string pos_id_str = std::to_string(pos_id);

    float element_value;

    //"SELECT ID FROM schedule_table where status=1 AND planned_start > '2021-02-06 11:10:08.000'"
    try
    {
        Connect();

        query_update = "SELECT " + pos_element + " FROM data_arm_mc_ref_landmark_pos where ID = " + pos_id_str;

        auto result = nanodbc::execute(conn_,query_update);

        // if there are new schedules available, sql module will mark down all the available schedule ids
        while(result.next())
        {
            element_value = result.get<float>(pos_element);
        };

        Disconnect();

        return element_value;
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << "EXIT_FAILURE: " << EXIT_FAILURE << std::endl;

        return 0;
    };
}

yf::data::arm::Point3d yf::sql::sql_server::GetArmRefLMPos(const int &pos_id)
{
    yf::data::arm::Point3d point3d;

    point3d.x = this->GetArmRefLMPosElement(pos_id, "x");
    point3d.y = this->GetArmRefLMPosElement(pos_id, "y");
    point3d.z = this->GetArmRefLMPosElement(pos_id, "z");
    point3d.rx = this->GetArmRefLMPosElement(pos_id, "rx");
    point3d.ry = this->GetArmRefLMPosElement(pos_id, "ry");
    point3d.rz = this->GetArmRefLMPosElement(pos_id, "rz");

    return point3d;

    return yf::data::arm::Point3d();
}

std::deque<yf::data::arm::Point3d>
yf::sql::sql_server::GetRefPathInitPoints(const int &arm_mission_config_id)
{
    std::deque<yf::data::arm::Point3d> clean_points;

    auto ref_path_init_point_ids =  this->GetRefPathInitPointIds(arm_mission_config_id);

    for (int n = 0; n < ref_path_init_point_ids.size(); n++)
    {
        clean_points.push_back(this->GetRefPathInitPoint(ref_path_init_point_ids[n]));
    }

    return clean_points;
}

std::deque<int> yf::sql::sql_server::GetRefPathInitPointIds(const int &arm_mission_config_id)
{
    std::deque<int> ref_path_init_point_ids;

    std::string query_update;

    std::string arm_mission_config_id_str = std::to_string(arm_mission_config_id);

    //"SELECT ID FROM schedule_table where status=1 AND planned_start > '2021-02-06 11:10:08.000'"
    try
    {
        Connect();

        query_update = "SELECT ID FROM data_arm_mc_ref_path_init_points where arm_mission_config_id = " + arm_mission_config_id_str + " ORDER BY point_order" ;

        auto result = nanodbc::execute(conn_,query_update);

        // if there are new schedules available, sql module will mark down all the available schedule ids
        while(result.next())
        {
            std::string id_str = result.get<std::string>(0, "null");

            int id = std::stoi(id_str);

            ref_path_init_point_ids.push_back(id);
        };

        Disconnect();

        return ref_path_init_point_ids;
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << "EXIT_FAILURE: " << EXIT_FAILURE << std::endl;
    }
}

float yf::sql::sql_server::GetRefPathInitPointElement(const int &point_id, const std::string &point_element)
{
    std::string query_update;

    std::string pos_id_str = std::to_string(point_id);

    float element_value;

    //"SELECT ID FROM schedule_table where status=1 AND planned_start > '2021-02-06 11:10:08.000'"
    try
    {
        Connect();

        query_update = "SELECT " + point_element + " FROM data_arm_mc_ref_path_init_points where ID = " + pos_id_str;

        auto result = nanodbc::execute(conn_,query_update);

        // if there are new schedules available, sql module will mark down all the available schedule ids
        while(result.next())
        {
            element_value = result.get<float>(point_element);
        };

        Disconnect();

        return element_value;
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << "EXIT_FAILURE: " << EXIT_FAILURE << std::endl;

        return 0;
    };
}

yf::data::arm::Point3d yf::sql::sql_server::GetRefPathInitPoint(const int &point_id)
{
    yf::data::arm::Point3d point3d;

    point3d.x = this->GetRefPathInitPointElement(point_id,"x");
    point3d.y = this->GetRefPathInitPointElement(point_id,"y");
    point3d.z = this->GetRefPathInitPointElement(point_id,"z");
    point3d.rx = this->GetRefPathInitPointElement(point_id,"rx");
    point3d.ry = this->GetRefPathInitPointElement(point_id,"ry");
    point3d.rz = this->GetRefPathInitPointElement(point_id,"rz");

    return point3d;
}

void yf::sql::sql_server::UpdateDeviceUgvCurPosition(const float &x, const float &y, const float &theta)
{
    std::string x_str = std::to_string(x);
    std::string y_str = std::to_string(y);
    std::string theta_str = std::to_string(theta);

    std::string query_update;

    try
    {
        /// x
        Connect();

        query_update = "UPDATE sys_status_device SET ugv_pos_x = " + x_str + ", modified_date='" + TimeNow() + "' WHERE device = 'ugv'" ;

        nanodbc::execute(conn_,query_update);

        Disconnect();

        /// y
        Connect();

        query_update = "UPDATE sys_status_device SET ugv_pos_y = " + y_str + " WHERE device = 'ugv'" ;

        nanodbc::execute(conn_,query_update);

        Disconnect();

        /// z
        Connect();

        query_update = "UPDATE sys_status_device SET ugv_pos_theta = " + theta_str + " WHERE device = 'ugv'" ;

        nanodbc::execute(conn_,query_update);

        Disconnect();
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << "EXIT_FAILURE: " << EXIT_FAILURE << std::endl;
    }
}

void yf::sql::sql_server::InsertRefLandmarkPos(const int &arm_mission_config_id, const yf::data::arm::Point3d &pos)
{
    std::string query_update;

    //"SELECT ID FROM schedule_table where status=1 AND planned_start > '2021-02-06 11:10:08.000'"
    try
    {
        Connect();

        query_update = "INSERT INTO data_arm_mc_ref_landmark_pos(arm_mission_config_id, x,y,z,rx,ry,rz) "
                       "VALUES (" + std::to_string(arm_mission_config_id)  + "," +
                                    std::to_string(pos.x)+ "," +
                                    std::to_string(pos.y)+ "," +
                                    std::to_string(pos.z)+ "," +
                                    std::to_string(pos.rx)+ "," +
                                    std::to_string(pos.ry)+ "," +
                                    std::to_string(pos.rz) +")";

        auto result = nanodbc::execute(conn_,query_update);

        Disconnect();

        return ;
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << "EXIT_FAILURE: " << EXIT_FAILURE << std::endl;
    }
}

int yf::sql::sql_server::GetRefPathLayerNo(const int &arm_mission_config_id)
{
    std::string query_update;

    //"SELECT ID FROM schedule_table where status=1 AND planned_start > '2021-02-06 11:10:08.000'"
    try
    {
        Connect();

        query_update = "SELECT ref_path_layer_no FROM data_arm_mission_config where ID = "+ std::to_string(arm_mission_config_id);

        int landmark_flag;

        auto result = nanodbc::execute(conn_,query_update);

        while(result.next())
        {
            landmark_flag = result.get<int>(0);
        };

        Disconnect();

        return landmark_flag;
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << "EXIT_FAILURE: " << EXIT_FAILURE << std::endl;
        return 0;
    }
}

float yf::sql::sql_server::GetRefPathStepRatioHorizontal(const int &arm_mission_config_id)
{
    std::string query_update;

    //"SELECT ID FROM schedule_table where status=1 AND planned_start > '2021-02-06 11:10:08.000'"
    try
    {
        Connect();

        query_update = "SELECT ref_path_step_ratio_horizontal FROM data_arm_mission_config where ID = "+ std::to_string(arm_mission_config_id);

        float landmark_flag;

        auto result = nanodbc::execute(conn_,query_update);

        while(result.next())
        {
            landmark_flag = result.get<float>(0);
        };

        Disconnect();

        return landmark_flag;
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << "EXIT_FAILURE: " << EXIT_FAILURE << std::endl;
        return 0;
    }
}

int yf::sql::sql_server::GetArmConfigIsValid(const int &arm_config_id)
{
    // query string
    std::string query_update;

    // input
    std::string arm_config_id_str = std::to_string(arm_config_id);

    // output
    int is_valid;

    //"SELECT arm_config_id FROM data_ugv_mission_config where model_config_id = 1"
    try
    {
        Connect();

        query_update = "SELECT is_valid FROM data_arm_config where ID = " + arm_config_id_str  ;

        auto result = nanodbc::execute(conn_,query_update);

        // if there are new schedules available, sql module will mark down all the available schedule ids
        while(result.next())
        {
            is_valid = result.get<int>(0);
        };

        Disconnect();

        return is_valid;
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << "EXIT_FAILURE: " << EXIT_FAILURE << std::endl;

        return 0;
    };
}

std::vector<int> yf::sql::sql_server::GetArmConfigValidResultQueue(const int &model_config_id)
{
    std::vector<int> valid_q;

    int total_mission_config_num = this->GetUgvMissionConfigNum(model_config_id);

    for (int n = 1; n < total_mission_config_num+1 ; n++)
    {
        int cur_arm_config_id = this->GetArmConfigId(model_config_id,n);

        int is_valid = this->GetArmConfigIsValid(cur_arm_config_id);

        valid_q.push_back(is_valid);
    }

    return valid_q;
}

int yf::sql::sql_server::GetFirstValidOrder(const int &model_config_id)
{
    std::vector<int> valid_q = this->GetArmConfigValidResultQueue(model_config_id);

    // find the first valid order!
    std::vector<int>::iterator first_valid_index = std::find(valid_q.begin(), valid_q.end(), 1);

    int first_valid_order = first_valid_index-valid_q.begin() + 1;

    return first_valid_order;
}

int yf::sql::sql_server::GetLastValidOrder(const int &model_config_id)
{
    std::vector<int> valid_q = this->GetArmConfigValidResultQueue(model_config_id);

    std::vector<int> is_valid = {1};

    // find the first valid order!
    std::vector<int>::iterator last_valid_index = std::find_end(valid_q.begin(), valid_q.end(), is_valid.begin(), is_valid.end());

    int last_valid_order = last_valid_index-valid_q.begin() + 1;

    return last_valid_order;
}

std::vector<int> yf::sql::sql_server::GetValidIndexes(const int& model_config_id)
{
    auto valid_result_queue = this->GetArmConfigValidResultQueue(model_config_id);

    /// valid_indexes
    std::vector<int> valid_indexes;

    std::vector<int>::iterator first_index = std::find(valid_result_queue.begin(), valid_result_queue.end(), 1);

    if(first_index == std::end(valid_result_queue))
    {
        std::cout << "there is no any valid index"<< std::endl;
    }
    else
    {
        //todo: for loop, find all valid index. push back to valid_indexes

        std::vector<int>::iterator iter = valid_result_queue.begin();

        while ((iter = std::find_if(iter, valid_result_queue.end(), IsOne)) != valid_result_queue.end())
        {
            // Do something with iter
            valid_indexes.push_back(iter-valid_result_queue.begin());

            iter++;
        }
    }
    return valid_indexes;
}

std::string yf::sql::sql_server::GetMapNameFromMapStatus()
{
    // query string
    std::string query_update;

    // input

    // output
    std::string map_name;

    //"SELECT position_name FROM data_ugv_mission_config where model_config_id = 1 ORDER BY mission_order"
    try
    {
        Connect();

        query_update = "SELECT map_name FROM data_ugv_map where status = 1 ";

        auto result = nanodbc::execute(conn_,query_update);

        // if there are new schedules available, sql module will mark down all the available schedule ids
        while(result.next())
        {
            map_name = result.get<std::string>(0);
        };

        Disconnect();

        return map_name;
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << "EXIT_FAILURE: " << EXIT_FAILURE << std::endl;

        return map_name;
    };
}

int yf::sql::sql_server::GetUgvInitPositionIdFromMapStatus()
{
    // query string
    std::string query_update;

    // input

    // output
    int init_position_id;

    //"SELECT position_name FROM data_ugv_mission_config where model_config_id = 1 ORDER BY mission_order"
    try
    {
        Connect();

        query_update = "SELECT init_position_id FROM data_ugv_map where status = 1 ";

        auto result = nanodbc::execute(conn_,query_update);

        // if there are new schedules available, sql module will mark down all the available schedule ids
        while(result.next())
        {
            init_position_id = result.get<int>(0);
        };

        Disconnect();

        return init_position_id;
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << "EXIT_FAILURE: " << EXIT_FAILURE << std::endl;

        return init_position_id;
    };
}

float yf::sql::sql_server::GetUgvInitPositionElement(const int &id, const std::string &element)
{
    // query string
    std::string query_update;

    // input
    std::string point_id_str = std::to_string(id);

    // output
    float element_value_str;

    //"SELECT position_name FROM data_ugv_mission_config where model_config_id = 1 ORDER BY mission_order"
    try
    {
        Connect();

        query_update = "SELECT " + element + " FROM data_ugv_map_InitPositionList where ID = " + point_id_str ;

        auto result = nanodbc::execute(conn_,query_update);

        // if there are new schedules available, sql module will mark down all the available schedule ids
        while(result.next())
        {
            element_value_str = result.get<float>(0);
        };

        Disconnect();

        return element_value_str;
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << "EXIT_FAILURE: " << EXIT_FAILURE << std::endl;

        return element_value_str;
    };
}

std::vector<float> yf::sql::sql_server::GetUgvInitPositionFromMapStatus()
{
    auto init_position_id  = this->GetUgvInitPositionIdFromMapStatus();

    float x = this->GetUgvInitPositionElement(init_position_id, "init_position_x");
    float y = this->GetUgvInitPositionElement(init_position_id, "init_position_y");
    float theta = this->GetUgvInitPositionElement(init_position_id, "init_position_theta");

    std::vector<float> init_position;

    init_position.push_back(x);
    init_position.push_back(y);
    init_position.push_back(theta);

    return init_position;
}

std::string yf::sql::sql_server::GetActivatedMapName()
{

    // query string
    std::string query_update;

    // input


    // output
    std::string map_name;

    //"SELECT position_name FROM data_ugv_mission_config where model_config_id = 1 ORDER BY mission_order"
    try
    {
        Connect();

        query_update = "SELECT map_name FROM data_ugv_map where status = 1 ";

        auto result = nanodbc::execute(conn_,query_update);

        // if there are new schedules available, sql module will mark down all the available schedule ids
        while(result.next())
        {
            map_name = result.get<std::string>(0);
        };

        Disconnect();

        return map_name;
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << "EXIT_FAILURE: " << EXIT_FAILURE << std::endl;

        return map_name;
    };
}

std::string yf::sql::sql_server::CountdownTimeSec(const int &countdown_sec)
{
    return this->get_future_db_time(countdown_sec);
}

int yf::sql::sql_server::GetScheduleCommand(const int &id)
{
    std::string query_update;

    int schedule_command;

    //"SELECT ID FROM schedule_table where status=1 AND planned_start > '2021-02-06 11:10:08.000'"
    try
    {
        Connect();

        query_update = "SELECT schedule_command FROM sys_schedule where ID = "+std::to_string(id)+" AND status=1 AND planned_start > '"+ TimeNow() +"'";

        auto result = nanodbc::execute(conn_,query_update);

        while(result.next())
        {
            schedule_command = result.get<int>(0);
        };

        Disconnect();

        return schedule_command;
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << "EXIT_FAILURE: " << EXIT_FAILURE << std::endl;

        return -999;
    }
}





















// connect_database
//
// get_latest_job_task
// get_waiting_schedule_job
// get_current_schedule_task
// get_task_detail_of_job
// get_task_detail
// get_location_info
// get_location_all_cleanzone
// get_clean_zone_detail
// get_location_info
// get_device_current_status
// get_all_waiting_schedule_job
// get_schedule_type

/// update_schedule_status
// update_task_status
// update_job_status
// update_device_status
// update_device_current_status_and_log
/// update_schedule_record
// update_job_record
// update_task_record
// update_schedule_finish_date

// insert_device_log
//

// check_schedule_operation_available
// check_all_schedule_operation_available