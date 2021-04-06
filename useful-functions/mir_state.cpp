// 2021-03-08

// mir state testing

#include <iostream>

#include "../include/ugv.h"
#include "../include/sql.h"
#include "../include/nw_status.h"

int main()
{
    std::shared_ptr<yf::status::nw_status> nw_status_ptr = std::make_shared<yf::status::nw_status>();
    std::shared_ptr<yf::sql::sql_server> sql_ptr = std::make_shared<yf::sql::sql_server>("ODBC Driver 17 for SQL Server","localhost","NW_mobile_robot_sys","sa","wuyongfeng1334");


    yf::ugv::mir mir100;

    mir100.Start("192.168.7.34", nw_status_ptr, sql_ptr);

//    auto connection_result = mir100.IsConnected();
//
//    std::cout << "IsConnected? " << connection_result << std::endl;
//
//    std::cout << "----------------------" << std::endl;
//
//    auto state_result = mir100.GetState();
//
//    std::cout << "State " << state_result << std::endl;
//
//    std::cout << "----------------------" << std::endl;

//    auto post_mission_result = mir100.PostMission(1);
//
//    std::cout << "post_mission_result " << post_mission_result << std::endl;
//
//    std::cout << "----------------------" << std::endl;

int current_mission_order;

/// move to point_name, based on mission_order.

/// set PLC002 = 1,2,3,4,5, based on mission_order
//    mir100.PostActionSetPLC(2,2, "1fe736a6-96a4-11eb-b10a-00012978eb45",1);

/// set PLC001 = 1
    mir100.PostActionSetPLC(1,1, "1fe736a6-96a4-11eb-b10a-00012978eb45",1);

/// wait for PLC001 = 0
    mir100.PostActionWaitPLC(1,0,"1fe736a6-96a4-11eb-b10a-00012978eb45",1);

#if 0

    Poco::JSON::Object Register;
    float value = 2014.1f;
    Register.set("value", value);

    Poco::JSON::Object Register1;
    float value1 = 2025.9f;
    Register1.set("value", value1);

    mir100.PutMethod("/api/v2.0.0/registers/101", Register);
    mir100.PutMethod("/api/v2.0.0/registers/102", Register);
    mir100.PutMethod("/api/v2.0.0/registers/102", Register1);
    mir100.PutMethod("/api/v2.0.0/registers/103", Register);
    mir100.PutMethod("/api/v2.0.0/registers/104", Register);
    mir100.PutMethod("/api/v2.0.0/registers/104", Register1);
    mir100.PostMethod("/api/v2.0.0/registers/105", Register1);

    int state = mir100.GetState();
    int state1 = mir100.GetState();
    int state2 = mir100.GetState();

#endif

#if 0

    // isConnected? === get

    // get_status === get

    // get_pasue === put
    // Status.set("state_id", 4);

    // get_ready  === put
    // Status.set("state_id", 3);

    // clear_error === put
    //Status.set("clear_error", true);

    Poco::JSON::Object Status;

    Status.set("clear_error", true);

    mir100.PutMethod("http://192.168.2.111/api/v2.0.0/status", Status);


//    Poco::JSON::Array::Ptr position = object->getArray("position");

    std::cout << "state: " << state << std::endl;

//    mir100.PutMethod("/api/v2.0.0/registers/104", Register);

//    mir100.PostMethod("/api/v2.0.0/registers/105", Register1);

//    mir100.UpdatePositionOnMap(19.048f, 31.291f, 27.605f);

#endif
    return 1;
}