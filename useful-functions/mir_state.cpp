// 2021-03-08

// mir state testing

#include <iostream>

#include "../include/ugv.h"
#include "../include/sql.h"
#include "../include/nw_status.h"

#include<iostream>
#include<vector>
#include<algorithm>
#include<deque>
#include <thread>

bool isOne(int x){
    return x == 1;
}

void thread_Web_UgvStatus(const int& sleep_duration,yf::ugv::mir& mir)
{
    std::thread::id this_id = std::this_thread::get_id();

    while(1)
    {
        ///TIME
        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_duration));

//        std::cout << "thread[" << std::this_thread::get_id << "]" << std::endl;

        auto status = mir.GetUgvStatus();



        std::cout << "thread[" << this_id << "]: mir_battery_percentage: " << status.battery_percentage << std::endl;
    }
}

int main()
{


    std::shared_ptr<yf::status::nw_status> nw_status_ptr = std::make_shared<yf::status::nw_status>();
//    std::shared_ptr<yf::sql::sql_server> sql_ptr = std::make_shared<yf::sql::sql_server>("SQL Server","192.168.7.84","NW_mobile_robot_sys","sa","wuyongfeng1334");
    std::shared_ptr<yf::sql::sql_server> sql_ptr = std::make_shared<yf::sql::sql_server>("SQL Server","192.168.7.127","NW_mobile_robot_sys","sa","NWcadcam2021");

    yf::ugv::mir mir100;

    mir100.Start("192.168.7.34", nw_status_ptr, sql_ptr);

#if 0

    auto status = mir100.GetUgvStatus();

    std::thread t1, t2;

    int duration_1 = 200;

    int duration_2 = 200;

    t1 = std::thread(&thread_Web_UgvStatus,std::ref(duration_1),std::ref(mir100));

    t2 = std::thread(&thread_Web_UgvStatus,std::ref(duration_2),std::ref(mir100));


    while(1)
    {
        ///TIME
        std::this_thread::sleep_for(std::chrono::milliseconds(10000));
    }
//    std::thread t2(&thread_Web_UgvStatus);

#endif

    mir100.PostMissionForDebugTest(8063);
    mir100.PostActionsForDebugTest(8063);


    std::cout << "done" << std::endl;
#if 0

//    std::cout << "docking guid:" << mir100.GetDockingGUID("21bd0de2-b2f7-11eb-bb31-00012978eb45") << std::endl;

//    auto status = mir100.GetUgvStatus();


    mir100.PostMissionForDebugTest(6046);
    mir100.PostActionsForDebugTest(6046);


////
//    mir100.ClearErrorState();
//
//    mir100.ChangeMapByDBMapStatus();
//
//    ///TIME
//    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
//
//    if(mir100.WaitForModeId(7,1))
//    {
//        mir100.ChangeInitPositionByDBMapStatus();
//    }

    mir100.PostActionSpeed(0.2,"221d0b85-bc84-11eb-9acb-00012978eb45",0);

    mir100.PostActionAdjustLocalization("221d0b85-bc84-11eb-9acb-00012978eb45",0);


    //todo: input an valid order: 4
    // find last valid order: 3
    ///*
    int cur_valid_order = 4;

    ///* input is_valid vector
    std::vector<int> v_test = {0,0,0,1,1,0,1,0,0,0,0};

    ///* element to be found
    std::vector<int> value = {1};

    /// valid_indexes
    std::vector<int> valid_indexes;

    std::vector<int>::iterator first_index = std::find(v_test.begin(), v_test.end(), 1);

    std::vector<int>::iterator last_index = std::find_end(v_test.begin(), v_test.end(), value.begin(), value.end());

    int fvv = first_index-v_test.begin();
    int lvv = last_index-v_test.begin();
    std::cout << "first valid index: " << fvv << std::endl;
    std::cout << "last valid index: " << lvv << std::endl;

    if(first_index == std::end(v_test))
    {
        std::cout << "there is no any valid index"<< std::endl;
    }
    else
    {
        //todo: for loop, find all valid index. push back to valid_indexes

        std::vector<int>::iterator iter = v_test.begin();

        while ((iter = std::find_if(iter, v_test.end(), isOne)) != v_test.end())
        {
            // Do something with iter
            valid_indexes.push_back(iter-v_test.begin());

            iter++;
        }
    }

    std::vector<int>::iterator cur_valid_index_index = std::find(valid_indexes.begin(), valid_indexes.end(), cur_valid_order);


    auto cur_valid_order_index = cur_valid_index_index - valid_indexes.begin();
    auto last_valid_order_index = cur_valid_index_index - valid_indexes.begin() - 1;

    std::cout << "last_valid_order: " <<  valid_indexes[last_valid_order_index] << std::endl;




//    auto name_list = mir100.GetSessionNameList();
//
//    for (int n = 0; n < name_list.size(); n++)
//    {
//        std::cout << "name[" << n << "]: " << name_list[n] << std::endl;
//    }
//
//    auto map_name_list = mir100.GetMapNameList("Default site");
//
//    for (int n = 0; n < map_name_list.size(); n++)
//    {
//        std::cout << "name[" << n << "]: " << map_name_list[n] << std::endl;
//    }


    auto pos = mir100.GetCurPosition();

#endif

#if 0
    auto pos_name_list = mir100.GetPositionNameList("HA","HA_MainBlock_3/F");

    for (int n = 0; n < pos_name_list.size(); n++)
    {
        std::cout << "name[" << n << "]: " << pos_name_list[n] << std::endl;
    }

#endif

#if 0

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

//// Method Testing

int current_mission_order;

/// move to point_name, based on mission_order.
////    mir100.GetPositionGUID();
//    mir100.PostActionMove("870d9691-8dd7-11eb-a5e3-00012978eb45", "1fe736a6-96a4-11eb-b10a-00012978eb45", 1);

/// set PLC002 = 1,2,3,4,5, based on mission_order
//    mir100.PostActionSetPLC(2,1, "1fe736a6-96a4-11eb-b10a-00012978eb45",1);

/// set PLC001 = 1
//    mir100.PostActionSetPLC(1,1, "1fe736a6-96a4-11eb-b10a-00012978eb45",1);

/// wait for PLC001 = 0
//    mir100.PostActionWaitPLC(1,0,"1fe736a6-96a4-11eb-b10a-00012978eb45",1);

/// get position guid
//    auto position_guid = mir100.GetPositionGUID("cb4cfe79-8dd6-11eb-a5e3-00012978eb45","corridor_handrail_001_via001_front");
//    std::cout << "position_guid: " << position_guid << std::endl;


//// STATUS Testing

auto state_id = mir100.GetState();

std::cout << "state_id: " << state_id << std::endl;

#endif
//
//    mir100.PostMissionQueue("a099f581-9b33-11eb-a67a-00012978eb45");
//
//    mir100.Play();

#if 0

/// how to post a new mission.
///
/// @@ input: model_config_id

    int model_config_id = 8;

    bool ugv_mission_continue_flag = false;

//    auto ugv_config_num2 = sql_ptr->GetUgvMissionConfigNum(model_config_id);

//    std::cout << "ugv_config_num2: " << ugv_config_num2 << std::endl;

    /// 1. post a new mission
    mir100.PostMission(model_config_id);
    /// 2. post actions
    mir100.PostActions(model_config_id);

    /// 3. post to mission_queue, set start flag.

    /// 4. wait 500ms

    /// 5. execute arm mission

    auto ugv_config_num = sql_ptr->GetUgvMissionConfigNum(model_config_id);

    for (int n = 1; n < ugv_config_num + 1 ; n++)
    {
        // if ugv is ready, we can start next step. or wait 2 minutes for ugv to resume.

        // How to judge whether ugv is ready or not?
        // 1. plc 004 == 1
        // 2. get state, should equal to executing
        // 3. mission_continue_flag
        ugv_mission_continue_flag = mir100.InitMissionStatusCheck(2);

        if(ugv_mission_continue_flag)
        {
            /// method: check_plc_003_flag
            // wait for plc_003 == 1, timeout 2min.

            bool check_plc_003_flag = true;

            std::string time_future = sql_ptr->CountdownTime(sql_ptr->TimeNow(),5);

            while (check_plc_003_flag)
            {

                int plc_003_value = mir100.GetPLCRegisterIntValue(3);

                if(plc_003_value == 1)
                {
                    check_plc_003_flag = false;
                }

                /// if wait too long.
                if(!sql_ptr->isFutureTime(time_future, sql_ptr->TimeNow()))
                {
                    check_plc_003_flag = false;
                    ugv_mission_continue_flag = false;
                    break;
                }
            }



        }
        else
        {
            // anyway, we should break the mission loop.
            break;
        }
    }

    /// finish the current job
    // set
    mir100.SetPLCRegisterIntValue(4,0);

    /// check  status
    // (1) get current ugv mission order.

#endif

#if 0

    /// 1.1 get mission_guid from REST
    std::string mission_guid = mir100.GetCurMissionGUID();

    /// 2. get position_names from DB, based on model_config_id
    std::deque<std::string> position_names = sql_ptr->GetUgvMissionConfigPositionNames(model_config_id);

    /// 3.
    /// a. get map name from db.
    auto model_id = sql_ptr->GetModelId(model_config_id);
    auto map_id = sql_ptr->GetMapId(model_id);
    auto map_name = sql_ptr->GetMapElement(map_id,"map_name");
    /// b. based on map_name, retrieve map_guid from REST.
    auto map_guid = mir100.GetMapGUID(map_name);

    // for loop
    // get mission_name from REST. mission_order from db
    //
    int total_position_num = sql_ptr->GetUgvMissionConfigNum(model_config_id);
    int priority = 1;

    for (int mission_count = 1; mission_count <= total_position_num; mission_count++)
    {
        // get position name.
        std::string position_name = position_names[mission_count-1];

        //todo: get position_guid



        /// b. position_guid
        std::string position_guid = mir100.GetPositionGUID(map_guid,position_name);

        //1. get ugv_mission_config_id, based on mission_count,
        // prepare: mission_config_id

        //@@ input: position_guid, mission_guid(done)
        //
        //
        mir100.PostActionMove(position_guid, mission_guid, priority);
        priority++;

        mir100.PostActionSetPLC(2,mission_count,mission_guid,priority);
        priority++;

        mir100.PostActionSetPLC(1,1,mission_guid,priority);
        priority++;

        mir100.PostActionWaitPLC(1,0,mission_guid,priority);
        priority++;

    }

#endif



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

/// vector iterator tutorial
#if 0
    std::vector<int> v;
    v.push_back(10);
    v.push_back(-100);
    v.push_back(20);
    v.push_back(40);
    v[2] = 30;
    v.erase(v.begin() + 1);  // erase the second element (value = -100)
    for (size_t i = 0; i < v.size(); i++) {
        std::cout << v[i] << std::endl;
    }  // prints: 10, 20, 30 (on separate lines)
    v.clear();

    std::vector<int> v1{0,10,20,30,40,50,60,70,80,90};
    std::vector<int> v2(v1.begin(), v1.end());
    // v2 = {0,10,20,30,40,50,60,70,80,90}
    std::vector<int> v3(v2.begin() + 2, v2.begin() + 6);
    // v3 == {20,30,40,50}
    v3.erase(v3.begin() + 1, v3.begin() + 2);
    // v3 == {20,50}
    std::reverse(v1.begin(), v1.end() - 2);
#endif
    return 1;
}