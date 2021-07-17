
#include "../include/sql.h"

#include "../include/al.h"

using std::cout;
using std::endl;

int main()
{

/// For localhost: ROG
//    yf::sql::sql_server sql("SQL Server","192.168.7.84","NW_mobile_robot_sys","sa","wuyongfeng1334");
//    std::shared_ptr<yf::sql::sql_server> sql = std::make_shared<yf::sql::sql_server>("ODBC Driver 17 for SQL Server","localhost","NW_mobile_robot_sys","sa","wuyongfeng1334");

/// For localhost: NW 238 OFFICE
//    yf::sql::sql_server sql("ODBC Driver 17 for SQL Server","192.168.0.8","NW_mobile_robot_sys","sa","Willsonic2010");


/// For IPC1
    std::shared_ptr<yf::sql::sql_server> sql = std::make_shared<yf::sql::sql_server>("SQL Server","192.168.7.127","NW_mobile_robot_sys","sa","NWcadcam2021");


    // upload chinese character.
    nanodbc::string error_description = "手臂錯誤!!";
//    std::string error_description = "Arm Error!";
    NANODBC_TEXT("");
    NANODBC_TEXT("123");
    sql->UpdateErrorLog1(2,error_description);


//    auto failed_q = sql->GetFailedTaskIds(10);
    auto cur_first_valid_order_  = sql->GetRedoFirstValidOrder(26);
    auto cur_last_valid_order_   = sql->GetRedoLastValidOrder(26);
    //  e.g {3,4,5}
    auto cur_valid_indexes_ = sql->GetRedoValidIndexes(26);



    sql->GetRedoArmConfigValidResultQueue(26);

    auto size = sql->GetFailedTaskIds(26).size();

    sql->FillUMCRedoTableErrorPart(8047);


    sql->FillTaskTableForCurJob(10050);



    int cur_order = 5;

    auto v = sql->GetValidIndexes(4038);
    auto first = sql->GetFirstValidOrder(4038);
    auto last  = sql->GetLastValidOrder(4038);

    int last_valid_order;

    if(cur_order == first)
    {
        last_valid_order = cur_order;
    }
    else
    {
        std::vector<int>::iterator cur_valid_index_index = std::find(v.begin(), v.end(), cur_order - 1);

        auto last_valid_order_index = cur_valid_index_index - v.begin() - 1;

        last_valid_order = v[last_valid_order_index] + 1;
    }

    sql->UpdateDeviceBatteryCapacity("ugv",10);

    sql->UpdateDeviceUgvCurPosition(10,10,10);


#if 0

    std::cout << "ODBCConnectionStr: " << sql->getODBCConnectionStr() << std::endl;
    //"Driver={SQL Server};Server=192.168.0.8;Database=NW_mobile_robot_sys;Uid=sa;Pwd=Willsonic2010"

    std::cout << sql->GetSysControlMode() << std::endl;

    auto motion_type =  sql->GetArmMotionType(2);

    std::cout << "mt: " << motion_type << std::endl;

    auto plain_p2_id = sql->GetArmMissionPointId(1, "plain_cleaning_p2");
    std::cout << "plain_p2_id: " << plain_p2_id << std::endl;

    auto plain_p2 = sql->GetArmPoint(plain_p2_id);

    std::cout << "plain_p2: " << plain_p2.x << std::endl;
    std::cout << "plain_p2: " << plain_p2.y << std::endl;
    std::cout << "plain_p2: " << plain_p2.z << std::endl;
    std::cout << "plain_p2: " << plain_p2.rx << std::endl;
    std::cout << "plain_p2: " << plain_p2.ry << std::endl;
    std::cout << "plain_p2: " << plain_p2.rz << std::endl;

//    auto model_config_id = sql->GetModelConfigId(2);
//
//    std::cout << "model_config_id: " << model_config_id << std::endl;

    std::cout << "mission_config_num: " << sql->GetUgvMissionConfigNum(1) << std::endl;

    // get arm_config_id based on model_config_id and plc_001
    //
    std::cout << "arm_config_id: " <<sql->GetArmConfigId(2,2) << std::endl;

    // get position names
    auto position_names = sql->GetUgvMissionConfigPositionNames(1);

    std::cout << position_names.size() << std::endl;

    std::cout << position_names[0] << std::endl;
    std::cout << position_names[1] << std::endl;
    std::cout << position_names[2] << std::endl;
    std::cout << position_names[3] << std::endl;

    // get model_id
    auto model_id = sql->GetModelId(3);
    std::cout << "model_id: " << model_id << std::endl;

    // get map_id
    auto map_id = sql->GetMapId(7);
    std::cout << "map_id: " << map_id << std::endl;

    // get map_element
    // a. map_guid
    // b. location_site_id
    // c. location_building_id
    // d. location_floor_id
    auto map_guid = sql->GetMapElement(1,"map_guid");
    cout << "map_guid: " << map_guid << endl;

    auto site_info = sql->GetSiteInfo(3);
    cout << "site: " << site_info << std::endl;

    auto building_info = sql->GetBuildingInfo(3);
    cout << "building_info: " << building_info << std::endl;

    auto floor_info = sql->GetFloorInfo(1);
    cout << "floor_info: " << floor_info << std::endl;

    sql->UpdateTime();
    auto time_year = sql->get_time_element("year");
    auto time_month = sql->get_time_element("month");
    auto time_day = sql->get_time_element("day");
    auto time_hour = sql->get_time_element("hour");
    auto time_min = sql->get_time_element("min");

    auto time = time_year + time_month + time_day + "_" + time_hour + time_min;

    cout << "time: " << time << endl;


#endif

#if 0

    /// workflow

    /// 0. get cur_job_id
    int cur_job_id = 2;

    /// 1. get model_config_id
    auto model_config_id = sql->GetModelConfigId(cur_job_id);

    /// 2. based on model_config_id, get task_mode, ugv_config_id.
    //
    // 2.1 task_mode
    auto task_mode = sql->GetModelConfigElement(model_config_id, "task_mode");
    // 2.2 ugv_config_id
    auto ugv_config_id = sql->GetModelConfigElement(model_config_id, "ugv_config_id");

    // 2.3 arm_config_id
    // get arm_config_id based on model_config_id and plc_001
    //
    // 2021/04/05
    //todo:
    // (0) get total ugv_mission_config_num.  4
    // (1) wait for plc_001 = 1;
    // (2) check current plc_001 value.
    // (3) get current arm_config_id, based on model_config_id and plc_002
    // (4) ....
    std::cout << "arm_config_id: " <<sql->GetArmConfigId(model_config_id,2) << std::endl;

    auto  arm_config_id = sql->GetArmConfigId(model_config_id,2);

    /// 3. based on task_mode, change the tool.

    /// 3.1 based on model_config_id, mission_order, position_name, create a ugv mission.

    /// 4. based on arm_config_id, select a list of arm_missions.
    auto arm_mission_config_ids = sql->GetArmMissionConfigIds(arm_config_id);

    /// 5. For loop.

    for (int n = 0; n < arm_mission_config_ids.size(); n++)
    {
        int cur_arm_mission_config_id = arm_mission_config_ids[n];

        //5.1 Get motion_type
        auto motion_type =  sql->GetArmMotionType(cur_arm_mission_config_id);

        //5.2 based on motion_type, get relevant init_clean_points.

        std::deque<yf::data::arm::Point3d> init_cleaning_points;

        switch (motion_type)
        {
            case 1: // plain_cleaning
            {

                auto plain_p1_id = sql->GetArmMissionPointId(1, "plain_cleaning_p1");
                auto plain_p2_id = sql->GetArmMissionPointId(1, "plain_cleaning_p2");
                auto plain_p3_id = sql->GetArmMissionPointId(1, "plain_cleaning_p3");
                auto plain_p4_id = sql->GetArmMissionPointId(1, "plain_cleaning_p4");

                init_cleaning_points.push_back(sql->GetArmPoint(plain_p1_id));
                init_cleaning_points.push_back(sql->GetArmPoint(plain_p2_id));
                init_cleaning_points.push_back(sql->GetArmPoint(plain_p3_id));
                init_cleaning_points.push_back(sql->GetArmPoint(plain_p4_id));

                break;
            }

            case 2: // line_cleaning
            {
                auto line_p1_id = sql->GetArmMissionPointId(1, "line_cleaning_p1");
                auto line_p2_id = sql->GetArmMissionPointId(1, "line_cleaning_p2");

                init_cleaning_points.push_back(sql->GetArmPoint(line_p1_id));
                init_cleaning_points.push_back(sql->GetArmPoint(line_p2_id));

                break;
            }
        }

        // 5.3 based on motion_type, generate relevant cleaning path. (points...)
        //
        yf::algorithm::cleaning_motion al_cleaning_motion;

        auto via_points = al_cleaning_motion.get_mop_via_points(motion_type, init_cleaning_points);

        std::cout << "via_points' size: " << via_points.size() << std::endl;

        // 5.4 Assign to Arm.
        //

        // 5.5 publish task to robotic arm.
        //
        // ArmTask("Post xxx")
    }

#endif



    return 1;
}