
#include "../include/sql.h"

#include "../include/al.h"

int main()
{
//    yf::sql::sql_server sql("SQL Server","192.168.7.84","NW_mobile_robot_sys","sa","wuyongfeng1334");
//    yf::sql::sql_server sql("ODBC Driver 17 for SQL Server","localhost","NW_mobile_robot_sys","sa","wuyongfeng1334");

    std::shared_ptr<yf::sql::sql_server> sql = std::make_shared<yf::sql::sql_server>("ODBC Driver 17 for SQL Server","localhost","NW_mobile_robot_sys","sa","wuyongfeng1334");
//    yf::sql::sql_server sql("ODBC Driver 17 for SQL Server","192.168.0.8","NW_mobile_robot_sys","sa","Willsonic2010");

    std::cout << "ODBCConnectionStr: " << sql->getODBCConnectionStr() << std::endl;
    //"Driver={SQL Server};Server=192.168.0.8;Database=NW_mobile_robot_sys;Uid=sa;Pwd=Willsonic2010"

    std::cout << sql->GetSysControlMode() << std::endl;

    auto motion_type =  sql->GetArmMotionType(2);

    std::cout << "mt: " << motion_type << std::endl;

    auto plain_p2_id = sql->GetArmPointId(1,"plain_cleaning_p2");
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

    /// workflow

    /// 0. get cur_job_id
    int cur_job_id = 2;

    /// 1. get model_config_id
    auto model_config_id = sql->GetModelConfigId(cur_job_id);

    /// 2. based on model_config_id, get task_mode, arm_config_id, ugv_config_id.
    //
    // 2.1 task_mode
    auto task_mode = sql->GetModelConfigElement(model_config_id, "task_mode");
    // 2.2 arm_config_id
    auto arm_config_id = sql->GetModelConfigElement(model_config_id, "arm_config_id");
    // 2.3 ugv_config_id
    auto ugv_config_id = sql->GetModelConfigElement(model_config_id, "ugv_config_id");

    /// 3. based on task_mode, change the tool.


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

                auto plain_p1_id = sql->GetArmPointId(1,"plain_cleaning_p1");
                auto plain_p2_id = sql->GetArmPointId(1,"plain_cleaning_p2");
                auto plain_p3_id = sql->GetArmPointId(1,"plain_cleaning_p3");
                auto plain_p4_id = sql->GetArmPointId(1,"plain_cleaning_p4");

                init_cleaning_points.push_back(sql->GetArmPoint(plain_p1_id));
                init_cleaning_points.push_back(sql->GetArmPoint(plain_p2_id));
                init_cleaning_points.push_back(sql->GetArmPoint(plain_p3_id));
                init_cleaning_points.push_back(sql->GetArmPoint(plain_p4_id));

                break;
            }

            case 2: // line_cleaning
            {
                auto line_p1_id = sql->GetArmPointId(1,"line_cleaning_p1");
                auto line_p2_id = sql->GetArmPointId(1,"line_cleaning_p2");

                init_cleaning_points.push_back(sql->GetArmPoint(line_p1_id));
                init_cleaning_points.push_back(sql->GetArmPoint(line_p2_id));

                break;
            }
        }

        // 5.3 based on motion_type, generate relevant cleaning path. (points...)
        //
        yf::algorithm::cleaning_motion al_cleaning_motion;

        auto via_points = al_cleaning_motion.get_via_points(motion_type,init_cleaning_points);

        std::cout << "via_points' size: " << via_points.size() << std::endl;

        // 5.4 Assign to Arm.
        //

        // 5.5 publish task to robotic arm.
        //
        // ArmTask("Post xxx")
    }





    return 1;
}