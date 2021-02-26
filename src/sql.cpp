#include "../include/sql.h"

int  main()
{
    yf::sql::sql_server sql;

    sql.UpdateDeviceConnectionStatus("ugv", 1);
    sql.UpdateDeviceMissionStatus("ugv", 3);
    sql.UpdateDeviceBatteryCapacity("ugv", 76.85f);

    sql.GetTaskMode(1);
    sql.GetTaskCommand(1);

    std::cout << sql.TimeNow() << std::endl;

    auto v_time = sql.Time_str2vector(sql.TimeNow());

    auto time_countdown =   std::to_string((int)v_time[0]) + "-" +
                            std::to_string((int)v_time[1]) + "-" +
                            std::to_string((int)v_time[2]) + " " +
                            std::to_string((int)v_time[3]) + ":" +
                            std::to_string((int)v_time[4]+3) + ":" +
                            std::to_string((int)v_time[5]) + ".000";

    std::cout << time_countdown << std::endl;

//    sql.GetScheduleExecTime(1);
//    std::cout << "id 1 execute time: " << sql.get_execute_time() << std::endl;


    // update schedule table when schedule 14 is in process.
    // update schedule record table.
    //
    //sql.UpdateScheduleData(14,2);
    //sql.UpdateScheduleRecordData(14,2);

    // update schedule table when schedule 14 has finished.
    //
    //sql.UpdateScheduleData(14,3);
    //sql.UpdateScheduleRecordData(14,3);

    // check available schedule ids.
    // sql.RetrieveAllAvailableScheduleId();

    // todo: (1) wait for available schedules.
//    sql.WaitAvailableSchedules();
//
//    std::deque<int> q_schedules_id = sql.GetSchedulesId();
//
//    for (int i = 0 ; i < q_schedules_id.size(); i++)
//    {
//        std::cout << "id[" << i << "]: " << q_schedules_id[i] << std::endl;
//    }
    // todo: (2) should notify the [wait schedule thread]...
    // todo: (3) should be blocked and wait for notify by [thread do schedule].



    return 1;
}