#include <thread>

//glog
#include <glog/logging.h>

//Net
#include "../include/net.h"
#include "../include/net_w0303_server.h"
#include "../include/net_w0303_client.h"

//Database
#include "../include/sql.h"

//Manager
//#include "../include/manager.h"
#include "../include/data.h"



#if 0

// For cout enum class...
template<typename T>
std::ostream& operator<<(typename std::enable_if<std::is_enum<T>::value, std::ostream>::type& stream, const T& e)
{
    return stream << static_cast<typename std::underlying_type<T>::type>(e);
}

// startup the server. will keep waiting for the new connection. will keep listening and respond the msg.
void th_server_start(IPCServer& server, bool& server_flag)
{
    server.Start();

    while (server_flag)
    {
        server.Update(-1, true);
    }

    server.Stop();
}

void NwSysStartup() {


    std::mutex mux;
    std::vector<std::vector<char>> messages;


    // TODO: startup IPC server. Non-stop server.
    IPCServer ipc_server(12345);

    bool sever_flag = true;

    std::thread t1(th_server_start, std::ref(ipc_server), std::ref(sever_flag) );

    std::cout << "wait for arm connection..." << std::endl;
    // block... wait for arm connection.... should i notify the database???
    ipc_server.WaitConnectionFromArm();
    std::cout << "connected!" << std::endl;

    // update the status???

    // block... wait time and schedule...

    // pretend some work... 5s
    std::this_thread::sleep_for(std::chrono::milliseconds(500000));

    //TODO: startup Arm manager. try to connect the arm, and then the arm manager should keep waiting.
//    yf::manager::arm_manager tm5;
//
//    tm5.

	//TODO: startup status manager. check status
	yf::manager::sys_status_manager sys_status_manager;

	std::cout << "arm connection status: " << sys_status_manager.GetArmConnectionStatus() << std::endl;


	// todo: Safety stop server thread.
    sever_flag = false;
    t1.join();
}

#endif

class nw_sys
{
public:

    nw_sys(uint16_t nPort)
            : ipc_server_(nPort) , ipc_port_(nPort){}
    virtual ~nw_sys(){}

public:

    void Startup()
    {
        // clear schedules...
        all_avaiable_schedule_ids.clear();
        schedule.clear();

        // todo: startup all sever and clients...
    }

    void Close()
    {
        // todo: close all sever, clients....
        //  1. close sever.
        this->CloseIPCServer();
        LOG(INFO) << "ipc server closed.";

        //  2. close glog
        LOG(INFO) << "nw sys closed.";
        google::ShutdownGoogleLogging();

    }

    void GetStatusLog()
    {
        // For Arm
        //
        //  1. preparation.
        std::string arm_connection_status_str{};
        std::string arm_mission_status_str{};

        // (1) arm connection status
        if(arm_connection_status_ == yf::data::common::ConnectionStatus::Connected)
        {
            arm_connection_status_str = "Connected";
        }
        else
        {
            arm_connection_status_str = "Disconnected";
        }

        // (2) arm mission status
        switch (arm_mission_status_)
        {
            case yf::data::common::MissionStatus::Error:
            {
                arm_mission_status_str = "Error";
                break;
            }
            case yf::data::common::MissionStatus::Running:
            {
                arm_mission_status_str = "Running";
                break;
            }
            case yf::data::common::MissionStatus::Idle:
            {
                arm_mission_status_str = "Idle";
                break;
            }
            case yf::data::common::MissionStatus::Finish:
            {
                arm_mission_status_str = "Finish";
                break;
            }
            case yf::data::common::MissionStatus::Pause:
            {
                arm_mission_status_str = "Pause";
                break;
            }
        }

        LOG(INFO) << "[Arm status]";
        LOG(INFO) << "[Arm Connection Status]: " << arm_connection_status_str;
        LOG(INFO) << "[Arm Mission Status]: " << arm_mission_status_str;
    }

public:

    void StartupIPCServer()
    {
        if(ipc_port_ == 12345)
        {
            th_ipc_server_ = std::thread(&nw_sys::thread_ipc_sever_startup , this, std::ref(ipc_server_), std::ref(ipc_server_flag_));

            LOG(INFO) << "wait for arm connection...";

            // block... wait for arm connection.... should i notify the database???
            ipc_server_.WaitConnectionFromArm();

            LOG(INFO) << "arm connected.";
            // check the connection status;
            arm_connection_status_ = ipc_server_.GetArmConnectionStatus();
            // check the mission status; by sending msg."Get Status"
            UpdateArmCurMissionStaus();

        }
        else
        {
            std::cerr << "Invalid port number! nw_sys out!!!" << std::endl;
        }
    }

    void CloseIPCServer()
    {
        // Just close server.
        ipc_server_flag_ = false;

        // need a msg to let ipc figure out the server is closed.
        SendRandomMsg();

        // manually update the status
        //
        // (1) Arm
        arm_connection_status_ = yf::data::common::ConnectionStatus::Disconnected;
        arm_mission_status_ = yf::data::common::MissionStatus::Error;

        // (2) IPC2

        // join the thread
        th_ipc_server_.join();
    }

    // Arm --- NetMsg
    // message arm and update status.
    void NetMessageArm(const std::string& str)
    {
        yf::net::message<CustomMsgTypes> msg;

        msg.body.resize(str.size());
        msg.body.assign(str.begin(),str.end());

        // input: arm client, and msg.
        ipc_server_.MessageClient(ipc_server_.GetArmClient(), msg);

        LOG(INFO) << "[IPC1 ---> Arm]: " << str;

        return;

#if 0   // No Need to update the arm status in msg function.
        // update the arm status, will block the code.
        arm_mission_status_ = ipc_server_.GetArmStatus();

        return arm_mission_status_;
#endif

    }

    // todo: My check function.
    //  1. send msg ---> keep looking the arm ---> if error,pause. handle the sys different.
    //                                        ---> if running. keep waiting..
    // keep checking the arm status.
    bool CheckArmStatus()
    {
        switch (arm_mission_status_)
        {
            case yf::data::common::MissionStatus::Running:
            {
                std::cout << "the arm is running what i pass, perfect." << std::endl;
                break;
            }
            case yf::data::common::MissionStatus::Finish:
            {
                std::cout << "the arm has finished its job, next command please." << std::endl;
                break;
            }
            case yf::data::common::MissionStatus::Idle:
            {
                std::cout << "the arm has finished its job, next command please." << std::endl;
                break;
            }
            case yf::data::common::MissionStatus::Error:
            {
                std::cerr << "the arm return error code, please figure it out." << std::endl;
                break;
            }
            case yf::data::common::MissionStatus::Pause:
            {
                std::cout << "the arm has pause its job, please just wait for resume." << std::endl;
                std::cout << "or i will start counting, 5 minutes later will alarm the user." << std::endl;
                break;
            }
        }
    }

    // Arm --- Network Msg
    // retrieve latest net msg from Arm
    std::string GetArmLatestNetMsg()
    {
        return ipc_server_.GetArmLatestNetMsg();
    }

    // Arm --- Network Msg
    // retrieve all msgs from Arm
    // todo:
    //  1. current the size of queue is infinite.
    //  2. need to limit the size.
    std::deque<std::string> GetArmAllNetMsgs()
    {
        return ipc_server_.GetArmAllNetMsg();
    }

    // Send the msg that used for checking whether the server is alive or not.
    void SendRandomMsg()
    {
        yf::net::message<CustomMsgTypes> msg;
        std::string str = "Get Status";

        msg.body.resize(str.size());
        msg.body.assign(str.begin(),str.end());

        ipc_server_.MessageClient(ipc_server_.GetArmClient(), msg);

        return;
    }

    // TODO: Arm --- Listen Node
    // Listen Node
    //
    void StartupArmLnClient()
    {
        // todo: need to check whether Arm has enter listen node first! --- if not, just return error!

        std::cout << "startup listen node client.." << std::endl;
        th_ipc_client_ln_ = std::thread(&nw_sys::thread_arm_ln_startup , this, std::ref(ipc_client_ln_), std::ref(ipc_client_ln_flag_));

    }

    void CloseArmLnClient()
    {
        std::cout << "close listen node client.." << std::endl;

        // stop the whole listen node program first, regradless the current task...
        ipc_client_ln_.SendMsg("$TMSCT,24,1,\r\n"
                               "StopAndClearBuffer(),*66\r\n");
        // todo: should I check the return msg first???

        // Exit the listen node, give back the control to Arm Network.
        ipc_client_ln_.SendMsg("$TMSCT,16,1,\r\n"
                               "ScriptExit(),*62\r\n");

        ipc_client_ln_flag_ = false;

        std::unique_lock<std::mutex> ul(mux_Blocking_ln);
        cv_Blocking_ln.notify_one();

        th_ipc_client_ln_.join();
    }

    // Listen Node
    //
    // Arm --- Listen Node send msg to Arm via Listen Node
    void LnMessageArm(const std::string& str)
    {
        // todo: str ---> final_str
        // todo: final_str ---> our msg type....

        // actually, not much help with this if check... the socket will open anyway..
        if (ipc_client_ln_.isConnected())
        {
            ipc_client_ln_.SendMsg(str);
            std::cout << "IPC send Listen Node Msg to Arm: " << str << std::endl;
        }
        else
        {
            std::cerr << "Not connection established." << std::endl;
        }
    }

    // Arm --- ListenNode Msg
    // retrieve latest listen node msg from Arm
    std::string GetArmLatestLnMsg()
    {

    }

    // Arm --- ListenNode Msg
    // retrieve all listen node msgs from Arm
    std::deque<std::string> GetArmAllLnMsg()
    {

    }

    // Arm --- Status
    void WaitArmReturnFinish()
    {
        while(arm_mission_status_ != yf::data::common::MissionStatus::Finish)
        {
            // wait 500ms.
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            // update_arm_status
            arm_mission_status_ = ipc_server_.GetArmMissionStatus();
        }
    }

    // Arm --- Status
    yf::data::common::MissionStatus GetArmStatus()
    {
        return arm_mission_status_;
    }

    // todo:
    //  Arm  ---- Retrieve Arm Current Mission Status
    //  1. Send msg "Get Status" to arm, and then block the whole code, waiting for return status.
    void RetrieveArmCurrentMissionStatus()
    {
        yf::net::message<CustomMsgTypes> msg;
        std::string str = "Get Status";

        msg.body.resize(str.size());
        msg.body.assign(str.begin(),str.end());

        ipc_server_.MessageClient(ipc_server_.GetArmClient(), msg);

        // get status and preserve the status in arm_mission_status_
        arm_mission_status_ = ipc_server_.GetArmMissionStatus();

//        GetStatusLog();

        // todo: Check has the status already been modified automatically? ---> triggered by OnMessage()
#if 0

        if(arm_mission_status_ == yf::common::MissionStatus::Error)
        {
            std::cerr << "Arm Error" << std::endl;
            return;
        }

        if(arm_mission_status_ == yf::common::MissionStatus::Pause)
        {
            std::cerr << "Arm Pause" << std::endl;
            return;
        }

        if(arm_mission_status_ == yf::common::MissionStatus::Finish)
        {
            std::cerr << "Arm Finish" << std::endl;
            return;
        }

        if(arm_mission_status_ == yf::common::MissionStatus::Idle)
        {
            std::cerr << "Arm Idle" << std::endl;
            return;
        }

        if(arm_mission_status_ == yf::common::MissionStatus::Running)
        {
            std::cerr << "Arm Running" << std::endl;
            return;
        }

#endif
    }

    // todo: for mission task
    //  Arm --- --- Check function 2.
    //  Keep Updating Arm Current Mission Status
    //
    void UpdateArmCurMissionStaus()
    {
        bool update_flag = true;

        while (update_flag)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));

            RetrieveArmCurrentMissionStatus();

            switch (arm_mission_status_)
            {

                // exit with error
                case yf::data::common::MissionStatus::Error:
                {
                    // todo:
                    //  1. update task status
                    //  2. update schedule status
                    std::cout << "From UpdateArmCurMssionStaus(): " << "IPC Needs to update the task status!!!" << std::endl;
                    std::cout << "From UpdateArmCurMssionStaus(): " << "IPC Needs to update the schedule status!!!" << std::endl;
                    std::cout << "From UpdateArmCurMssionStaus(): " << "IPC Needs to update the Task Failure Queue!!!" << std::endl;

                    // todo:
                    //  1. Need to record current job, task, etc...
                    //  2. Report to database....

                    // todo:
                    //  3. Anyway, current arm_task will be regarded as fail. just record and skip it.
                    update_flag = false;
                    break;
                }

                case yf::data::common::MissionStatus::Pause:
                {
                    // todo:
                    //  1. update task status
                    //  2. update job status
                    //  2. update schedule status
                    std::cout << "From UpdateArmCurMssionStaus(): " << "IPC Needs to update the task status!!!" << std::endl;
                    std::cout << "From UpdateArmCurMssionStaus(): " << "IPC Needs to update the schedule status!!!" << std::endl;
                    std::cout << "From UpdateArmCurMssionStaus(): " << "IPC Needs to update the Task Failure Queue!!!" << std::endl;

                    // todo:
                    //  1. Need to record current job, task, etc...
                    //  ***2. Waitfor resume.... should I set update_flag = false? (no need now)
                    break;
                }

                // exit safely
                case yf::data::common::MissionStatus::Finish:
                {
                    // todo:
                    //  1. update task status
                    //  2. update schedule status
                    std::cout << "From UpdateArmCurMssionStaus(): " << "IPC Needs to update the task status!!!" << std::endl;
                    std::cout << "From UpdateArmCurMssionStaus(): " << "IPC Needs to update the schedule status!!!" << std::endl;
                    std::cout << "From UpdateArmCurMssionStaus(): " << "IPC Needs to update the Task Failure Queue!!!" << std::endl;

                    // todo:
                    //  1. Need to record current job, task, etc...
                    //  2. Report to database....

                    // todo:
                    //  3. Current arm_task will be regarded as success. just record and finish it.
                    update_flag = false;
                    arm_mission_flag_ = false;
                    break;
                }

                // exit safely
                case yf::data::common::MissionStatus::Idle:
                {
                    // todo:
                    //  1. update task status
                    //  2. update schedule status
                    std::cout << "From UpdateArmCurMssionStaus(): " << "IPC Needs to update the task status!!!" << std::endl;
                    std::cout << "From UpdateArmCurMssionStaus(): " << "IPC Needs to update the schedule status!!!" << std::endl;
                    std::cout << "From UpdateArmCurMssionStaus(): " << "IPC Needs to update the Task Failure Queue!!!" << std::endl;

                    // todo:
                    //  1. Need to record current job, task, etc...?
                    //  2. Report to database....?

                    // todo:
                    //  3. Current arm_task will be regarded as success. just (record and) finish it.
                    update_flag = false;
                    arm_mission_flag_ = false;
                    break;
                }

                // arm is working well, just keep checking
                case yf::data::common::MissionStatus::Running:
                {
                    // todo:
                    //  1. schedule and task should already been updated....
                    //  2. wait ...
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                    // todo:
                    //  3. Anyway, ipc will keep updating arm mission status until the arm has finished.
                    break;
                }
            }
        }
    }

    // todo: for mission task
    //  Arm --- --- Check function 1.
    //  Check Arm Current Mission Status
    //
    void CheckArmInitMssionStaus()
    {
        bool check_flag = true;

        while (check_flag)
        {

            RetrieveArmCurrentMissionStatus();

            switch (arm_mission_status_)
            {
                case yf::data::common::MissionStatus::Idle:
                {
                    // Everything is fine. The arm is waiting any mission to do. Enjoy.
                    arm_mission_flag_ = true;

                    // No need to check again, break
                    check_flag = false;
                    break;
                }

                case yf::data::common::MissionStatus::Error:
                {
                    // The Arm is Error!
                    arm_mission_flag_ = false;

                    // No need to check again, break
                    check_flag = false;
                    break;
                }

                case yf::data::common::MissionStatus::EStop:
                {
                    // The Arm is E-Stop!
                    arm_mission_flag_ = false;

                    // No need to check again, break.
                    check_flag = false;
                    break;
                }
                default:
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                    break;
                }
            }
        }
    }

    // TODO: Level 1: Update Arm command string form task queue.
    void UpdateArmCommandStr()
    {
        // retrieve arm command string from parsed task information.

        return;
    }

    // TODO: Level 1: Retrieve real path.
    std::string RetrieveRealPath(const std::string& str)
    {
        // hard code now
        std::string real_path = "$TMSCT,78,1,\r\n"
                                "float[] targetP1= {-20,-4,72,14,86,-1}\r\n"
                                "PTP(\"JPP\",targetP1,10,200,0,false),*6b\r\n";
        return real_path;
    }

    void AssignArmMission(const std::string& arm_command_str)
    {
        // todo: check the mission flag,
        if(arm_mission_flag_)
        {
            // assgin the task. Basically. they are all strings.
            UpdateArmCommandStr();

            // For listen node --- motion control
            // todo: hard code ---> soft code
            //
            if(arm_command_str == "EnterListenNode")
            {

                // 1. First of all, send the enter command.
                NetMessageArm(arm_command_str);
                std::this_thread::sleep_for(std::chrono::milliseconds(500));        //wait 500ms

                // 2. Start Listen node client. (Now hard code) and update the connection status
                ipc_client_ln_.Connect("192.168.2.29",5890);
                arm_ln_connection_status_ = yf::data::common::ConnectionStatus::Connected;

                std::cout << "listen node connection established." << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(500));        //wait 500ms

                // 3. retrieve the real_path, send the real_path and then trigger!
                std::string real_path = RetrieveRealPath("EMSD-GF-Handrail");

                ipc_client_ln_.SendMsg(real_path);
                std::this_thread::sleep_for(std::chrono::milliseconds(500));

                // 4. Exit the listen node.

                std::string ln_exit_str = "$TMSCT,16,1,\r\n"
                                          "ScriptExit(),*62\r\n";
                ipc_client_ln_.SendMsg(ln_exit_str);
                std::this_thread::sleep_for(std::chrono::milliseconds(200));

                // 5. Shut down the listen node connection and then update status.
                std::cout << "listen node connection destroyed." << std::endl;
                ipc_client_ln_.Disconnect();
                arm_ln_connection_status_ = yf::data::common::ConnectionStatus::Disconnected;

            }
            else
            {
                // Normal Arm msg, just send it peacefully.
                NetMessageArm(arm_command_str);
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }
        }
        else
        {
            // todo: mission deny.
            std::cout << "From AssignArmMission(): Mission deny!" << std::endl;
            return;
        }
    }
    ///////////////////////////////////////////////////////
    /// Job funtions..
    ////////////////////////////////////////////////////////
    const std::string currentDateTime() {

        struct tm newtime;

        __time64_t long_time;

        errno_t err;

        // Get time as 64-bit integer.
        _time64(&long_time);
        // Convert to local time.
        err = _localtime64_s(&newtime, &long_time);
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

        if (newtime.tm_mon < 10)
        {
            month = "0" + std::to_string(newtime.tm_mon);
        }
        else
        {
            month = std::to_string(newtime.tm_mon);
        }

        if (newtime.tm_mday < 10)
        {
            day = "0" + std::to_string(newtime.tm_mday);
        }
        else
        {
            day = std::to_string(newtime.tm_mday);
        }

        std::string hour = std::to_string(newtime.tm_hour);
        std::string min = std::to_string(newtime.tm_min);
        std::string sec = std::to_string(newtime.tm_sec);
        /// "2021-01-28 18:02:00.000"

        std::string  time = year + "-" + month + "-" + day + " " + hour + ":" + min + ":" + sec + ".000";

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

    ///////////////////////////////////////////////////////

    int GetScheduleNumber()
    {
        int number = 3;
        return number;
    }

    // Assign Each Task
    //
    void AssignTasks(yf::data::schedule::TaskForRefModel& task)
    {
        // For each task
        //
        task.Clear();

        // assign model name, retrieved from db.
        //
        task.model_config.name =  yf::data::models::ModelName::Desk;

        // Arm config --- init
        // (1) operation area
        // (2) ref_obstacle_p0
        // (3) ref_vision_p0
        // (4) ref_init_p0
        // (5) motion type
        // (6) ref_p0

        task.arm_motion_config_no = 3;

        for(int i = 0; i < task.arm_motion_config_no ; i++)
        {
            yf::data::arm::MotionConfig arm_motion_config;

            arm_motion_config.clean_type = yf::data::arm::CleanType::Mopping;
            arm_motion_config.ref_operation_area = yf::data::arm::OperationArea::Right;

            // ref_vision_obs_p0
            arm_motion_config.ref_vision_obs_p0.x   = 31.13;
            arm_motion_config.ref_vision_obs_p0.y   = -32.07;
            arm_motion_config.ref_vision_obs_p0.z   = 144.6;
            arm_motion_config.ref_vision_obs_p0.rx  = 5.4;
            arm_motion_config.ref_vision_obs_p0.ry  = 87.18;
            arm_motion_config.ref_vision_obs_p0.rz  = 128.92;

            // ref_vision_p0
            arm_motion_config.ref_vision_p0.x   = 30.13;
            arm_motion_config.ref_vision_p0.y   = -31.07;
            arm_motion_config.ref_vision_p0.z   = 143.6;
            arm_motion_config.ref_vision_p0.rx  = 4.4;
            arm_motion_config.ref_vision_p0.ry  = 85.18;
            arm_motion_config.ref_vision_p0.rz  = 124.92;

            // read ref_path (listen node) // todo: retrieve from database
            arm_motion_config.ref_path;

            // motion mode. landmark, listen node, network.. // todo: retrieve from database

            arm_motion_config.arm_motion_mode = yf::data::arm::MotionMode::NetCommand;

            task.arm_motion_configs.push_back(arm_motion_config);
        }

    }

    void PublishArmTask(const std::string& arm_command_str)
    {
        // todo: how to ensure the arm has received the msg?
        CheckArmInitMssionStaus();
        AssignArmMission(arm_command_str);
        UpdateArmCurMissionStaus();
    }

    // Assign Job retrieved from database
    //
    //
    void AssignJobs()
    {
        schedule.clear();

        // Compare schedule time with now().

        // todo:: db: retrieve schedule time...
        std::string time_from_database("2021-01-28 18:02:00.000");

        std::string time_now = currentDateTime();
        auto time_flag = IsLatestSchedule(time_from_database,time_now);

        if(time_flag)
        {
            // latest schedule!
            // todo:: db: latest schedule information from db.

            for(int i = 0; i < schedule_number ; i ++)
            {
                // Assign Job
                yf::data::schedule::Job job;

                job.appointment_config.location = yf::data::schedule::Location::HKSTP;
                job.appointment_config.floor = 2;
                job.appointment_config.start_time = "2021-01-28 18:02:00.000";
                job.task_no = 4;

                job.tasks.clear();

                for(int j =0 ; j < job.task_no ; j++)
                {
                    yf::data::schedule::TaskForRefModel task;

                    //todo: db: assign task information from db.
                    AssignTasks(task);

                    // push back a task...
                    job.tasks.push_back(task);
                }

                schedule.push_back(job);
            }
        }
        else
        {
            std::cout << "schedule outdated" << std::endl;
            return;
        };
    }

    void WaitSchedules()
    {
        while (schedule_flag)
        {
            while (wait_schedule_flag)
            {
                // Check whether arm is idle
                while (arm_mission_status_ != yf::data::common::MissionStatus::Idle)
                {
                    LOG(INFO) << "wait for arm is idle";
                    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                }

                while (schedule_number == 0)
                {
                    LOG(INFO) << "wait for incoming schedules";

                    //todo: retrieve schedule number from database...
                    schedule_number = GetScheduleNumber();

                    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                }

                // Assign all schedules
                AssignJobs();

                schedule_number = 0;
                wait_schedule_flag = false;

                // notify thread: wait schedule
                do_schedule_flag = true;
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                std::unique_lock<std::mutex> ul(mux_Blocking_do_schedule);
                cv_Blocking_do_schedule.notify_one();

                LOG(INFO) << "unlock thread: do schedule";

            }
            // (1) option 1 -- keep looping
            //std::this_thread::sleep_for(std::chrono::milliseconds(2000));

            // (2) option 2 -- mutex lock
            //
            // lock and wait for notify.
            LOG(INFO) << "lock thread: wait schedule";

            std::unique_lock<std::mutex> ul_wait(mux_Blocking_wait_schedule);
            cv_Blocking_wait_schedule.wait(ul_wait);

            LOG(INFO) << "thread: wait schedule has been unlocked";

        }

        LOG(INFO) << "schedule flag is false, thread: wait schedules should be shut down...";

    };

    void WaitForExecute(const std::string& execute_time)
    {
        // wait for execute time...
        //
        auto time_now = currentDateTime();
        auto time_temp = time_now;
        auto time_scheduled = execute_time;

        // wait for execute time..
        while (IsLatestSchedule(time_scheduled, time_temp))
        {
            time_temp = currentDateTime();
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        }
    }

    void DoSchedule()
    {
        // wait for do_schedule_flag = ture
        while(schedule_flag)
        {
            // Default status is locking....
            while (do_schedule_flag == false)
            {
                LOG(INFO) << "lock thread: do schedule";

                std::unique_lock<std::mutex> ul(mux_Blocking_do_schedule);
                cv_Blocking_do_schedule.wait(ul);
            }

            LOG(INFO) << "thread: do schedule has been unlocked";

            // if there is schedule
            //
            while  (schedule.size() != 0)
            {
                auto job = schedule.front();
                schedule.pop_front();

                // wait for execute time...
                //
                auto time_now = job.currentDateTime();
                auto time_temp = time_now;
                auto time_scheduled = job.appointment_config.start_time;

                if(IsLatestSchedule(time_scheduled, time_now))
                {
                    std::cout << "Received schedule!" << std::endl;
                    std::cout << "waiting..." << std::endl;
                    std::cout << "Execute time: "<< time_scheduled << std::endl;

                    // wait for execute time..
                    while(IsLatestSchedule(time_scheduled, time_temp))
                    {
                        time_temp = job.currentDateTime();
                        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                    }

                    // TODO: Execute the schedule!
                    DoTask(job.tasks);

                    // Update Task Status , Update Job status...
                }
                else
                {
                    std::cout << "Job outdated!" << std::endl;
                    std::cout << "Next Job...." << std::endl;
                }
            }

            // No more schedules
            //
            // reset do schedule thread to lock
            // wait for nitified by wait schedule thread
            do_schedule_flag = false;

            // trigger thread: wait schedule
            bool wait_schedule_flag = true;
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            std::unique_lock<std::mutex> ul_wait(mux_Blocking_wait_schedule);
            cv_Blocking_wait_schedule.notify_one();

            LOG(INFO) << "unlock thread: wait schedule";

            auto time_now = currentDateTime();

            std::cout << "no schedule!" << std::endl;
            std::cout << "current time: " << time_now << std::endl;

            // record the end time
            // update the schedule status to database
        }
    }

    void DoTask(std::deque<yf::data::schedule::TaskForRefModel> tasks)
    {
        while (!tasks.empty())
        {
            auto task = tasks.front();
            tasks.pop_front();

            // based on each task , execute the combined missions.

            // (0) Initialization

            std::string arm_init_p0 = "Set init p0 = 180,520,125,50,96,45";
            std::string arm_lm_init_p0 = "Set lm init p0 = 180,500,125,50,96,45";

            PublishArmTask(arm_init_p0);

            // (1) Move robot to init home position
            std::string arm_back_home = "BackToHomePosition";
            PublishArmTask(arm_back_home);

            // (2) Move to operation area
            std::string arm_operation_area = "1";
            PublishArmTask(arm_operation_area);

            // (3) Move to init p0
            PublishArmTask(arm_init_p0);

            // (4) Move to init landmark p0
            PublishArmTask(arm_lm_init_p0);

            // (5) Execute cleaning handrail task.
            PublishArmTask("Post Vision Task");

            // (6) Move back to init p0

            // (7) Move back to home position.

            // (8) Update the task status.
        }
    }

    // Thread --- ipc_server
    // thread: start up the server.
    void thread_ipc_sever_startup(IPCServer& server, bool& server_flag)
    {
        server.Start();

        while (server_flag)
        {
            server.Update(-1, true);
        }

        server.Stop();
    }

    // Thread --- arm_ln_client
    void thread_arm_ln_startup(IPCClient& client, bool& client_flag)
    {
        client.Connect("192.168.2.29", 5890);

        while (client_flag)
        {
            std::unique_lock<std::mutex> ul(mux_Blocking_ln);
            cv_Blocking_ln.wait(ul);
        }

        client.Disconnect();
    }

    // setter
    void set_schedule_status(const yf::data::common::MissionStatus& status)
    {
        schedule_status_ = status;
    }

private:
    // Status
    //
    // the main point is to know each device's status.
    //
    // Connection Status
    // (1) Arm connection
    // (2) MiR100 connection
    // (3) Arm Mission Status
    // (4) MiR Mission Status
    //
    // (1)
    // yf::common::ConnectionStatus    arm_connection_status_;             // tm_net_sever_connection_status
    // yf::common::ConnectionStatus    arm_ln_connection_status_;          // tm_ln_connection_status;
    // (2)
    // yf::common::ConnectionStatus    mir_connection_status_;             // mir_rest_api_client_connection_status
    // (3)
    // yf::common::MissionStatus       arm_mission_status_;                // tm_mission_status
    // (4)
    // yf::common::MissionStatus       mir_mission_status;                 // mir_mission_status


    // Net
    //
    // ipc_sever: for Network connection with Arm. (IPC2 in the future)
    IPCServer   ipc_server_;                                            // tm_net_server
    uint16_t    ipc_port_{};                                            // tm_net_server_port
    bool        ipc_server_flag_ = true;                                // tm_net_server_living_flag
    std::thread th_ipc_server_;                                         // th_tm_net_server

    // (1)
    yf::data::common::ConnectionStatus    arm_connection_status_;             // tm_net_sever_connection_status
    // (3)
    yf::data::common::MissionStatus       arm_mission_status_;                // tm_mission_status

    bool                                  arm_mission_flag_ = false;

    yf::data::common::ConnectionStatus    mir_connection_status_;             // mir_rest_api_client_connection_status
    yf::data::common::MissionStatus       mir_mission_status;                 // mir_mission_status

    // Arm Listen Node
    //
    //
    IPCClient   ipc_client_ln_;
    bool        ipc_client_ln_flag_ = true;
    std::thread th_ipc_client_ln_;

    // net_w0303_server.h line202   "arm_ln_status"
    yf::data::common::ConnectionStatus    arm_ln_connection_status_;          // tm_ln_connection_status;

    std::mutex mux_Blocking_ln;
    std::condition_variable cv_Blocking_ln;

    // Job
    //
    //
    yf::data::common::MissionStatus schedule_status_;

    // overall control
    //
    bool schedule_flag = true;

    //wait schedule
    //
    // (1) control flag
    bool wait_schedule_flag = true;
    // (2) mutex lock
    std::mutex mux_Blocking_wait_schedule;
    std::condition_variable cv_Blocking_wait_schedule;

    // do schedule
    //
    // (1) control flag..
    bool do_schedule_flag = false;
    // (2) mutex lock
    std::mutex mux_Blocking_do_schedule;
    std::condition_variable cv_Blocking_do_schedule;

    int schedule_number = 0;
    std::deque<yf::data::schedule::Job> schedule{};
    std::deque<int>  all_avaiable_schedule_ids;

};


int main(int argc, char *argv[]) {

    do
    {

        std::cout << "Press key '1' to continue..." << std::endl;;
    } while (std::cin.get() != '1');

    // glog
    //
    // Start Logging...
    FLAGS_alsologtostderr = true;
    FLAGS_log_dir = "C:/Dev/w0303/Arm_Control_Module_v1.0/logs";
    google::InitGoogleLogging(argv[0]);

    // Todo:
    //  (done)1.  Establish connection with Arm (Network).    --- ipc_server -- arm_connection_id
    //  (done)2.  Establish connection with Arm (ListenNode). --- ipc_client
    //  3.  Establish connection with MiR100.           --- ipc_client2?
    //  4.  Establish connection with IPC2.             --- ipc_server -- ipc2_connection_id

    // Todo:
    //  (important)1. optimize the status management sequence.
    LOG(INFO) <<  "Startup nw_sys...";

    nw_sys nw_sys(12345);
    nw_sys.StartupIPCServer(); // startup ipc server and then block, waiting for arm connection.

    // Todo: Wait for incoming schedule.
    //  1. parse each schedule.
    //  2. assign each task.
    //  3. assign each subtask.
    // simulate waiting Job from database.

    // (1) startup the database module
    // SQL Server
    //
    while(true)
    {
        // wait for new available schedules.
        yf::sql::sql_server sql;
        sql.WaitAvailableSchedules();

        // do schedule
        std::deque<int> q_schedules_id = sql.GetSchedulesId();

        int id = q_schedules_id.front();
        q_schedules_id.pop_front();

        if(id == 1 || id == 4)
        {
            // get current id execute time..
            sql.GetScheduleExecTime(id);
            std::string execute_time = sql.get_execute_time();

            nw_sys.WaitForExecute(execute_time);

            // assgin the task based on database information
            std::string arm_demo_str = "Demo1";

            sql.UpdateScheduleData(id,2);
            sql.UpdateScheduleLog(id, 2);

            nw_sys.PublishArmTask(arm_demo_str);
        }

        // base on mission result update the database.
        // update and record the schedule
        sql.UpdateScheduleData(id,3);
        sql.UpdateScheduleLog(id, 3);

        std::cout << "wait 15s..." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(15000));
    }



    std::cout << "wait Schedules..." << std::endl;


    nw_sys.WaitSchedules();
    nw_sys.AssignJobs();


    std::this_thread::sleep_for(std::chrono::milliseconds(15000));

    // Todo: Handle each schedule.
    //  1. location time
    //  2. wait until start time.
    //  (3) Handle each task
    //      a.

    // Todo: For Each Arm Task
    //
    // task 1
    std::string arm_command_str = "EnterListenNode";

    nw_sys.CheckArmInitMssionStaus();
    nw_sys.AssignArmMission(arm_command_str);
    nw_sys.UpdateArmCurMissionStaus();

    // task 2
    arm_command_str = "Move Home Position";

    nw_sys.CheckArmInitMssionStaus();
    nw_sys.AssignArmMission(arm_command_str);
    nw_sys.UpdateArmCurMissionStaus();

#if 0  // a specific robot arm task. --- "Move Home Position"
    nw_sys.UpdateArmCurMissionStaus();
    if (nw_sys.GetArmStatus() == yf::common::MissionStatus::Idle)
    {
        nw_sys.NetMessageArm("Move Home Position");

        //wait for arm finish.
        nw_sys.WaitArmReturnFinish();
    }

    // a specific robot arm task. --- "EnterListenNode"


    // Finish all schedule, should return to wait...
    std::cout << "wait 15s..." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(15000));

    nw_sys.CloseIPCServer();
#endif

    std::cout << "wait 10s..." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(10000));

    //todo: how to shut down the program properly?
    // 1. close sever.
    // 2. close glog.
    // 3. close clients?
    std::cout << "program shut down anyway...." << std::endl;

    nw_sys.Close();

    return 0;
}

// TODO:
//  1. try to manage each status properly
//      a. arm connection status
//      b. arm mission status
//      c. arm ln connection status
//  2. COUT ALL OF THEM!!

