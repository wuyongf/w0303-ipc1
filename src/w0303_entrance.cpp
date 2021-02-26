#include <thread>

#include "../include/net.h"
#include "../include/net_w0303_server.h"

//#include "../include/manager.h"
#include "../include/data.h"
#include "../include/manager_status.h"
#include "../include/manager_arm.h"

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

yf::common::MissionStatus UpdateArmStatus(IPCServer& ipc_server)
{
    yf::net::message<CustomMsgTypes> msg;
    std::string str = "Get Status";

    msg.body.resize(str.size());
    msg.body.assign(str.begin(),str.end());

    ipc_server.MessageClient(ipc_server.GetArmClient(), msg);

    return ipc_server.GetArmMissionStatus();
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

    //update arm status
    auto arm_status = UpdateArmStatus(ipc_server);

    std::cout<< "arm_status: " << arm_status << std::endl;

    if (arm_status == yf::common::MissionStatus::Idle)
    {
        std::cout << "arm is idle!" << std::endl;
    }


    // update the status???

    // block... wait time and schedule...

    // pretend some work... 5s


    std::this_thread::sleep_for(std::chrono::milliseconds(500000));

    //TODO: startup Arm manager. try to connect the arm, and then the arm manager should keep waiting.
//    yf::manager::arm_manager tm5;


	//TODO: startup status manager. check status
	yf::manager::sys_status_manager sys_status_manager;

	std::cout << "arm connection status: " << sys_status_manager.GetArmConnectionStatus() << std::endl;


	// todo: Safety stop server thread.
    sever_flag = false;
    t1.join();
}

int main() {
 
	do
	{
		std::cout << "Press key '1' to continue..." << std::endl;;
	} while (std::cin.get() != '1');

	NwSysStartup();

	
	return 0;
}

