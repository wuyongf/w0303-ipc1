#include "../include/nw_sys.h"

int main(int argc, char *argv[])
{
    // glog
    //
    // Start Logging...
    FLAGS_alsologtostderr = true;
    FLAGS_log_dir = "C:/Dev/w0303/Arm_Control_Module_v1.0/logs";
    google::InitGoogleLogging(argv[0]);

    yf::sys::nw_sys nw_sys(12345);

    nw_sys.Start();

//    std::cout << "sleep 10s... " << std::endl;
//    std::this_thread::sleep_for(std::chrono::milliseconds(10000));

    std::cout << "sleep 3000s... " << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(3000000));

    std::cout << "sys shut down... " << std::endl;
    nw_sys.Close();

    return 1;
}