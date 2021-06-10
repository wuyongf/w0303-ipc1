#include "../include/nw_sys.h"

int main(int argc, char *argv[])
{
    // GLOG
    // Start Logging...
    FLAGS_alsologtostderr = true;

    //FLAGS_log_dir = "C:\\Dev\\w0303\\Arm_Control_Module_v1.0";
    FLAGS_log_dir = "C:\\dev\\w0303-ipc1\\logs";

    google::InitGoogleLogging(argv[0]);

    // NW_SYS Start!
    yf::sys::nw_sys nw_sys(12345);
    nw_sys.Start();

    // Sleep...
    std::cout << "sleep 600000000s... " << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(600000000));

    // NW_SYS Shutdown!
    std::cout << "sys shut down... " << std::endl;
    nw_sys.Close();

    return 1;
}
