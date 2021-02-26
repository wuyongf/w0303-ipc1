#include <glog/logging.h>

using namespace std;

int main(int /*argc*/, char** argv)
{
    FLAGS_alsologtostderr = true;
    FLAGS_log_dir = "C:/Dev/w0303/Arm_Control_Module_v1.0/logs/";

    google::InitGoogleLogging(argv[0]);

    LOG(INFO) << "This is INFO";
    LOG(WARNING) << "This is WARNING";
    LOG(ERROR) << "This is Error";

    google::ShutdownGoogleLogging();

    system("pause");


    return 0;
}
