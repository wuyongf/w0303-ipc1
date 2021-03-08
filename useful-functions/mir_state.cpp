// 2021-03-08

// mir state testing

#include <iostream>

#include "../include/ugv.h"

int main()
{
    yf::ugv::mir mir100;

    mir100.Start("192.168.2.111");

    std::cout << "IsConnected? " << mir100.IsConnected() << std::endl;

    std::cout << "----------------------" << std::endl;

    Poco::JSON::Object Register;
    float value = 2014.1f;
    Register.set("value", value);

    Poco::JSON::Object Register1;
    float value1 = 2025.9f;
    Register1.set("value", value1);
    Register1.set("label", "string");

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