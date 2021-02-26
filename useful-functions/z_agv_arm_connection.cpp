#include <iostream>
#include <string>


#include "Poco/Net/HTTPClientSession.h"
#include "Poco/Net/HTTPRequest.h"
#include "Poco/Net/HTTPResponse.h"
#include "Poco/StreamCopier.h"
#include "Poco/Path.h"
#include "Poco/Net/HTTPMessage.h"
#include "Poco/Net/NetException.h"
#include "Poco/Net/HTMLForm.h"
#include "Poco/URI.h"
#include "Poco/JSON/JSON.h"
#include "Poco/JSON/Stringifier.h"
#include "Poco/JSON/Object.h"
#include "Poco/Dynamic/Var.h"
#include "Poco/JSON/Parser.h"

// for time control.
#include <chrono>
#include <thread>

// for connection with arm
#include "../include/tcpip_arm_connection_lib/ServerSocket.h"
#include "../include/tcpip_arm_connection_lib/SocketException.h"

using Poco::URI;
using Poco::Net::HTTPClientSession;
using Poco::Net::HTTPRequest;
using Poco::Net::HTTPResponse;
using Poco::Net::HTTPMessage;
using Poco::Net::HTMLForm;
using Poco::Net::NetException;
using Poco::StreamCopier;
using Poco::JSON::Stringifier;
using Poco::JSON::Object;
using Poco::JSON::Array;
using Poco::JSON::Parser;
using Poco::Dynamic::Var;

using std::cout;
using std::string;
using std::istream;
using std::iostream;

string get_key_value(string& post_result, string& key) {

    Parser parser;
    Var result = parser.parse(post_result);
    Object::Ptr object1 = result.extract<Object::Ptr>(); //
    Var test1 = object1->get(key); // test1 holds { "value" }
    std::string val1 = test1.toString(); // val1 holds "value"
    return  val1;
}

string Request_Get(URI& url, const string& sub_path) {
    try {

        // TODO: prepare session.

        HTTPClientSession session(url.getHost(), url.getPort());

        // TODO: prepare path.
        url.setPath(sub_path);
        std::string path(url.getPathAndQuery());
        if (path.empty()) path = "/";

        // TODO: send request.

        HTTPRequest req(HTTPRequest::HTTP_GET, path, HTTPMessage::HTTP_1_1);
        req.setContentType("application/json");

        // TODO: set header here. For trans200, there is no need to set header.
        //

        // TODO: set request body, skipped.
        //

        // TODO: sends request, returns open stream.

        std::ostream& os = session.sendRequest(req);
        //        req.write(std::cout); // print out the

                // TODO: get response.

        HTTPResponse res;
        std::cout << "GET_Status: " << res.getStatus() << " " << res.getReason() << std::endl;

        istream& is = session.receiveResponse(res);
        std::string result;
        StreamCopier::copyToString(is, result);
        std::cout << result << "\n";

        return  result;

    }
    catch (NetException& ex) {
        auto result = ex.displayText();
        std::cout << "result: " << result;
        return "";
    }
}

string Request_Post(URI& url, const string& sub_path, const Object& obj) {
    try {
        // TODO: prepare session.

        HTTPClientSession session(url.getHost(), url.getPort());
        session.setKeepAlive(true);
        // TODO: prepare path.
        url.setPath(sub_path);
        std::string path(url.getPathAndQuery());
        if (path.empty()) path = "/";

        // TODO: send request.

        HTTPRequest req(HTTPRequest::HTTP_POST, path, HTTPMessage::HTTP_1_1);

        req.setChunkedTransferEncoding(false);
        req.setContentType("application/json");

        // TODO: set header here. For trans200, there is no need to set header.
        //

        // TODO: set request body.  obj

        std::ostringstream ss;
        obj.stringify(ss);
        std::string body;
        body = ss.str();
        req.setContentLength(body.length());

        // TODO: sends request, returns open stream.

        session.sendRequest(req) << body;

        //        req.write(std::cout); // print out request

                // TODO: get response.

        HTTPResponse res;
        std::cout << "POST_Status: " << res.getStatus() << " " << res.getReason() << std::endl;   // 200 OK.

        istream& is = session.receiveResponse(res);
        std::string result;
        StreamCopier::copyToString(is, result);
        std::cout << result << "\n";

        return  result;
    }

    catch (NetException& ex) {
        auto result = ex.displayText();
        std::cout << "result: " << result;
        return "";
    }
}

string get_status(URI& url, string& sub_path2, string& mission_x_contents)
{
    string key_id = "id";
    string id_val = get_key_value(mission_x_contents, key_id);             // Parse string function.
    string sub_path2_1 = sub_path2 + "/" + id_val;

    string missionWorks_contents = Request_Get(url, sub_path2_1);
    string key_status("status");
    string missionWorks_STATUS_val = get_key_value(missionWorks_contents, key_status);
    return missionWorks_STATUS_val;
}

void thread_assign_robot_task(const int& port, string& data, string& robot_flag, Poco::JSON::Object& mission_x)
{
    std::cout << "running....\n";

    URI url("http://192.168.1.127:8080");
    string sub_path2 = "/api/v2/missionWorks";

    try
    {
        // Create the socket
        ServerSocket server(port);

        //true int a =1
        while (true)
        {

            ServerSocket new_sock;
            server.accept(new_sock);
            cout << "new socket created!" << "\n";
            try
            {
                while (true)
                {
                    if (robot_flag == "0") {
                        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                    }

                    if (robot_flag == "1") {
                        //todo: assgin task for AGV.
                        string mission_1_contents = Request_Post(url, sub_path2, mission_x); // string2 { "property" : "value" }

                        std::this_thread::sleep_for(std::chrono::milliseconds(500));  // wait 500ms

                        string STATUS = get_status(url, sub_path2, mission_1_contents);  //  get the STATUS, get_Trans200_Status

                        //todo: rog waits for AGV finish.

                        while (STATUS != "SUCCESS") {

                            std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // wait 1s

                            if (STATUS == "FAULT") {
                                cout << "AGV missionWork FAULT!" << "\n";
                            }

                            if (STATUS == "RUNNING") {
                                cout << "AGV is running!" << "\n";
                            }

                            // update the status
                            STATUS = get_status(url, sub_path2, mission_1_contents);
                        }
                        cout << "AGV has arrived!" << "\n";
                        //todo: return waiting loop.
                        robot_flag = "0";
                    }

                    if (robot_flag == "2") {
                        std::this_thread::sleep_for(std::chrono::milliseconds(200)); // wait 200ms
                        new_sock << data;
                        if (data != "robot_task_done") {
                            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                            new_sock >> data;
                        }
                        robot_flag = "0";
                    }

                    std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // wait 1s
                }
            }
            catch (SocketException&) {}

        }
    }
    catch (SocketException& e)
    {
        std::cout << "Exception was caught:" << e.description() << "\nExiting.\n";
    }

}

int main(int argc, char**)
{
    std::cout << "program starting....\n";

    string robot_flag("0");
    string data("empty");

    Poco::JSON::Object mission_x;

    // thread_assign_robot_task
    std::thread t1(thread_assign_robot_task, 12345, std::ref(data), std::ref(robot_flag), std::ref(mission_x));

    //todo: assign task to AGV

    mission_x.set("missionId", "90ab25d1-ac63-11ea-bd59-0242ac110002");
    mission_x.set("agvId", "602126a7-5e08-11ea-8e73-0242ac110002");

    robot_flag = "1";

    if (robot_flag != "0") {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    //todo: assign task to Arm

    data = "task_1";

    robot_flag = "2";

    if (robot_flag != "0") {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

}