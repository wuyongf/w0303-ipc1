// latest update: 2021/03/12 23:02

#pragma once

#include "nw_status.h"
#include "sql.h"


#include <stdio.h>
#include <time.h>
#include <iostream>
#include <string>
#include <map>

// for time control
#include <chrono>
#include <thread>

// Poco
#include "Poco/Net/HTTPClientSession.h"
#include "Poco/Net/HTTPRequest.h"
#include "Poco/Net/HTTPResponse.h"
#include "Poco/Net/HTTPCredentials.h"
#include "Poco/StreamCopier.h"
#include "Poco/NullStream.h"
#include "Poco/Path.h"
#include "Poco/URI.h"
#include "Poco/Exception.h"

#include "Poco/Net/HTTPMessage.h"
#include "Poco/Net/NetException.h"
#include "Poco/Net/HTMLForm.h"
#include "Poco/Net/ICMPClient.h"

#include "Poco/JSON/JSON.h"
#include "Poco/JSON/Stringifier.h"
#include "Poco/JSON/Object.h"
#include "Poco/Dynamic/Var.h"
#include "Poco/JSON/Parser.h"
#include "Poco/JSON/PrintHandler.h"

using Poco::Net::HTTPClientSession;
using Poco::Net::HTTPRequest;
using Poco::Net::HTTPResponse;
using Poco::Net::HTTPMessage;
using Poco::Net::ICMPClient;
using Poco::StreamCopier;
using Poco::Path;
using Poco::URI;
using Poco::Exception;

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
static const std::string slash="\\";
#else
static const std::string slash="/";
#endif

namespace yf
{
    namespace ugv
    {
        class mir
        {
        public:
            mir(){};
            virtual ~mir(){};

        public:

            /// Setter
            void SetIpAddress(const std::string& ip_addr);
            void SetUriAddress();
            void SetAuthentication(const std::string& auth_info);

            /// Getter();
            std::string GetIpAddress();
            std::string GetUriAddress();
            std::string GetAuthentication();

            int GetRespondStatus();
            std::string GetRespondReason();

            /// Important.
            std::string GetRequestResult();

            /// Methods

            bool GetMethod(const std::string& sub_path);
            bool PutMethod(const std::string& sub_path,const Poco::JSON::Object& obj);
            bool PostMethod(const std::string& sub_path, const Poco::JSON::Object& obj);
            bool DeleteMethod();

        public:

            void Start(const std::string& ip_addr, std::shared_ptr<yf::status::nw_status> status_ptr, std::shared_ptr<yf::sql::sql_server> sql_ptr);
            void Close();

        public: /// layer 1: basic GET/POST/PUT/Delete Method

            // (1) done: Get Connection Status
            // (2) todo: update to nw_status
            // (3) todo: update to database
            bool IsConnected();

            // (1) done: Get State
            // (2) todo: update to nw_status
            // (3) todo: update to database
            int  GetState();

            bool PostActions();
            bool PostActionSetPLC(const int& plc_register, const float& value, const std::string& mission_id, const int& priority);
            bool PostActionWaitPLC(const int& plc_register, const float& value, const std::string& mission_id, const int& priority);

        public: ///  layer2: interaction with nw_status, database

            // (1) todo: update to nw_status
            // (2) todo: update to database
            void UpdateUgvCurState();

            // (1) check whether
            bool ClearErrorState();

            // (1) For Different Areas Initialization
            //
            bool UpdatePositionOnMap(const float& pos_x, const float& pos_y, const float& pos_theta);

        public: /// layer3: create a mission

            ///3.1 based on model_config_id, mission_order, position_name, create a ugv mission.

            // name protocal:
            // building_floor_ModelName_time
            // 12w_2f_handrail_001_202104061026
            bool PostMission(const int& model_config_id);


        private:

            bool doRequest(Poco::Net::HTTPClientSession& session, Poco::Net::HTTPRequest& request, Poco::Net::HTTPResponse& response);

        private:

            // shared status
            std::shared_ptr<yf::status::nw_status> nw_status_ptr_;

            // shared database
            std::shared_ptr<yf::sql::sql_server> sql_ptr_;

            // for model info
            std::string model_name_ = "mir100";

            // for rest api
            std::string ip_address_     = "192.168.2.113";
            std::string uri_address_    = "http://" + ip_address_;

            URI uri_;

            std::string api_content_type_               = "application/json";
            std::string api_credentials_scheme_         = "Basic";
            std::string api_credentials_authentication_ = "ZGlzdHJpYnV0b3I6NjJmMmYwZjFlZmYxMGQzMTUyYzk1ZjZmMDU5NjU3NmU0ODJiYjhlNDQ4MDY0MzNmNGNmOTI5NzkyODM0YjAxNA==";

            // for rest api result
            std::string request_result_;

            int         respond_status_;
            std::string respond_reason_;

            // for mir missions
            std::string session_guid_HKSTP_ = "7aa0de9c-8579-11eb-9840-00012978eb45";
            std::string session_guid_EMSD_  = "7f708490-8579-11eb-9840-00012978eb45";
            std::string session_guid_HA_    = "84c07703-8579-11eb-9840-00012978eb45";

            std::string group_id_           = "mirconst-guid-0000-0011-missiongroup";   // default group: mission group
            bool hidden_flag_    = false;

            std::string cur_mission_name_;

        };
    }
}

std::string yf::ugv::mir::GetIpAddress()
{
    return ip_address_;
}

std::string yf::ugv::mir::GetUriAddress()
{
    return uri_address_;
}

void yf::ugv::mir::SetIpAddress(const std::string& ip_addr)
{
    ip_address_ = ip_addr;
    return;
}

void yf::ugv::mir::SetUriAddress()
{
    uri_address_ = "http://" + ip_address_;
    return;
}

std::string yf::ugv::mir::GetAuthentication()
{
    return api_credentials_authentication_;
}

void yf::ugv::mir::SetAuthentication(const std::string &auth_info)
{
    api_credentials_authentication_ = auth_info;
    return;
}

std::string yf::ugv::mir::GetRequestResult()
{
//    auto result = request_result_;
//    request_result_.clear();
//    return result;

    return  request_result_;
}

bool yf::ugv::mir::doRequest(HTTPClientSession &session,
                             HTTPRequest &request,
                             HTTPResponse &response)
{
//    request_result_.clear();

    session.sendRequest(request);
    std::istream& rs = session.receiveResponse(response);

    respond_status_ = response.getStatus();
    respond_reason_ = response.getReason();

    std::cout << respond_status_ << " " << respond_reason_ << std::endl;

    if (response.getStatus() != Poco::Net::HTTPResponse::HTTP_UNAUTHORIZED)
    {
        request_result_.clear();

        StreamCopier::copyToString(rs, request_result_);
        std::cout << "result: " << request_result_ << std::endl;
        return true;
    }
    else
    {
        Poco::NullOutputStream null;
        StreamCopier::copyStream(rs, null);
        return false;
    }
}

bool yf::ugv::mir::GetMethod(const std::string &sub_path)
{
    try
    {
        uri_.setPath(sub_path);

        std::string path(uri_.getPathAndQuery());
        if (path.empty()) path = "/";

        HTTPClientSession session(uri_.getHost(), uri_.getPort());
        HTTPRequest request(HTTPRequest::HTTP_GET, path, HTTPMessage::HTTP_1_1);
        request.setContentType(api_content_type_);
        request.setCredentials(api_credentials_scheme_, api_credentials_authentication_);

        HTTPResponse response;

        doRequest(session, request, response);
    }
    catch (Exception& exc)
    {
        std::cerr << exc.displayText() << std::endl;
        return false;
    }
}

void yf::ugv::mir::Start(const std::string& ip_addr, std::shared_ptr<yf::status::nw_status> status_ptr, std::shared_ptr<yf::sql::sql_server> sql_ptr)
{

    /// Initial setup for REST API
    //
    SetIpAddress(ip_addr);
    SetUriAddress();

    URI uri_origin(uri_address_);
    uri_ = uri_origin;

    /// Initial setup for SQL
    //
    nw_status_ptr_  = status_ptr;
    sql_ptr_ = sql_ptr;

    return;

}

bool yf::ugv::mir::PutMethod(const std::string &sub_path,const Poco::JSON::Object& obj)
{
    try
    {
        // TODO: prepare session.

        HTTPClientSession session(uri_.getHost(), uri_.getPort());
        session.setKeepAlive(true);
        // TODO: prepare path.
        uri_.setPath(sub_path);
        std::string path(uri_.getPathAndQuery());
        if (path.empty()) path = "/";

        // TODO: send request.

        HTTPRequest req(HTTPRequest::HTTP_PUT, path, HTTPMessage::HTTP_1_1);

        req.setChunkedTransferEncoding(false);
        req.setContentType(api_content_type_);
        req.setCredentials(api_credentials_scheme_, api_credentials_authentication_);

        // TODO: set header here.

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

        respond_status_ = res.getStatus();
        respond_reason_ = res.getReason();

        std::cout << respond_status_ << " " << respond_reason_ << std::endl;   // 200 OK.

        std::istream& is = session.receiveResponse(res);

        request_result_.clear();
        StreamCopier::copyToString(is, request_result_);
        std::cout << request_result_ << std::endl;

        return true;
    }

    catch (Poco::Net::NetException& ex)
    {
        auto result = ex.displayText();
        std::cout << "result: " << result;

        return false;
    }
}

bool yf::ugv::mir::PostMethod(const std::string &sub_path, const Poco::JSON::Object &obj)
{
    try
    {
        // TODO: prepare session.

        HTTPClientSession session(uri_.getHost(), uri_.getPort());
        session.setKeepAlive(true);
        // TODO: prepare path.
        uri_.setPath(sub_path);
        std::string path(uri_.getPathAndQuery());
        if (path.empty()) path = "/";

        // TODO: send request.

        HTTPRequest req(HTTPRequest::HTTP_POST, path, HTTPMessage::HTTP_1_1);

        req.setChunkedTransferEncoding(false);
        req.setContentType(api_content_type_);
        req.setCredentials(api_credentials_scheme_, api_credentials_authentication_);

        // TODO: set header here.

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

        respond_status_ = res.getStatus();
        respond_reason_ = res.getReason();

        std::cout << respond_status_ << " " << respond_reason_ << std::endl;   // 200 OK.

        std::istream& is = session.receiveResponse(res);

        request_result_.clear();
        StreamCopier::copyToString(is, request_result_);
        std::cout << request_result_ << std::endl;

        return true;
    }

    catch (Poco::Net::NetException& ex)
    {
        auto result = ex.displayText();
        std::cout << "result: " << result;

        return false;
    }
}

int yf::ugv::mir::GetRespondStatus()
{

    return respond_status_;
}

std::string yf::ugv::mir::GetRespondReason()
{

    return respond_reason_;
}

bool yf::ugv::mir::UpdatePositionOnMap(const float &pos_x, const float &pos_y, const float &pos_theta)
{
    Poco::JSON::Object Position;

    Poco::JSON::Object pos_json;

    pos_json.set("x",pos_x);
    pos_json.set("y",pos_y);
    pos_json.set("orientation",pos_theta);

    Position.set("position", pos_json);

    return PutMethod("/api/v2.0.0/status", Position);
}

int yf::ugv::mir::GetState()
{
    GetMethod("/api/v2.0.0/status");

    auto json_result = GetRequestResult();

    Poco::JSON::Parser parser;

    Poco::Dynamic::Var result = parser.parse(json_result);

    Poco::JSON::Object::Ptr object = result.extract<Poco::JSON::Object::Ptr>();

    int state = object->getValue<int>("state_id");

    return state;
}

//todo: fine tune......
//  (1) cout problem...
//  (2) database?

bool yf::ugv::mir::IsConnected()
{
    std::string ping_command;

    // check whether it is on Windows or Linux

    if(slash == "/")    // linux
    {
        //ping -c1 -s1 -w1 192.168.2.113
        ping_command = "ping -c1 -s1 -w1 " + ip_address_;
    }
    else                // windows
    {
        //By using win ping method.

        int ping_request_no = 1;

        int ping_timeout = 800; //ms

        ping_command =  "ping " + ip_address_ +
                        " -n " + std::to_string(ping_request_no) +
                        " -w " + std::to_string(ping_timeout);
    }

    const char* cstr_ping_command = ping_command.c_str();

    if (system( cstr_ping_command ) )
    {
        return false;
    }
    else
    {
        return true;
    }
}

bool yf::ugv::mir::PostMission(const int& model_config_id)
{
    /// based on model_config_id, mission_order, position_name, create a ugv mission.

    // name protocol:
    // building_floor_ModelName_time
    // 12w_2f_handrail_001_202104061026
    // 12w_2f_handrail_001_20210406_1020

    /// 1. get positions.
    auto position_names = sql_ptr_->GetUgvMissionConfigPositionNames(model_config_id);

    /// 2. get session.
    auto session_info = sql_ptr_->GetSiteInfo(model_config_id);

    /// 3. get building---floor info--model name---time
    auto building_info = sql_ptr_->GetBuildingInfo(model_config_id);
    auto floor_info = sql_ptr_->GetFloorInfo(model_config_id);
    auto model_name = sql_ptr_->GetModelName(model_config_id);

    // time
    sql_ptr_->UpdateTime();
    auto time_year = sql_ptr_->get_time_element("year");
    auto time_month = sql_ptr_->get_time_element("month");
    auto time_day = sql_ptr_->get_time_element("day");
    auto time_hour = sql_ptr_->get_time_element("hour");
    auto time_min = sql_ptr_->get_time_element("min");

    auto time = time_year + time_month + time_day + "_" + time_hour + time_min;

    /// (1) set cur_mission_name
    cur_mission_name_ = building_info + "_" + floor_info + "/F_" + model_name + "_" + time;

    /// (2) get session_id
    std::string session_id;

    if(session_info == "hkstp")
    {
        session_id = session_guid_HKSTP_;
    }
    else if(session_info == "emsd")
    {
        session_id = session_guid_EMSD_;
    }
    else if(session_info == "ha")
    {
        session_id = session_guid_HA_;
    }

    /// (3) mission creation
    //@@ input:
    //   (1) name,       done    12w_2f_handrail_001_20210406_1020
    //   (2) session_id, done    hkstp  guid: 7aa0de9c-8579-11eb-9840-00012978eb45
    //   (3) hidden,     done    false
    //   (4) group_id,   done    Missions guid: mirconst-guid-0000-0011-missiongroup

    Poco::JSON::Object mission;

    mission.set("session_id",session_id);
    mission.set("name",cur_mission_name_);

    mission.set("hidden", hidden_flag_);
    mission.set("group_id", group_id_);

    return PostMethod("/api/v2.0.0/missions", mission);

}

bool yf::ugv::mir::PostActions()
{
    // (1) move to p1            based on mission_order.
    // (2) set plc002 = 1,2,3... based on mission_order.
    // (3) set plc001 = 1.

    // (4) count the priority.

    /// @@ input: current mission_name.
    /// @@ output: actions
    ///
    /// how to post an action?

    // mission_id
    // action_type
    // parameters
    // priority

    /// (1) mission_id:         mission_name: 0326-test ---> mission_guid: 4d3dcb5f-8dd9-11eb-a5e3-00012978eb45
    ///                         mission_name: 12w_2/F_handle_003_20210406_1449 ---> mission_guid: 1fe736a6-96a4-11eb-b10a-00012978eb45
    /// (2) action_type:        wait_for_plc_register, set_plc_register, move

//    PostActionMove(1);
//    PostActionSetPLC(2);
//    PostActionWaitPLC(3);


    return false;
}

bool yf::ugv::mir::PostActionSetPLC(const int &plc_register, const float &value, const std::string& mission_id, const int& priority)
{
    int value_i = static_cast<int>(value);

    if (plc_register <= 100)
    {
        // mission_id (done)
        // action_type (done)
        // parameters (?)  2 set 1  "register" "action" "value"
        // priority (1)

        Poco::JSON::Object action_set_plc;

        Poco::JSON::Object registry_json;
        Poco::JSON::Object action_json;
        Poco::JSON::Object value_json;

        registry_json.set("value",plc_register);
        registry_json.set("id","register");

        action_json.set("value","set");
        action_json.set("id","action");

        value_json.set("value",value_i);
        value_json.set("id","value");

        Poco::JSON::Array parameters_array;
        parameters_array.set(0,registry_json);
        parameters_array.set(1,action_json);
        parameters_array.set(2,value_json);

        action_set_plc.set("parameters", parameters_array);
        action_set_plc.set("priority",priority);
        action_set_plc.set("mission_id", mission_id);
        action_set_plc.set("action_type", "set_plc_register");

        /// (4) fine tune the sub_path

        std::string sub_path = "/api/v2.0.0/missions/" + mission_id + "/actions";

        return PostMethod(sub_path, action_set_plc);
    }
    else
    {
        // mission_id (done)
        // action_type (done)
        // parameters (?)  2 set 1  "register" "action" "value"
        // priority (1)


        std::string action_type =  "set_plc_register";

        Poco::JSON::Object action_set_plc;

        Poco::JSON::Object registry_json;
        Poco::JSON::Object action_json;
        Poco::JSON::Object value_json;

        registry_json.set("value",plc_register);
        registry_json.set("id","register");

        action_json.set("value","set");
        action_json.set("id","action");

        value_json.set("value",value);
        value_json.set("id","value");

        Poco::JSON::Array parameters_array;
        parameters_array.set(0,registry_json);
        parameters_array.set(1,action_json);
        parameters_array.set(2,value_json);

        action_set_plc.set("parameters", parameters_array);
        action_set_plc.set("priority",priority);
        action_set_plc.set("mission_id", mission_id);
        action_set_plc.set("action_type", "set_plc_register");

        /// (4) fine tune the sub_path

        std::string sub_path = "/api/v2.0.0/missions/" + mission_id + "/actions";

        return PostMethod(sub_path, action_set_plc);
    }

}

bool yf::ugv::mir::PostActionWaitPLC(const int &plc_register, const float &value, const std::string &mission_id,
                                     const int &priority)
{
    int value_i = static_cast<int>(value);

    if (plc_register <= 100)
    {
        // mission_id (done)
        // action_type (done)
        // parameters (?)  2 set 1  "register" "action" "value"
        // priority (1)

        Poco::JSON::Object action_set_plc;

        Poco::JSON::Object registry_json;
        Poco::JSON::Object value_json;
        Poco::JSON::Object timeout_json;

        registry_json.set("value",plc_register);
        registry_json.set("id","register");

        value_json.set("value",value_i);
        value_json.set("id","value");

        timeout_json.set("value", {});
        timeout_json.set("id","timeout");

        Poco::JSON::Array parameters_array;
        parameters_array.set(0,registry_json);
        parameters_array.set(1,value_json);
        parameters_array.set(2,timeout_json);

        action_set_plc.set("parameters", parameters_array);
        action_set_plc.set("priority",priority);
        action_set_plc.set("mission_id", mission_id);
        action_set_plc.set("action_type", "wait_for_plc_register");

        /// (4) fine tune the sub_path

        std::string sub_path = "/api/v2.0.0/missions/" + mission_id + "/actions";

        return PostMethod(sub_path, action_set_plc);
    }
    else
    {

    }

}

// post mission actions
//
// @@ input: mission_id
//
// mission name: 12w_2/F_corridor_handrail_001
// guid: fa1868c9-8dd8-11eb-a5e3-00012978eb45
//

// 1. Get map_id based on ugv_config_id.

// 2. Get position_name in order.

// 3. Get position guid in order.

// 4. post a list of new actions.























