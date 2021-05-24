#pragma once

#include "nw_status.h"
#include "sql.h"

#include <stdio.h>
#include <time.h>
#include <iostream>
#include <string>
#include <map>

// time control
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

// For determining WIN or Linux
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

            /// Getter
            std::string GetIpAddress();
            std::string GetUriAddress();
            std::string GetAuthentication();

            int GetRespondStatus();
            std::string GetRespondReason();

            /// Important.
            // todo: comment the cout
            std::string GetRequestResult();

            /// Basic Methods

            bool GetMethod(const std::string& sub_path);
            bool PutMethod(const std::string& sub_path,const Poco::JSON::Object& obj);
            bool PostMethod(const std::string& sub_path, const Poco::JSON::Object& obj);
            bool DeleteMethod(const std::string& sub_path);

        public:

            void Start(const std::string& ip_addr, std::shared_ptr<yf::status::nw_status> status_ptr, std::shared_ptr<yf::sql::sql_server> sql_ptr);
            void Close();

        public: /// layer 1: basic GET/POST/PUT/Delete Method

            // (1) done: Get Connection Status
            // (2) todo: update to nw_status
            // (3) todo: update to database
            bool IsConnected();

            int GetPLCRegisterIntValue(const int& plc_register);
            float GetPLCRegisterFloatValue(const int& plc_register);

            bool SetPLCRegisterIntValue(const int& plc_register, const int& value);
            bool SetPLCRegisterfloatValue(const int& plc_register, const float& value);


        public: ///  layer2: interaction with nw_status, database

            // methods for ipc1
            //
            // (1) check mission continue flag
            bool MissionStatusCheck(const int& timeout_min);
            
            // (2) current ugv mission state
            void RetrieveUgvCurrentMissionState();

            // a. Initial status check for mission
            bool InitialStatusCheckForMission(const int& timeout_min);

            // (1) todo: update to nw_status
            // (2) todo: update to database
            void UpdateUgvCurState();

            // (1) check whether
            bool ClearErrorState();

            // (1) For Different Areas Initialization
            //
            bool UpdatePositionOnMap(const float& pos_x, const float& pos_y, const float& pos_theta);

        public: /// layer3: create a mission --- REST API

            ///3.1 based on model_config_id, mission_order, position_name, create a ugv mission.

            // name protocal:
            // building_floor_ModelName_time
            // 12w_2f_handrail_001_202104061026
            bool PostMission(const int& model_config_id);
            void PostActions(const int& model_config_id);

            bool PostActionSetPLC(const int& plc_register, const float& value, const std::string& mission_guid, const int& priority);
            bool PostActionWaitPLC(const int& plc_register, const float& value, const std::string& mission_guid, const int& priority);
            bool PostActionMove(const std::string& position_guid, const std::string &mission_guid, const int &priority);

            std::string GetPositionGUID(const std::string& map_guid, const std::string& position_name);
            std::string GetCurMissionGUID();
            std::string GetMapGUID(const std::string& map_name);

            std::deque<std::string> GetSessionNameList();
            std::deque<std::string> GetMapNameList(const std::string& session_name);
            std::deque<std::string> GetPositionNameList(const std::string& session_name, const std::string& map_name);


        private:
            std::string GetSessionGUID(const std::string& session_name);
            std::string GetMapGUID(const std::string& session_name, const std::string& map_name);

        public:
            // MiR Mission Control
            bool Play();
            bool Pause();

            bool PostMissionQueue(const std::string& mission_guid);
            bool DeleteMissionQueue();

            //
            bool ChangeMap(const std::string& map_name);
            bool ChangeMapByDBMapStatus();

            //
            bool ChangeInitPositionByDBMapStatus();

        public:
            // Getter: MiR Properties
            int     GetState();
            float   GetBatteryPercentage();
            std::vector<float> GetCurPosition();

        private:

            bool doRequest(Poco::Net::HTTPClientSession& session, Poco::Net::HTTPRequest& request, Poco::Net::HTTPResponse& response);

        private:

            // shared status
            std::shared_ptr<yf::status::nw_status> nw_status_ptr_;

            // shared database
            std::shared_ptr<yf::sql::sql_server> sql_ptr_;

            // ugv info
            std::string model_name_ = "mir100";

            // for rest api
            std::string ip_address_     = "192.168.7.34";
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
            bool        hidden_flag_        = false;

            std::string cur_mission_name_;
            std::string cur_session_guid_;
            std::deque<std::string> cur_position_names_;

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
        //1. prepare session.

        HTTPClientSession session(uri_.getHost(), uri_.getPort());
        session.setKeepAlive(true);
        //2. prepare path.
        uri_.setPath(sub_path);
        std::string path(uri_.getPathAndQuery());
        if (path.empty()) path = "/";

        //3. send request.

        HTTPRequest req(HTTPRequest::HTTP_POST, path, HTTPMessage::HTTP_1_1);

        req.setChunkedTransferEncoding(false);
        req.setContentType(api_content_type_);
        req.setCredentials(api_credentials_scheme_, api_credentials_authentication_);

        //4. set header here.

        //5. set request body.  obj

        std::ostringstream ss;
        obj.stringify(ss);
        std::string body;
        body = ss.str();
        req.setContentLength(body.length());

        //6. sends request, returns open stream.

        session.sendRequest(req) << body;

        //        req.write(std::cout); // print out request

        //7. get response.

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

    // will store:
    // cur_mission_name_
    // cur_session_guid_

    // name protocol:
    // building_floor_ModelName_time
    // 12w_2f_handrail_001_202104061026
    // 12w_2f_handrail_001_20210406_1020

    /// 1. get positions.
//    auto position_names = sql_ptr_->GetUgvMissionConfigPositionNames(model_config_id);

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
    auto time_min = sql_ptr_->get_time_element("minute");

    auto time = time_year + time_month + time_day + "_" + time_hour + time_min;

    /// (1) set cur_mission_name
    cur_mission_name_ = building_info + "_" + floor_info + "/F_" + model_name + "_" + time;

    /// (2) get session_id

    if(session_info == "hkstp")
    {
        cur_session_guid_ = session_guid_HKSTP_;
    }
    else if(session_info == "emsd")
    {
        cur_session_guid_ = session_guid_EMSD_;
    }
    else if(session_info == "ha")
    {
        cur_session_guid_ = session_guid_HA_;
    }

    /// (3) mission creation
    //@@ input:
    //   (1) name,       done    12w_2f_handrail_001_20210406_1020
    //   (2) session_id, done    hkstp  guid: 7aa0de9c-8579-11eb-9840-00012978eb45
    //   (3) hidden,     done    false
    //   (4) group_id,   done    Missions guid: mirconst-guid-0000-0011-missiongroup

    Poco::JSON::Object mission;

    mission.set("session_id",cur_session_guid_);
    mission.set("name",cur_mission_name_);

    mission.set("hidden", hidden_flag_);
    mission.set("group_id", group_id_);

    return PostMethod("/api/v2.0.0/missions", mission);

}

//@@ input: model_config_id
//@@
//@@ output: a list of actions
void yf::ugv::mir::PostActions(const int& model_config_id)
{
    // 1. get mission_guid from REST
    std::string mission_guid = this->GetCurMissionGUID();

    // 2. get position_names from DB, based on model_config_id
    std::deque<std::string> position_names = sql_ptr_->GetUgvMissionConfigPositionNames(model_config_id);

    // 3.
    // 3.a. get map name from db.
    auto model_id = sql_ptr_->GetModelId(model_config_id);
    auto map_id = sql_ptr_->GetMapIdFromModelId(model_id);
    auto map_name = sql_ptr_->GetMapElement(map_id,"map_name");
    // 3.b. based on map_name, retrieve map_guid from REST.
    std::string map_guid = this->GetMapGUID(map_name);

    /// Actions detail
    ///
    int total_position_num = sql_ptr_->GetUgvMissionConfigNum(model_config_id);
    int priority = 1;

    /// (1) Set Ugv Mission Start Flag
    this->PostActionSetPLC(4,1,mission_guid,priority);
    priority++;

    /// (2) Initialization Stage
    this->PostActionSetPLC(1,0,mission_guid,priority);
    priority++;
    this->PostActionSetPLC(2,0,mission_guid,priority);
    priority++;
    this->PostActionSetPLC(3,0,mission_guid,priority);
    priority++;

    // todo: speed, footprint initialization

    /// (3) For loop, mission details
    //
    // get mission_name from REST. mission_order from db
    for (int mission_count = 1; mission_count <= total_position_num; mission_count++)
    {
        /// a. get position name.
        std::string position_name = position_names[mission_count-1];

        //todo: get position_guid

        /// b. position_guid
        std::string position_guid = this->GetPositionGUID(map_guid,position_name);

        //1. get ugv_mission_config_id, based on mission_count,
        // prepare: mission_config_id
        //
        //@@ input: position_guid, mission_guid(done)
        //
        this->PostActionSetPLC(2,mission_count,mission_guid,priority);
        priority++;

        this->PostActionSetPLC(3,1,mission_guid,priority);
        priority++;

        this->PostActionMove(position_guid, mission_guid, priority);
        priority++;

        this->PostActionSetPLC(1,1,mission_guid,priority);
        priority++;

        this->PostActionWaitPLC(1,0,mission_guid,priority);
        priority++;

    }

    /// (4) Set Ugv Mission Finish Flag
    //
    this->PostActionSetPLC(4,2,mission_guid,priority);
    priority++;
}

bool yf::ugv::mir::PostActionSetPLC(const int &plc_register, const float &value, const std::string& mission_guid, const int& priority)
{
    int value_i = static_cast<int>(value);

    if (plc_register <= 100)
    {
        // mission_guid (done)
        // action_type (done)
        // parameters (?)  2 set 1  "register" "action" "value"
        // priority (1)

        Poco::JSON::Object action_plc_json;

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

        action_plc_json.set("parameters", parameters_array);
        action_plc_json.set("priority", priority);
        action_plc_json.set("mission_id", mission_guid);
        action_plc_json.set("action_type", "set_plc_register");

        /// (4) fine tune the sub_path

        std::string sub_path = "/api/v2.0.0/missions/" + mission_guid + "/actions";

        return PostMethod(sub_path, action_plc_json);
    }
    else
    {
        // mission_id (done)
        // action_type (done)
        // parameters (?)  2 set 1  "register" "action" "value"
        // priority (1)


        std::string action_type =  "set_plc_register";

        Poco::JSON::Object action_plc_json;

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

        action_plc_json.set("parameters", parameters_array);
        action_plc_json.set("priority", priority);
        action_plc_json.set("mission_id", mission_guid);
        action_plc_json.set("action_type", "set_plc_register");

        /// (4) fine tune the sub_path

        std::string sub_path = "/api/v2.0.0/missions/" + mission_guid + "/actions";

        return PostMethod(sub_path, action_plc_json);
    }

}

bool yf::ugv::mir::PostActionWaitPLC(const int &plc_register, const float &value, const std::string &mission_guid,
                                     const int &priority)
{
    int value_i = static_cast<int>(value);

    if (plc_register <= 100)
    {
        // mission_guid (done)
        // action_type (done)
        // parameters (?)  2 waits 1  "register"  "value" "timeout"
        // priority (1)

        Poco::JSON::Object action_plc_json;

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

        action_plc_json.set("parameters", parameters_array);
        action_plc_json.set("priority", priority);
        action_plc_json.set("mission_id", mission_guid);
        action_plc_json.set("action_type", "wait_for_plc_register");

        /// (4) fine tune the sub_path

        std::string sub_path = "/api/v2.0.0/missions/" + mission_guid + "/actions";

        return PostMethod(sub_path, action_plc_json);
    }
    else
    {
        // mission_id (done)
        // action_type (done)
        // parameters (?)  2 waits 1  "register"  "value" "timeout"
        // priority (1)

        Poco::JSON::Object action_plc_json;

        Poco::JSON::Object registry_json;
        Poco::JSON::Object value_json;
        Poco::JSON::Object timeout_json;

        registry_json.set("value",plc_register);
        registry_json.set("id","register");

        value_json.set("value",value);
        value_json.set("id","value");

        timeout_json.set("value", {});
        timeout_json.set("id","timeout");

        Poco::JSON::Array parameters_array;
        parameters_array.set(0,registry_json);
        parameters_array.set(1,value_json);
        parameters_array.set(2,timeout_json);

        action_plc_json.set("parameters", parameters_array);
        action_plc_json.set("priority", priority);
        action_plc_json.set("mission_id", mission_guid);
        action_plc_json.set("action_type", "wait_for_plc_register");

        /// (4) fine tune the sub_path

        std::string sub_path = "/api/v2.0.0/missions/" + mission_guid + "/actions";

        return PostMethod(sub_path, action_plc_json);
    }

}


// position_name.       from DB. [table: data_ugv_mission_config] // mission_order
// map_id.              sql->GetModelId(model_config_id), sql->GetMapId(model_id)
//
// position_guid. this->GetPositionGUID(map_id, position_name);
//
//@@ input: position_guid, mission_id(guid), priority
//
bool yf::ugv::mir::PostActionMove(const std::string& position_guid, const std::string &mission_guid, const int &priority)
{
    // mission_guid (done)
    // action_type (done)  "move"
    // parameters (?)   {position}
    //                  {cart_entry_position}
    //                  {main_or_entry_position}
    //                  {marker_entry_position}
    //                  {retries}
    //                  {distance_threshold}
    // priority (1)
    //
    // {position}: value: “position_guid" id: "position"

    /// prerequisite:
    /// 1. get position guid... based on position_name, map_id

    Poco::JSON::Object action_move_json;

    Poco::JSON::Object position_json;
    Poco::JSON::Object cart_entry_position_json;
    Poco::JSON::Object main_or_entry_position_json;
    Poco::JSON::Object marker_entry_position_json;
    Poco::JSON::Object retries_json;
    Poco::JSON::Object distance_threshold_json;


    position_json.set("value", position_guid);
    position_json.set("id", "position");

    cart_entry_position_json.set("value", "main");
    cart_entry_position_json.set("id", "cart_entry_position");

    main_or_entry_position_json.set("value", "main");
    main_or_entry_position_json.set("id", "main_or_entry_position");

    marker_entry_position_json.set("value", "entry");
    marker_entry_position_json.set("id", "marker_entry_position");

    retries_json.set("value", 10);
    retries_json.set("id", "retries");

    distance_threshold_json.set("value", 0.1);
    distance_threshold_json.set("id", "distance_threshold");

    Poco::JSON::Array parameters_array;
    parameters_array.set(0, position_json);
    parameters_array.set(1, cart_entry_position_json);
    parameters_array.set(2, main_or_entry_position_json);
    parameters_array.set(3, marker_entry_position_json);
    parameters_array.set(4, retries_json);
    parameters_array.set(5, distance_threshold_json);

    action_move_json.set("parameters", parameters_array);
    action_move_json.set("priority", priority);
    action_move_json.set("mission_id", mission_guid);
    action_move_json.set("action_type", "move");

    /// (4) fine tune the sub_path

    std::string sub_path = "/api/v2.0.0/missions/" + mission_guid + "/actions";

    return PostMethod(sub_path, action_move_json);

}

// position naming protocol: object_name_via001
// handrail_001_via001
// handrail_001_via002
// handrail_001_via003
// handrail_001_via004

//@@ input: map_id: cb4cfe79-8dd6-11eb-a5e3-00012978eb45
//          position_name: corridor_handrail_001_via001_front
//
std::string yf::ugv::mir::GetPositionGUID(const std::string &map_guid, const std::string& position_name)
{
    // /api/v2.0.0/maps/cb4cfe79-8dd6-11eb-a5e3-00012978eb45/positions

    /// 0. prepare the position_guid
    std::string position_guid;

    /// 1. fine tune sub_path
    std::string sub_path = "/api/v2.0.0/maps/" + map_guid + "/positions";

    GetMethod(sub_path);

    auto json_result = GetRequestResult();

    Poco::JSON::Parser parser;

    Poco::Dynamic::Var result = parser.parse(json_result);

    Poco::JSON::Array::Ptr  array = result.extract<Poco::JSON::Array::Ptr>();

    bool found_flag = false;

    for (Poco::JSON::Array::ConstIterator it= array->begin(); it != array->end(); ++it)
    {
        if(found_flag == false)
        {
            // iteration, find the position_name here.
            Poco::JSON::Object::Ptr obj = it->extract<Poco::JSON::Object::Ptr>();

            std::string pos_name = obj->getValue<std::string>("name");

            if(pos_name == position_name)
            {
                // 1. set found flag
                found_flag = true;

                // 2. get the position guid.
                position_guid = obj->getValue<std::string>("guid");
            }
        }
        else
        {
            // already found the position
            // do nothing.
            return position_guid;
        }
    }

    if(found_flag == false)
    {
        std::cout << "didn't find the position_name in DB!" << std::endl;
    }

    return position_guid;

}

// must run PostMission() first
std::string yf::ugv::mir::GetCurMissionGUID()
{
    // input
    // cur_mission_name_;
    // cur_session_guid_;

    // output
    std::string mission_guid;

    // method

    /// 1. fine tune sub_path
    std::string sub_path = "/api/v2.0.0/sessions/" + cur_session_guid_ + "/missions";

    GetMethod(sub_path);

    auto json_result = GetRequestResult();

    Poco::JSON::Parser parser;

    Poco::Dynamic::Var result = parser.parse(json_result);

    Poco::JSON::Array::Ptr array = result.extract<Poco::JSON::Array::Ptr>();

    bool found_flag = false;

    for (Poco::JSON::Array::ConstIterator it= array->begin(); it != array->end(); ++it)
    {
        if(found_flag == false)
        {
            // iteration, find the position_name here.
            Poco::JSON::Object::Ptr obj = it->extract<Poco::JSON::Object::Ptr>();

            std::string pos_name = obj->getValue<std::string>("name");

            if(pos_name == cur_mission_name_)
            {
                // 1. set found flag
                found_flag = true;

                // 2. get the position guid.
                mission_guid = obj->getValue<std::string>("guid");
            }
        }
        else
        {
            // already found the position
            // do nothing.
            return mission_guid;
        }
    }

    if(found_flag == false)
    {
        std::cout << "there is not such mission_name in MiR!" << std::endl;
    }

    return mission_guid;

}

std::string yf::ugv::mir::GetMapGUID(const std::string &map_name)
{
    // output
    std::string map_guid;

    // method

    /// 1. fine tune sub_path
    //std::string sub_path = "/api/v2.0.0/sessions/" + cur_session_guid_ + "/maps";
    std::string sub_path = "/api/v2.0.0/maps";

    GetMethod(sub_path);

    auto json_result = GetRequestResult();

    Poco::JSON::Parser parser;

    Poco::Dynamic::Var result = parser.parse(json_result);

    Poco::JSON::Array::Ptr array = result.extract<Poco::JSON::Array::Ptr>();

    bool found_flag = false;

    for (Poco::JSON::Array::ConstIterator it= array->begin(); it != array->end(); ++it)
    {
        if(found_flag == false)
        {
            // iteration, find the position_name here.
            Poco::JSON::Object::Ptr obj = it->extract<Poco::JSON::Object::Ptr>();

            std::string name = obj->getValue<std::string>("name");

            if(name == map_name)
            {
                // 1. set found flag
                found_flag = true;

                // 2. get the position guid.
                map_guid = obj->getValue<std::string>("guid");
            }
        }
        else
        {
            // already found the map
            // do nothing.
            return map_guid;
        }
    }

    if(found_flag == false)
    {
        std::cout << "there is not such map in MiR!" << std::endl;
    }

    return map_guid;
}

// @@ input: timeout. suggestion: 1 minute.
//
bool yf::ugv::mir::MissionStatusCheck(const int& timeout_min)
{
    // init
    bool inner_mission_flag = false;

    // initialization
    bool update_flag = true;

    std::string time_future = sql_ptr_->CountdownTime(sql_ptr_->TimeNow(),timeout_min);

    while (update_flag)
    {
        // 1. check plc 004
        int plc_004_value = this->GetPLCRegisterIntValue(4);

        if(plc_004_value == 1)
        {
            inner_mission_flag = true;
        }

        // 2. get current state
        this->RetrieveUgvCurrentMissionState();

        // 3. set mission_continue_flag
        if(inner_mission_flag == true && nw_status_ptr_->ugv_mission_status_ == data::common::UgvState::Executing)
        {
            update_flag = false;
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        /// wait too long.
        if(!sql_ptr_->isFutureTime(time_future, sql_ptr_->TimeNow()))
        {
            update_flag = false;
            return false;
        }
    }

    return true;
}

//@@ input: plc register: 0-100
//
int yf::ugv::mir::GetPLCRegisterIntValue(const int &plc_register) {

    if(plc_register > 100 || plc_register <= 0)
    {
        std::cerr << "wrong plc register!" << std::endl;
        return 0;
    }

    std::string sub_path = "/api/v2.0.0/registers/"+ std::to_string(plc_register);

    GetMethod(sub_path);

    auto json_result = GetRequestResult();

    Poco::JSON::Parser parser;

    Poco::Dynamic::Var result = parser.parse(json_result);

    Poco::JSON::Object::Ptr object = result.extract<Poco::JSON::Object::Ptr>();

    int value = object->getValue<int>("value");

    return value;

}

//@@ input: plc register: 101-200
//
float yf::ugv::mir::GetPLCRegisterFloatValue(const int &plc_register)
{
    if(plc_register <= 100 || plc_register >200)
    {
        std::cerr << "wrong plc register!" << std::endl;
        return 0;
    }

    std::string sub_path = "/api/v2.0.0/registers/"+ std::to_string(plc_register);

    GetMethod(sub_path);

    auto json_result = GetRequestResult();

    Poco::JSON::Parser parser;

    Poco::Dynamic::Var result = parser.parse(json_result);

    Poco::JSON::Object::Ptr object = result.extract<Poco::JSON::Object::Ptr>();

    float value = object->getValue<float>("value");

    return value;
}

void yf::ugv::mir::RetrieveUgvCurrentMissionState() 
{
    int state = this->GetState();

    switch (state) 
    {
        case 1:
        {
            nw_status_ptr_->ugv_mission_status_ = yf::data::common::UgvState::Starting;
            break;
        }
        case 2:
        {
            nw_status_ptr_->ugv_mission_status_ = yf::data::common::UgvState::ShuttingDown;
            break;
        }
        case 3:
        {
            nw_status_ptr_->ugv_mission_status_ = yf::data::common::UgvState::Ready;
            break;
        }
        case 4:
        {
            nw_status_ptr_->ugv_mission_status_ = yf::data::common::UgvState::Pause;
            break;
        }
        case 5:
        {
            nw_status_ptr_->ugv_mission_status_ = yf::data::common::UgvState::Executing;
            break;
        }
        case 6:
        {
            nw_status_ptr_->ugv_mission_status_ = yf::data::common::UgvState::Aborted;
            break;
        }
        case 7:
        {
            nw_status_ptr_->ugv_mission_status_ = yf::data::common::UgvState::Completed;
            break;
        }
        case 10:
        {
            nw_status_ptr_->ugv_mission_status_ = yf::data::common::UgvState::EmergencyStop;
            break;
        }
        case 11:
        {
            nw_status_ptr_->ugv_mission_status_ = yf::data::common::UgvState::ManualControl;
            break;
        }
        case 12:
        {
            nw_status_ptr_->ugv_mission_status_ = yf::data::common::UgvState::Error;
            break;
        }
    }
}

bool yf::ugv::mir::SetPLCRegisterIntValue(const int &plc_register, const int &value)
{
    if(plc_register > 100 || plc_register < 0)
    {
        std::cerr << "wrong plc register!" << std::endl;
        return false;
    }

    Poco::JSON::Object reg_json;

    reg_json.set("value",value);

    //  /api/v2.0.0/registers/1
    std::string sub_path = "/api/v2.0.0/registers/" + std::to_string(plc_register);

    return PostMethod(sub_path, reg_json);
}

bool yf::ugv::mir::SetPLCRegisterfloatValue(const int &plc_register, const float &value)
{
    if(plc_register <= 100 || plc_register >200)
    {
        std::cerr << "wrong plc register!" << std::endl;
        return false;
    }

    Poco::JSON::Object reg_json;

    reg_json.set("value",value);

    //  /api/v2.0.0/registers/1
    std::string sub_path = "/api/v2.0.0/registers/" + std::to_string(plc_register);

    return PostMethod(sub_path, reg_json);
}

bool yf::ugv::mir::InitialStatusCheckForMission(const int &timeout_min)
{
    // initialization
    bool update_flag = true;

    std::string time_future = sql_ptr_->CountdownTime(sql_ptr_->TimeNow(),timeout_min);

    while (update_flag)
    {
        // get current ugv status
        this->RetrieveUgvCurrentMissionState();

        if (nw_status_ptr_->ugv_mission_status_ != data::common::UgvState::Ready)
        {
            this->DeleteMissionQueue();

            this->Play();

            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
        else
        {
            // Ugv is ready
            update_flag = false;
            break;
        }

        /// IPC1 waits too long.
        if(!sql_ptr_->isFutureTime(time_future, sql_ptr_->TimeNow()))
        {
            LOG(INFO) << "The Ugv is not ready.";
            LOG(INFO) << "The Task failed at the initial status check stage.";

            return false;
        }
    }

    return true;
}

bool yf::ugv::mir::Play()
{
    Poco::JSON::Object State;

    State.set("state_id", 3);

    return PutMethod("/api/v2.0.0/status", State);
}

bool yf::ugv::mir::Pause()
{
    Poco::JSON::Object State;

    State.set("state_id", 4);

    return PutMethod("/api/v2.0.0/status", State);
}

bool yf::ugv::mir::PostMissionQueue(const std::string &mission_guid)
{
    Poco::JSON::Object MissionQueue;

    MissionQueue.set("mission_id", mission_guid);

    return PostMethod("/api/v2.0.0/mission_queue", MissionQueue);
}

bool yf::ugv::mir::DeleteMissionQueue()
{
    return DeleteMethod("/api/v2.0.0/mission_queue");
}

bool yf::ugv::mir::DeleteMethod(const std::string& sub_path)
{
    try
    {
        uri_.setPath(sub_path);

        std::string path(uri_.getPathAndQuery());
        if (path.empty()) path = "/";

        HTTPClientSession session(uri_.getHost(), uri_.getPort());
        HTTPRequest request(HTTPRequest::HTTP_DELETE, path, HTTPMessage::HTTP_1_1);
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

std::deque<std::string> yf::ugv::mir::GetSessionNameList()
{
    // output
    std::deque<std::string> session_name_list;

    // method

    /// 1. fine tune sub_path
    std::string sub_path = "/api/v2.0.0/sessions";

    GetMethod(sub_path);

    auto json_result = GetRequestResult();

    Poco::JSON::Parser parser;

    Poco::Dynamic::Var result = parser.parse(json_result);

    Poco::JSON::Array::Ptr array = result.extract<Poco::JSON::Array::Ptr>();

    // iteration
    for (Poco::JSON::Array::ConstIterator it= array->begin(); it != array->end(); ++it)
    {

        Poco::JSON::Object::Ptr obj = it->extract<Poco::JSON::Object::Ptr>();

        std::string name = obj->getValue<std::string>("name");

        session_name_list.push_back(name);
    }

    return session_name_list;
}

std::deque<std::string> yf::ugv::mir::GetMapNameList(const std::string &session_name)
{
    // output
    std::deque<std::string> map_name_list;

    // input
    std::string session_guid = this->GetSessionGUID(session_name);

    // method

    /// 1. fine tune sub_path
    // /api/v2.0.0/sessions/7aa0de9c-8579-11eb-9840-00012978eb45/maps
    std::string sub_path = "/api/v2.0.0/sessions/" + session_guid + "/maps";

    GetMethod(sub_path);

    auto json_result = GetRequestResult();

    Poco::JSON::Parser parser;

    Poco::Dynamic::Var result = parser.parse(json_result);

    Poco::JSON::Array::Ptr array = result.extract<Poco::JSON::Array::Ptr>();

    // iteration
    for (Poco::JSON::Array::ConstIterator it= array->begin(); it != array->end(); ++it)
    {

        Poco::JSON::Object::Ptr obj = it->extract<Poco::JSON::Object::Ptr>();

        std::string name = obj->getValue<std::string>("name");

        map_name_list.push_back(name);
    }

    return map_name_list;
}

std::string yf::ugv::mir::GetSessionGUID(const std::string &session_name)
{
    // output
    std::string session_guid;

    // method

    /// 1. fine tune sub_path
    std::string sub_path = "/api/v2.0.0/sessions";

    GetMethod(sub_path);

    auto json_result = GetRequestResult();

    Poco::JSON::Parser parser;

    Poco::Dynamic::Var result = parser.parse(json_result);

    Poco::JSON::Array::Ptr array = result.extract<Poco::JSON::Array::Ptr>();

    bool found_flag = false;

    for (Poco::JSON::Array::ConstIterator it= array->begin(); it != array->end(); ++it)
    {
        if(found_flag == false)
        {
            // iteration, find the position_name here.
            Poco::JSON::Object::Ptr obj = it->extract<Poco::JSON::Object::Ptr>();

            std::string name = obj->getValue<std::string>("name");

            if(name == session_name)
            {
                // 1. set found flag
                found_flag = true;

                // 2. get the position guid.
                session_guid = obj->getValue<std::string>("guid");
            }
        }
        else
        {
            // already found the map
            // do nothing.
            return session_guid;
        }
    }

    if(found_flag == false)
    {
        std::cout << "there is not such session in MiR!" << std::endl;
    }

    return session_guid;
}

std::deque<std::string> yf::ugv::mir::GetPositionNameList(const std::string& session_name, const std::string& map_name)
{
    // output
    std::deque<std::string> position_name_list;

    // input
    std::string map_guid = this->GetMapGUID(session_name,map_name);

    // method

    /// 1. fine tune sub_path
    // /api/v2.0.0/maps/c9a8d3bf-8237-11eb-aa75-00012978eb45/positions
    std::string sub_path = "/api/v2.0.0/maps/" + map_guid + "/positions";

    GetMethod(sub_path);

    auto json_result = GetRequestResult();

    Poco::JSON::Parser parser;

    Poco::Dynamic::Var result = parser.parse(json_result);

    Poco::JSON::Array::Ptr array = result.extract<Poco::JSON::Array::Ptr>();

    // iteration
    for (Poco::JSON::Array::ConstIterator it= array->begin(); it != array->end(); ++it)
    {

        Poco::JSON::Object::Ptr obj = it->extract<Poco::JSON::Object::Ptr>();

        std::string name = obj->getValue<std::string>("name");

        position_name_list.push_back(name);
    }

    return position_name_list;
}

std::string yf::ugv::mir::GetMapGUID(const std::string &session_name, const std::string &map_name)
{
    // input
    std::string session_guid_str = this->GetSessionGUID(session_name);

    // output
    std::string map_guid;

    // method

    /// 1. fine tune sub_path
    // /api/v2.0.0/sessions/7aa0de9c-8579-11eb-9840-00012978eb45/maps
    std::string sub_path = "/api/v2.0.0/sessions/" + session_guid_str + "/maps";

    GetMethod(sub_path);

    auto json_result = GetRequestResult();

    Poco::JSON::Parser parser;

    Poco::Dynamic::Var result = parser.parse(json_result);

    Poco::JSON::Array::Ptr array = result.extract<Poco::JSON::Array::Ptr>();

    bool found_flag = false;

    for (Poco::JSON::Array::ConstIterator it= array->begin(); it != array->end(); ++it)
    {
        if(found_flag == false)
        {
            // iteration, find the position_name here.
            Poco::JSON::Object::Ptr obj = it->extract<Poco::JSON::Object::Ptr>();

            std::string name = obj->getValue<std::string>("name");

            if(name == map_name)
            {
                // 1. set found flag
                found_flag = true;

                // 2. get the position guid.
                map_guid = obj->getValue<std::string>("guid");
            }
        }
        else
        {
            // already found the map
            // do nothing.
            return map_guid;
        }
    }

    if(found_flag == false)
    {
        std::cout << "there is not such session in MiR!" << std::endl;
    }

    return map_guid;
}

float yf::ugv::mir::GetBatteryPercentage()
{
    GetMethod("/api/v2.0.0/status");

    auto json_result = GetRequestResult();

    Poco::JSON::Parser parser;

    Poco::Dynamic::Var result = parser.parse(json_result);

    Poco::JSON::Object::Ptr object = result.extract<Poco::JSON::Object::Ptr>();

    float state = object->getValue<float>("battery_percentage");

    return state;
}

std::vector<float> yf::ugv::mir::GetCurPosition()
{
    GetMethod("/api/v2.0.0/status");

    auto json_result = GetRequestResult();

    Poco::JSON::Parser parser;

    Poco::Dynamic::Var result = parser.parse(json_result);

    Poco::JSON::Object::Ptr object = result.extract<Poco::JSON::Object::Ptr>();

    Poco::JSON::Object::Ptr position_obj = object->getObject("position");

    auto x = position_obj->getValue<float>("x");
    auto y = position_obj->getValue<float>("y");
    auto orientation = position_obj->getValue<float>("orientation");

    std::vector<float> position;

    position.push_back(x);
    position.push_back(y);
    position.push_back(orientation);

    return position;
}

bool yf::ugv::mir::ChangeMap(const std::string& map_name)
{
    // Pause first
    this->Pause();

    // Change the map
    Poco::JSON::Object State;

    auto map_guid = this->GetMapGUID(map_name);

    State.set("map_id", map_guid);

    return PutMethod("/api/v2.0.0/status", State);

}

bool yf::ugv::mir::ChangeMapByDBMapStatus()
{
    auto map_name = sql_ptr_->GetMapNameFromMapStatus();

    return this->ChangeMap(map_name);
}

bool yf::ugv::mir::ChangeInitPositionByDBMapStatus()
{
    // Pause first
    this->Pause();

    auto init_position =  sql_ptr_->GetUgvInitPositionFromMapStatus();


    Poco::JSON::Object status_json;

    Poco::JSON::Object position_json;

    position_json.set("x", init_position[0]);
    position_json.set("y", init_position[1]);
    position_json.set("orientation", init_position[2]);

    status_json.set("position", position_json);

    /// (4) fine tune the sub_path

    std::string sub_path = "/api/v2.0.0/status";

    return PutMethod(sub_path, status_json);

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























