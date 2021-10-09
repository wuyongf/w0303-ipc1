#pragma once

// Std
#include <cstdio>
#include <ctime>
#include <iostream>
#include <string>
#include <map>
#include <thread>
#include <mutex>
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

#include "Poco/Mutex.h"

// Sys
#include "nw_status.h"
#include "sql.h"
#include "al.h"

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

        public: /// MiR Properties

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

        private: /// Rest API: CRUD

            /// Most Basic Function *
            std::string GetRequestResult();

            /// Most Basic Function **
            bool doRequest(Poco::Net::HTTPClientSession& session, Poco::Net::HTTPRequest& request, Poco::Net::HTTPResponse& response);

            /// Basic Methods
            bool GetMethod(const std::string& sub_path);
            bool PutMethod(const std::string& sub_path,const Poco::JSON::Object& obj);
            bool PostMethod(const std::string& sub_path, const Poco::JSON::Object& obj);
            bool DeleteMethod(const std::string& sub_path);

            Poco::Mutex _mutex;

        public:

            void Start(const std::string& ip_addr, std::shared_ptr<yf::status::nw_status> status_ptr, std::shared_ptr<yf::sql::sql_server> sql_ptr);
            void Close();

        public: /// Layer 1: Interact with Basic GET/POST/PUT/Delete Methods

            // Connection Status
            bool IsConnected();

            // Get Mission Status From RestAPI

            int   GetPLCRegisterIntValue(const int& plc_register);
            float GetPLCRegisterFloatValue(const int& plc_register);

            bool  SetPLCRegisterIntValue(const int& plc_register, const int& value);
            bool  SetPLCRegisterFloatValue(const int& plc_register, const float& value);

        public: ///  Layer2: Interact with nw_status, database

            // methods for ipc1
            //
            // (1) check mission continue flag
            bool MissionStatusCheck(const int& timeout_min);
            
            // (2) current ugv mission state / mission status
            void RetrieveUgvCurrentMissionState();

            // a. Initial status check for mission
            bool InitialStatusCheckForMission(const int& timeout_min);

            // 1. update to nw_status
            // 2. update to database
            void UpdateUgvMissionStatus(const yf::data::ugv::Status& status);

            // (1) check whether
            bool ClearErrorState();

            // (1) For Different Areas Initialization
            //
            bool UpdatePositionOnMap(const float& pos_x, const float& pos_y, const float& pos_theta);

        public: /// Layer3: create a mission --- REST API

            ///3.1 based on model_config_id, mission_order, position_name, create a ugv mission.

            // name protocal:
            // building_floor_ModelName_time
            // 12w_2f_handrail_001_202104061026
            bool PostMission(const int& model_config_id);
            void PostActions(const int& model_config_id);

            bool PostRedoMission(const int& origin_model_config_id);
            void PostRedoActions(const int& origin_model_config_id, const std::vector<int>& redo_ids);

            bool PostMissionForDebugTest(const int& model_config_id);
            void PostActionsForDebugTest(const int& model_config_id);

            bool PostActionSetPLC(const int& plc_register, const float& value, const std::string& mission_guid, const int& priority);
            bool PostActionWaitPLC(const int& plc_register, const float& value, const std::string& mission_guid, const int& priority);
            bool PostActionMove(const std::string& position_guid, const std::string &mission_guid, const int &priority);
            bool PostActionAdjustLocalization(const std::string &mission_guid, const int &priority);
            bool PostActionSpeed(const float& speed, const std::string &mission_guid, const int &priority);
            bool PostActionDocking(const std::string& docking_position_guid, const std::string &mission_guid, const int &priority);
            bool PostActionCharging(const std::string &mission_guid, const int &priority);

            // for relative move
            bool PostActionWhile(const std::string& mission_guid, const int& priority);

            bool PostMissionForCharging();
            void PostActionsForCharging();

            std::string GetDockingPositionGUID(const std::string& map_guid);

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
            // Getter: MiR Properties
            int     GetModeId();
            int     GetStateId();
            float   GetBatteryPercentage();
            std::vector<float> GetCurPosition();

            yf::data::ugv::Status GetUgvStatus();

        public:
            // MiR Mission Control
            bool Play();
            bool Pause();

            bool PostMissionQueue(const std::string& mission_guid);
            bool DeleteMissionQueue();

            bool PutMap(const std::string& map_name);

        public:
            // MiR Rest API & Database
            bool ChangeMapByDBMapStatus();

            bool WaitForModeId(const int& mode_id, const int& timeout_min);

            bool ChangeInitPositionByDBMapStatus();


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

            yf::algorithm::TimeSleep sleep;
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
    return  request_result_;
}

bool yf::ugv::mir::doRequest(HTTPClientSession &session,
                             HTTPRequest &request,
                             HTTPResponse &response)
{
//    std::scoped_lock lock();
//    request_result_.clear();

    session.sendRequest(request);
    std::istream& rs = session.receiveResponse(response);

    respond_status_ = response.getStatus();
    respond_reason_ = response.getReason();

//    std::cout << respond_status_ << " " << respond_reason_ << std::endl;

    if (response.getStatus() != Poco::Net::HTTPResponse::HTTP_UNAUTHORIZED)
    {
        request_result_.clear();

        StreamCopier::copyToString(rs, request_result_);
//        std::cout << "result: " << request_result_ << std::endl;
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
    Poco::Mutex::ScopedLock lock(_mutex);

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
    Poco::Mutex::ScopedLock lock(_mutex);

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
    Poco::Mutex::ScopedLock lock(_mutex);

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

int yf::ugv::mir::GetStateId()
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
    Poco::Net::AddressFamily family;

    Poco::Net::ICMPClient icmpClient(family.IPv4);

    auto result = icmpClient.ping(ip_address_,3);

    if(result == 0)
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
    auto building_info  = sql_ptr_->GetBuildingInfo(model_config_id);
    auto floor_info     = sql_ptr_->GetFloorInfo(model_config_id);
    auto model_name     = sql_ptr_->GetModelName(model_config_id);

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
    // 1. map
    //  1.a. map name
    auto map_id = sql_ptr_->GetMapIdFromModelId(sql_ptr_->GetModelId(model_config_id));
    auto map_name = sql_ptr_->GetMapElement(map_id,"map_name");
    //  1.b. map_guid
    std::string map_guid = this->GetMapGUID(map_name);

    // 2. get mission_guid from REST
    std::string mission_guid = this->GetCurMissionGUID();

    // 3. get position_names from DB, based on model_config_id
    std::deque<std::string> position_names  = sql_ptr_->GetUgvMissionConfigPositionNames(model_config_id);

    // 4. for relative move
    std::deque<int>         mission_types   = sql_ptr_->GetUgvMissionConfigMissionType(model_config_id);
    std::deque<int>         amc_ids_count   = sql_ptr_->GetArmMissionConfigIdsCount(model_config_id);

    /// Actions details
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

    /// Initialization Ugv Properties
    // speed
    this->PostActionSpeed(0.6,mission_guid,priority);
    priority++;
    // adjust localization
    this->PostActionAdjustLocalization(mission_guid,priority);
    priority++;

    /// (3) For loop, mission details
    //
    // get mission_name from REST. mission_order from db
    for (int mission_count = 1; mission_count <= total_position_num; mission_count++)
    {
        /// a. get position name.
        std::string position_name = position_names[mission_count-1];

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
    sleep.ms(50);

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

        // null value
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

        if(plc_004_value == 1) // mission is executing
        {
            inner_mission_flag = true;
        }

        if(plc_004_value == 3) // mission has error
        {
            return false;
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

///\brief
/// 1. Get Ugv Mission Status from RestAPI
/// 2. Update nw_status
/// 3. Update database
void yf::ugv::mir::RetrieveUgvCurrentMissionState() 
{
    int state = this->GetStateId();

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
            sql_ptr_->UpdateDeviceMissionStatus("ugv", 1);
            break;
        }
        case 4:
        {
            nw_status_ptr_->ugv_mission_status_ = yf::data::common::UgvState::Pause;
            sql_ptr_->UpdateDeviceMissionStatus("ugv", 4);
            break;
        }
        case 5:
        {
            nw_status_ptr_->ugv_mission_status_ = yf::data::common::UgvState::Executing;
            sql_ptr_->UpdateDeviceMissionStatus("ugv", 2);
            break;
        }
        case 6:
        {
            nw_status_ptr_->ugv_mission_status_ = yf::data::common::UgvState::Aborted;
            sql_ptr_->UpdateDeviceMissionStatus("ugv", 5);
            break;
        }
        case 7:
        {
            nw_status_ptr_->ugv_mission_status_ = yf::data::common::UgvState::Completed;
            sql_ptr_->UpdateDeviceMissionStatus("ugv", 3);
            break;
        }
        case 10:
        {
            nw_status_ptr_->ugv_mission_status_ = yf::data::common::UgvState::EmergencyStop;
            sql_ptr_->UpdateDeviceMissionStatus("ugv", 7);
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
            sql_ptr_->UpdateDeviceMissionStatus("ugv", 6);
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

bool yf::ugv::mir::SetPLCRegisterFloatValue(const int &plc_register, const float &value)
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
    Poco::Mutex::ScopedLock lock(_mutex);

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

bool yf::ugv::mir::PutMap(const std::string& map_name)
{
    // Clear Error

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

    return this->PutMap(map_name);
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

int yf::ugv::mir::GetModeId()
{
    GetMethod("/api/v2.0.0/status");

    auto json_result = GetRequestResult();

    Poco::JSON::Parser parser;

    Poco::Dynamic::Var result = parser.parse(json_result);

    Poco::JSON::Object::Ptr object = result.extract<Poco::JSON::Object::Ptr>();

    int mode_id = object->getValue<int>("mode_id");

    return mode_id;
}

bool yf::ugv::mir::WaitForModeId(const int &mode_id, const int &timeout_min)
{
    std::string time_future = sql_ptr_->CountdownTime(sql_ptr_->TimeNow(),timeout_min);

    LOG(INFO) << "Wait for Mode Id == " << mode_id ;
    while (true)
    {
        // get cur ugv plc register xx, value xxx.
        auto result = this->GetModeId();

        if (mode_id != result)
        {
            ///TIME
            sleep.ms(2000);
        }
        else
        {
            break;
        }

        /// IPC1 waits too long.
        if(!sql_ptr_->isFutureTime(time_future, sql_ptr_->TimeNow()))
        {
            LOG(INFO) << "The Mode Id has not changed, current mode_id: " << result;

            return false;
        }
    }

    return true;
}

bool yf::ugv::mir::PostMissionForDebugTest(const int &model_config_id)
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
    cur_mission_name_ = building_info + "_" + floor_info + "/F_" + model_name + "_" + time + "_DebugTest";

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

void yf::ugv::mir::PostActionsForDebugTest(const int &model_config_id)
{
    // 1. map
    //  1.a. map name
    auto map_id = sql_ptr_->GetMapIdFromModelId(sql_ptr_->GetModelId(model_config_id));
    auto map_name = sql_ptr_->GetMapElement(map_id,"map_name");
    //  1.b. map_guid
    std::string map_guid = this->GetMapGUID(map_name);

    // 2. get mission_guid from REST
    std::string mission_guid = this->GetCurMissionGUID();

    // 3. get position_names from DB, based on model_config_id
    std::deque<std::string> position_names = sql_ptr_->GetUgvMissionConfigPositionNames(model_config_id);

    // 4. for relative move
    std::deque<int>         mission_types   = sql_ptr_->GetUgvMissionConfigMissionType(model_config_id);
    std::deque<int>         amc_ids_count   = sql_ptr_->GetArmMissionConfigIdsCount(model_config_id);

    /// Actions details
    int total_position_num = sql_ptr_->GetUgvMissionConfigNum(model_config_id);
    int priority = 1;

    this->PostActionWhile(mission_guid,priority);
    priority++;
    this->PostActionSetPLC(4,1,mission_guid,priority);
    priority++;

#if 0
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

    /// Initialization Ugv Properties
    // speed
    this->PostActionSpeed(0.6,mission_guid,priority);
    priority++;
    // adjust localization
    this->PostActionAdjustLocalization(mission_guid,priority);
    priority++;

    /// (3) For loop, mission details
    //
    // get mission_name from REST. mission_order from db
    for (int mission_count = 1; mission_count <= total_position_num; mission_count++)
    {
        /// a. get position name.
        std::string position_name = position_names[mission_count-1];

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

#endif
}

bool yf::ugv::mir::PostActionAdjustLocalization(const std::string &mission_guid, const int &priority)
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

    Poco::JSON::Object action_json;


    Poco::JSON::Array parameters_array;

    action_json.set("parameters", parameters_array);
    action_json.set("priority", priority);
    action_json.set("action_type", "adjust_localization");

    /// (4) fine tune the sub_path

    std::string sub_path = "/api/v2.0.0/missions/" + mission_guid + "/actions";

    return PostMethod(sub_path, action_json);
}

bool
yf::ugv::mir::PostActionDocking(const std::string &docking_position_guid, const std::string &mission_guid, const int &priority)
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


    Poco::JSON::Object action_docking_json;

    Poco::JSON::Object marker_json;
    Poco::JSON::Object marker_type_json;
    Poco::JSON::Object retries_json;
    Poco::JSON::Object max_linear_speed_json;

    marker_json.set("value", docking_position_guid);
    marker_json.set("id", "marker");

    marker_type_json.set("value", "mirconst-guid-0000-0001-marker000001");
    marker_type_json.set("id", "marker_type");

    retries_json.set("value", 10);
    retries_json.set("id", "retries");

    max_linear_speed_json.set("value", 0.5);
    max_linear_speed_json.set("id", "max_linear_speed");

    Poco::JSON::Array parameters_array;
    parameters_array.set(0, marker_json);
    parameters_array.set(1, marker_type_json);
    parameters_array.set(2, retries_json);
    parameters_array.set(3, max_linear_speed_json);

    action_docking_json.set("parameters", parameters_array);
    action_docking_json.set("priority", priority);

    action_docking_json.set("action_type", "docking");

    /// (4) fine tune the sub_path

    std::string sub_path = "/api/v2.0.0/missions/" + mission_guid + "/actions";

    return PostMethod(sub_path, action_docking_json);
}

std::string yf::ugv::mir::GetDockingPositionGUID(const std::string &map_guid)
{
    std::string docking_name = "ChargingStation";

    // /api/v2.0.0/maps/cb4cfe79-8dd6-11eb-a5e3-00012978eb45/positions

    /// 0. prepare the position_guid
    std::string docking_guid;

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
            int type_id = obj->getValue<int>("type_id");

            if(pos_name == docking_name && type_id == 7)
            {
                // 1. set found flag
                found_flag = true;

                // 2. get the position guid.
                docking_guid = obj->getValue<std::string>("guid");
            }
        }
        else
        {
            // already found the position
            // do nothing.
            return docking_guid;
        }
    }

    if(found_flag == false)
    {
        std::cout << "Please create a charging station first!" << std::endl;
    }

    return docking_guid;
}

bool yf::ugv::mir::ClearErrorState()
{
    Poco::JSON::Object State;

    State.set("clear_error", true);

    return PutMethod("/api/v2.0.0/status", State);
}

bool yf::ugv::mir::PostActionSpeed(const float &speed, const std::string &mission_guid, const int &priority)
{
    /// prerequisite:
    /// 1. get position guid... based on position_name, map_id

    Poco::JSON::Object action_move_json;

    Poco::JSON::Object planner_params_json;
    Poco::JSON::Object desired_speed_json;
    Poco::JSON::Object path_timeout_json;
    Poco::JSON::Object path_deviation_json;
    Poco::JSON::Object obstacle_history_json;


    planner_params_json.set("value", "desired_speed_key");
    planner_params_json.set("id", "planner_params");

    desired_speed_json.set("value", speed);
    desired_speed_json.set("id", "desired_speed");

    path_timeout_json.set("value", 5);
    path_timeout_json.set("id", "path_timeout");

    path_deviation_json.set("value", 5);
    path_deviation_json.set("id", "path_deviation");

    obstacle_history_json.set("value", "no_clearing");
    obstacle_history_json.set("id", "obstacle_history");

    Poco::JSON::Array parameters_array;
    parameters_array.set(0, planner_params_json);
    parameters_array.set(1, desired_speed_json);
    parameters_array.set(2, path_timeout_json);
    parameters_array.set(3, path_deviation_json);
    parameters_array.set(4, obstacle_history_json);

    action_move_json.set("parameters", parameters_array);
    action_move_json.set("priority", priority);
    action_move_json.set("action_type", "planner_settings");

    /// (4) fine tune the sub_path

    std::string sub_path = "/api/v2.0.0/missions/" + mission_guid + "/actions";

    return PostMethod(sub_path, action_move_json);
}

bool yf::ugv::mir::PostActionCharging(const std::string &mission_guid, const int &priority)
{
    /// prerequisite:
    /// 1. get position guid... based on position_name, map_id

    Poco::JSON::Object action_move_json;

    Poco::JSON::Object minimum_time_json;
    Poco::JSON::Object minimum_percentage_json;
    Poco::JSON::Object charge_until_new_mission_json;


    minimum_time_json.set("value", {});
    minimum_time_json.set("id", "minimum_time");

    minimum_percentage_json.set("value", {});
    minimum_percentage_json.set("id", "minimum_percentage");

    charge_until_new_mission_json.set("value", true);
    charge_until_new_mission_json.set("id", "charge_until_new_mission");

    Poco::JSON::Array parameters_array;
    parameters_array.set(0, minimum_time_json);
    parameters_array.set(1, minimum_percentage_json);
    parameters_array.set(2, charge_until_new_mission_json);

    action_move_json.set("parameters", parameters_array);
    action_move_json.set("priority", priority);
    action_move_json.set("action_type", "charging");

    /// (4) fine tune the sub_path

    std::string sub_path = "/api/v2.0.0/missions/" + mission_guid + "/actions";

    return PostMethod(sub_path, action_move_json);
}

bool yf::ugv::mir::PostMissionForCharging()
{
    /// 1. get site_id, building_id, floor_id
    auto site_id        = sql_ptr_->GetSiteIdFromMapStatus();
    auto building_id    = sql_ptr_->GetBuildingIdFromMapStatus();
    auto floor_id       = sql_ptr_->GetFloorIdFromMapStatus();
    /// 2. get site_info, building_info, floor_info
    auto site_info      = sql_ptr_->GetSiteInfoFromSiteId(site_id);
    auto building_info  = sql_ptr_->GetBuildingInfoFromBuildingId(building_id);
    auto floor_info     = sql_ptr_->GetFloorInfoFromFromFloorId(floor_id);

    /// 4. get time
    sql_ptr_->UpdateTime();
    auto time_year = sql_ptr_->get_time_element("year");
    auto time_month = sql_ptr_->get_time_element("month");
    auto time_day = sql_ptr_->get_time_element("day");
    auto time_hour = sql_ptr_->get_time_element("hour");
    auto time_min = sql_ptr_->get_time_element("minute");
    auto time_sec = sql_ptr_->get_time_element("sec");

    auto time = time_year + time_month + time_day + "_" + time_hour + time_min + "_" + time_sec;

    /// (1) set cur_mission_name
    cur_mission_name_ = building_info + "_" + floor_info + "/F_UgvBackToChargingStation_" + time;

    /// (2) get session_id

    if(site_info == "hkstp")
    {
        cur_session_guid_ = session_guid_HKSTP_;
    }
    else if(site_info == "emsd")
    {
        cur_session_guid_ = session_guid_EMSD_;
    }
    else if(site_info == "ha")
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

void yf::ugv::mir::PostActionsForCharging()
{
    /// prerequisite
    // 1. get mission_guid from REST
    std::string mission_guid = this->GetCurMissionGUID();

    // 2. map_guid
    auto map_name = sql_ptr_->GetMapNameFromMapStatus();
    std::string map_guid = this->GetMapGUID(map_name);

    // 3. docking_position_guid
    std::string dock_position_guid = this->GetDockingPositionGUID(map_guid);

    /// Actions detail
    //
    int priority = 1;

    /// Initialization Ugv Properties
    // speed
    this->PostActionSpeed(0.6,mission_guid,priority);
    priority++;
    // adjust localization
    this->PostActionAdjustLocalization(mission_guid,priority);
    priority++;
    // go to docking
    this->PostActionDocking(dock_position_guid,mission_guid,priority);
    priority++;
    // charging
    this->PostActionCharging(mission_guid,priority);
    priority++;
}

bool yf::ugv::mir::PostRedoMission(const int &origin_model_config_id)
{
    /// based on model_config_id, mission_order, position_name, create a ugv mission.

    /// 1. get positions.
//    auto position_names = sql_ptr_->GetUgvMissionConfigPositionNames(model_config_id);

    /// 2. get session.
    auto session_info = sql_ptr_->GetSiteInfo(origin_model_config_id);

    /// 3. get building---floor info--model name---time
    auto building_info  = sql_ptr_->GetBuildingInfo(origin_model_config_id);
    auto floor_info     = sql_ptr_->GetFloorInfo(origin_model_config_id);
    auto model_name     = sql_ptr_->GetModelName(origin_model_config_id);

    // time
    sql_ptr_->UpdateTime();
    auto time_year = sql_ptr_->get_time_element("year");
    auto time_month = sql_ptr_->get_time_element("month");
    auto time_day = sql_ptr_->get_time_element("day");
    auto time_hour = sql_ptr_->get_time_element("hour");
    auto time_min = sql_ptr_->get_time_element("minute");

    auto time = time_year + time_month + time_day + "_" + time_hour + time_min;

    /// (1) set cur_mission_name
    cur_mission_name_ = building_info + "_" + floor_info + "/F_" + model_name + "_" + "Redo_"+ time;

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

void yf::ugv::mir::PostRedoActions(const int& origin_model_config_id, const std::vector<int>& redo_ids)
{
// 1. get mission_guid from REST
    std::string mission_guid = this->GetCurMissionGUID();

    // 2. get position_names from DB, based on task_group_id
    std::deque<std::string> position_names = sql_ptr_->GetRedoUgvMissionConfigPositionNames(redo_ids);

    // 3.
    // 3.a. get map name from db.
    auto model_id = sql_ptr_->GetModelId(origin_model_config_id);
    auto map_id = sql_ptr_->GetMapIdFromModelId(model_id);
    auto map_name = sql_ptr_->GetMapElement(map_id,"map_name");
    // 3.b. based on map_name, retrieve map_guid from REST.
    std::string map_guid = this->GetMapGUID(map_name);

    /// Actions detail
    ///
    int total_position_num = redo_ids.size();
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

    /// Initialization Ugv Properties
    // speed
    this->PostActionSpeed(0.6,mission_guid,priority);
    priority++;
    // adjust localization
    this->PostActionAdjustLocalization(mission_guid,priority);
    priority++;

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

yf::data::ugv::Status yf::ugv::mir::GetUgvStatus()
{
    /// Get mir100 status via RestAPI

    GetMethod("/api/v2.0.0/status");

    auto json_result = GetRequestResult();

    Poco::JSON::Parser parser;

    Poco::Dynamic::Var result = parser.parse(json_result);

    Poco::JSON::Object::Ptr object = result.extract<Poco::JSON::Object::Ptr>();

    /// Retrieve all information.

    // 1. battery_percentage
    auto battery_percentage = object->getValue<float>("battery_percentage");

    // 2. position
    Poco::JSON::Object::Ptr position_obj = object->getObject("position");

    auto x = position_obj->getValue<float>("x");
    auto y = position_obj->getValue<float>("y");
    auto orientation = position_obj->getValue<float>("orientation");

    // 3. state_id
    auto state_id = object->getValue<int>("state_id");

    // 4. mode_id
    auto mode_id = object->getValue<int>("mode_id");

    /// Assign mir100 status
    yf::data::ugv::Status status;

    // 1. battery_status
    status.battery_percentage = battery_percentage;
    // 2. position
    status.position.x = x;
    status.position.y = y;
    status.position.orientation = orientation;
    // 3. state_id
    status.state_id = state_id;
    // 4. mode_id
    status.mode_id  = mode_id;

    return status;
}

void yf::ugv::mir::UpdateUgvMissionStatus(const yf::data::ugv::Status &status)
{
    int state_id = status.state_id;

    switch (state_id)
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
            sql_ptr_->UpdateDeviceMissionStatus("ugv", 1);
            break;
        }
        case 4:
        {
            nw_status_ptr_->ugv_mission_status_ = yf::data::common::UgvState::Pause;
            sql_ptr_->UpdateDeviceMissionStatus("ugv", 4);
            break;
        }
        case 5:
        {
            nw_status_ptr_->ugv_mission_status_ = yf::data::common::UgvState::Executing;
            sql_ptr_->UpdateDeviceMissionStatus("ugv", 2);
            break;
        }
        case 6:
        {
            nw_status_ptr_->ugv_mission_status_ = yf::data::common::UgvState::Aborted;
            sql_ptr_->UpdateDeviceMissionStatus("ugv", 5);
            break;
        }
        case 7:
        {
            nw_status_ptr_->ugv_mission_status_ = yf::data::common::UgvState::Completed;
            sql_ptr_->UpdateDeviceMissionStatus("ugv", 3);
            break;
        }
        case 10:
        {
            nw_status_ptr_->ugv_mission_status_ = yf::data::common::UgvState::EmergencyStop;
            sql_ptr_->UpdateDeviceMissionStatus("ugv", 7);
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
            sql_ptr_->UpdateDeviceMissionStatus("ugv", 6);
            break;
        }
    }
}

bool yf::ugv::mir::PostActionWhile(const std::string &mission_guid, const int &priority)
{
    Poco::JSON::Object action_plc_json;

    Poco::JSON::Object registry_json;
    Poco::JSON::Object action_json;
    Poco::JSON::Object value_json_1;

    registry_json.set("value",001);
    registry_json.set("id","register");

    action_json.set("value","set");
    action_json.set("id","action");

    value_json_1.set("value",1);
    value_json_1.set("id","value");

    Poco::JSON::Array parameters_array_1;
    parameters_array_1.set(0,registry_json);
    parameters_array_1.set(1,action_json);
    parameters_array_1.set(2,value_json_1);

    action_plc_json.set("parameters", parameters_array_1);
    action_plc_json.set("priority", priority);
    action_plc_json.set("mission_id", mission_guid);
    action_plc_json.set("action_type", "set_plc_register");

    Poco::JSON::Array content_array;
    content_array.set(0, action_plc_json);

    std::ostringstream oss;
    Poco::JSON::Stringifier::stringify( action_plc_json, oss);

    // mission_guid (done)
    // action_type (done)  "while"
    // parameters (?)   {compare    }
    //                  {module     }
    //                  {io_port    }
    //                  {register   }
    //                  {operator   }
    //                  {value      }
    //                  {content    }
    // priority (done)

    Poco::JSON::Object action_while_json;

    Poco::JSON::Object compare_json;
    Poco::JSON::Object module_json;
    Poco::JSON::Object io_port_json;
    Poco::JSON::Object register_json;
    Poco::JSON::Object operator_json;
    Poco::JSON::Object value_json;
    Poco::JSON::Object content_json;


    compare_json.set("value", "plc_register");
    compare_json.set("id", "compare");

    module_json.set("value", {});
    module_json.set("id", "module");

    io_port_json.set("value", 0);
    io_port_json.set("id", "io_port");

    register_json.set("value", 6);
    register_json.set("id", "register");

    operator_json.set("value", "!=");
    operator_json.set("id", "operator");

    value_json.set("value", 0);
    value_json.set("id", "value");

    content_json.set("value", oss);
    content_json.set("id", "content");

    Poco::JSON::Array parameters_array;
    parameters_array.set(0, compare_json);
    parameters_array.set(1, module_json);
    parameters_array.set(2, io_port_json);
    parameters_array.set(3, register_json);
    parameters_array.set(4, operator_json);
    parameters_array.set(5, value_json);
    parameters_array.set(6, content_json);

    action_while_json.set("parameters", parameters_array);
    action_while_json.set("priority", priority);
    action_while_json.set("mission_id", mission_guid);
    action_while_json.set("action_type", "while");

    /// (4) fine tune the sub_path

    std::string sub_path = "/api/v2.0.0/missions/" + mission_guid + "/actions";

    return PostMethod(sub_path, action_while_json);
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























