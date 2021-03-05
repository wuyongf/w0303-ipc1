#pragma once

#include "data.h"

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
using Poco::StreamCopier;
using Poco::Path;
using Poco::URI;
using Poco::Exception;

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

            void Start(const std::string& ip_addr);
            void Close();

        public:

            bool IsConnected();

            int GetState();
            bool UpdatePositionOnMap(const float& pos_x, const float& pos_y, const float& pos_theta);

        private:

            bool doRequest(Poco::Net::HTTPClientSession& session, Poco::Net::HTTPRequest& request, Poco::Net::HTTPResponse& response);

        private:

            // for model info
            std::string model_name_ = "mir100";

            // for rest api
            std::string ip_address_ = "192.168.2.111";
            std::string uri_address_ = "http://" + ip_address_;

            URI uri_;

            std::string api_content_type_ = "application/json";
            std::string api_credentials_scheme_ = "Basic";
            std::string api_credentials_authentication_ = "ZGlzdHJpYnV0b3I6NjJmMmYwZjFlZmYxMGQzMTUyYzk1ZjZmMDU5NjU3NmU0ODJiYjhlNDQ4MDY0MzNmNGNmOTI5NzkyODM0YjAxNA==";

            // for rest api result
            std::string request_result_;

            int         respond_status_;
            std::string respond_reason_;
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

void yf::ugv::mir::Start(const std::string& ip_addr)
{
    // Initial setup
    //
    SetIpAddress(ip_addr);
    SetUriAddress();

    URI uri_origin(uri_address_);
    uri_ = uri_origin;

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

    return PutMethod("http://192.168.2.111/api/v2.0.0/status", Position);
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
//  can not use for now.
//  use modbus?
bool yf::ugv::mir::IsConnected()
{
    uri_.setPath("/api/v2.0.0/registers/102");

    std::string path(uri_.getPathAndQuery());
    if (path.empty()) path = "/";

    HTTPClientSession session(uri_.getHost(), uri_.getPort());
    session.setKeepAlive(true);
    session.setKeepAliveTimeout(3000);
    std::cout << "time out: " << session.getTimeout().totalSeconds() << std::endl;
    std::cout << "connection time out: " <<session.getKeepAliveTimeout().totalSeconds()<< std::endl;

    HTTPRequest request(HTTPRequest::HTTP_GET, path, HTTPMessage::HTTP_1_1);
    request.setContentType(api_content_type_);
    request.setCredentials(api_credentials_scheme_, api_credentials_authentication_);

    HTTPResponse response;

    session.sendRequest(request);
    std::istream& rs = session.receiveResponse(response);

    respond_status_ = response.getStatus();

    if(respond_status_ == 200)
    {
        return true;
    }
    else
    {
        return false;
    }
}











