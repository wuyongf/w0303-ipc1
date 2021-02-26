#pragma once

#include <boost/algorithm/string/trim.hpp>

#include "../include/net.h"
#include "../include/net_w0303_common.h"

#include "../include/data.h"

class IPCClient : public yf::net::client_interface<CustomMsgTypes>
{
public:

    void SendMsg(const std::string& str)
    {
        yf::net::message<CustomMsgTypes> msg;

        msg.body.resize(str.size());
        msg.body.assign(str.begin(),str.end());

        Send(msg);
        // update the arm status, will block the code.
    }



};

