#pragma once

#include "../include/net_common.h"

namespace yf
{
    namespace net
    {
        template<typename T>
        struct message_header
        {
            //id ?? such as int: 1, 2, 3, ...
            T id{};
            // body size...
            uint32_t size = 512;
        };

        template<typename T>
        struct message
        {
            message_header<T> header{};

            std::vector<char> body{};

            size_t size() const
            {
                return sizeof(message_header<T>) + body.size();
            }

        };

        // Forward declare the connection
        template <typename T>
        class connection;

        template <typename T>
        struct owned_message
        {
            std::shared_ptr<connection<T>> remote = nullptr;
            message<T> msg;

        };

        struct message_net
        {
            std::vector<uint8_t> body;

            size_t size() const
            {
                return body.size();
            }
        };
    }
}
