#pragma once

#include "../include/net_common.h"
#include "net_tsqueue.h"
#include "net_message.h"
#include "net_connection.h"

namespace yf
{
    namespace net
    {
        template<typename T>
        class client_interface
        {
        public:

            client_interface(){}

            virtual ~client_interface()
            {
                // if the client is destroyed, always try and disconnect from sever
                Disconnect();
            }

        public:

            // 1. arm listen node: 192.168.2.29:5890
            // 2. mir rest api:
            bool Connect(const std::string& host, const uint16_t port)
            {
                try
                {
                    //todo:
                    // 0. Resolve hostname/ip-address into tangiable physical address
                    boost::asio::ip::tcp::resolver resolver(m_context);
                    boost::asio::ip::tcp::resolver::results_type endpoints = resolver.resolve(host, std::to_string(port));

                    //todo:
                    // 1. create connection
                    m_connection = std::make_unique<connection<T>>(
                           connection<T>::owner::client,
                           m_context,
                           boost::asio::ip::tcp::socket(m_context),m_qMessagesIn);

                    //todo:
                    // 2. tell the connection object to connect to sever. --- give the context some work to do. (work: read msg body form the server)
                    m_connection->ConnectToSever(endpoints);

                    //todo:
                    // 3. create a thread for each client
                    th_context = std::thread([this](){m_context.run();});
                }
                catch (std::exception& e)
                {
                    std::cerr << "Client Exception: " << e.what() << std::endl;
                    return false;
                }
                return false;
            }

            // Disconnect from sever -- Client shut down the connection
            void Disconnect()
            {
                // if connection exists, and it's connected then...
                if (isConnected())
                {
                    // ...disconnect from sever gracefully
                    m_connection->Disconnect();
                }

                // either way, we are also done with the asio context...
                m_context.stop();
                // ... and its thread
                if (th_context.joinable())
                    th_context.join();

                // destory the connection object
                m_connection.release();
            }

            // check if client is actually connected to a server
            bool isConnected()
            {
                if(m_connection)
                    return m_connection->isConnected();
                else
                    return false;
            }

        public:

            // Send message to server
            void Send(const message<T>& msg)
            {
                if(isConnected())
                {
                    m_connection->SendRawMsg(msg);
                }
            }

            // retrieve queue of message from sever
            tsqueue<owned_message<T>>& Incoming()
            {
                return m_qMessagesIn;
            }

        protected:
            // asio context handles the data transfer
            boost::asio::io_context m_context;
            // but it needs a thread of its own to execute it work commands
            std::thread th_context;
            // the client will have a single instance of a "connection" object,
            // which handles data transfer.
            std::unique_ptr<connection<T>> m_connection;

        private:
            // This is the thread safe queue of incoming message from server
            tsqueue<owned_message<T>> m_qMessagesIn;
        };
    }
}

