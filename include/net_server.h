#pragma once

#include "../include/net_common.h"

namespace yf
{
    namespace net
    {
        template<typename T>
        class server_interface
        {
        public:
            // Create a server, ready to listen on specified port
            server_interface(uint16_t port)
            : m_asioAccpetor(m_asioContext,
                             boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), port))
                             {}
            virtual ~server_interface()
            {
                // May as well try and tidy up
                Stop();
            }

            // Starts the sever!
            bool Start()
            {
                // start listening??  -- start uploading some work for io_context to do and then run()..
                try
                {
                    // Issue a task to the asio context - This is important
                    // as it will prime the context with "work", and stop it
                    // from exiting immediately. Since this is a server, we
                    // want it primed ready to handle clients trying to
                    // connect.
                    WaitForClientConnection();

                    // launch the asio context in its own thread
                    m_threadContext = std::thread([this](){ m_asioContext.run();});
                }
                catch (std::exception& e)
                {
                    // Something prohibited the sever from listening
                    std::cerr << "[SERVER] Exception: " << e.what() << "\n";
                    return false;
                }

                std::cout << "[SERVER] Started!\n";
                return true;
            }

            // Stop the sever
            void Stop()
            {
                // Request the context to close
                m_asioContext.stop();

                // Tidy up the context thread
                if (m_threadContext.joinable())
                    m_threadContext.join();

                // Inform someone, anybody, if they care...
                std::cout << "[SERVER] Stopped!\n";
            }

            // ASYNC - Instruct asio to wait for connection - in this case, tm5 network.
            void WaitForClientConnection()
            {
                // Prime context with an instruction to wait until a socket connects. This
                // is the purpose of an "acceptor" object. It will provide a unique socket
                // for each incoming connection attempt
                // yf --- wait for connection...
                // yf --- if accept, will create a new socket...
                // and then the new socket will be moved to connection class instance.
                m_asioAccpetor.async_accept(
                        [this](std::error_code ec, boost::asio::ip::tcp::socket socket)
                        {
                            // Triggered by incoming connection request
                            if (!ec)
                            {
                                // Display user info
                                std::cout << "[SERVER] New Connection: " << socket.remote_endpoint() << "\n";

                                // Create a new connection to handle this client, move the new socket.
                                std::shared_ptr<connection<T>> newconn =
                                        std::make_shared<connection<T>>(connection<T>::owner::server,
                                                m_asioContext, std::move(socket), m_qMessagesIn);

                                // Give the user server a chance to deny connection
                                if (OnClientConnect(newconn))
                                {
                                    // Connection allowed, so add to container of new connections
                                    m_deqConnections.push_back(std::move(newconn));

                                    // todo: And very important! Issue a task to the connection's
                                    //  asio context to sit and wait for bytes to arrive!
                                    m_deqConnections.back()->ConnectToClient(nIDCounter++);

                                    std::cout << "[" << m_deqConnections.back()->GetID() << "] Connection Approved\n";
                                }
                                else
                                {
                                    std::cout << "[-----] Connection Denied\n";

                                    // Connection will go out of scope with no pending tasks, so will
                                    // get destroyed automagically due to the wonder of smart pointers
                                }
                            }
                            else
                            {
                                // Error has occurred during acceptance
                                std::cout << "[SERVER] New Connection Error: " << ec.message() << "\n";
                            }

                            // Prime the asio context with more work - again simply wait for
                            // another connection...
                            WaitForClientConnection();
                        }
                        );
            }

            // Send a message to a specific client
            void MessageClient(std::shared_ptr<connection<T>> client, const message<T>& msg)
            {
                // Check client is legitimate...
                if ( client && client->isConnected())
                {
                    // and then we will post the message via the connection
//                    client->Send(msg);
                    client->SendRawMsg(msg);
                }
                else
                {
                    // If we cant communicate with the client...
                    OnClientDisconnect(client);
                    client.reset();
                    m_deqConnections.erase(std::remove(m_deqConnections.begin(),m_deqConnections.end(),client), m_deqConnections.end());
                }
            }

            // todo:
            void MessageAllClients(const message<T>& msg, std::shared_ptr<connection<T>> pIgnoreClient = nullptr)
            {
                bool bInvalidClientExists = false;

                // Iterate through all clients in container
                for (auto& client : m_deqConnections)
                {
                    // Check client is connected...
                    if (client && client->isConnected())
                    {
                        // ..it is!
                        if(client != pIgnoreClient)
                            client->Send(msg);
                    }
                    else
                    {
                        // The client couldnt be contacted, so assume it has
                        // disconnected.
                        OnClientDisconnect(client);
                        client.reset();

                        // Set this flag to then remove dead clients from container
                        bInvalidClientExists = true;
                    }
                }

                // Remove dead clients, all in one go - this way, we dont invalidate the
                // container as we iterated through it.
                if (bInvalidClientExists)
                    m_deqConnections.erase(
                            std::remove(m_deqConnections.begin(), m_deqConnections.end(), nullptr), m_deqConnections.end());
            }

            // Force sever to respond to incoming message
            void Update(size_t nMaxMessages = -1, bool bWait = false)
            {
                if (bWait)
                    m_qMessagesIn.wait();

                // Process as many messages as you can up to the value
                // specified
                size_t nMessageCount = 0;
                while (nMessageCount < nMaxMessages && !m_qMessagesIn.is_empty())
                {
                    // Grab the front message
                    auto msg = m_qMessagesIn.pop_front();

                    // Pass to message handler
                    OnMessage(msg.remote, msg.msg);

                    nMessageCount++;
                }

            }


        protected:

            // the sever class should override these functions to implement
            // customised functionality

            // Called when a client connects, you can veto connection by return false;
            virtual bool OnClientConnect(std::shared_ptr<connection<T>> client)
            {
                return false;
            }

            // Called when a client appears to have disconnected
            virtual void OnClientDisconnect(std::shared_ptr<connection<T>> client)
            {

            }

            // Called when a message arrives
            virtual void OnMessage(std::shared_ptr<connection<T>> client, message<T>& msg)
            {

            }

        public:
            // Get First connection ID.
            uint32_t GetTheFirstConnectionID()
            {
                return m_deqConnections.back().get()->GetID();
            }

            // Get connection list!!!
            std::deque<std::shared_ptr<connection<T>>> GetConnectionList()
            {
                return m_deqConnections;
            }

//            void SetIdMap(const std::string& str,const uint32_t& id)
//            {
//                id_map.insert({str,id});
//            }

        protected:
            // Thread safe queue for incoming message packet.
            tsqueue<owned_message<T>> m_qMessagesIn = {};

            // Container of active validated connections
            std::deque<std::shared_ptr<connection<T>>> m_deqConnections;

            // Order of declaration is important - it is also the order of initialization
            boost::asio::io_context m_asioContext;
            std::thread m_threadContext;

            // These things need an asio context
            boost::asio::ip::tcp::acceptor m_asioAccpetor; // handles new incoming connection attempts...

            // Client will be identified in the "nw system" via an ID
            uint32_t nIDCounter = 10000;
            //
//            std::map<std::string, uint32_t> id_map;



        };
    }
}
