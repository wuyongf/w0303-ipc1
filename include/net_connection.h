#pragma once

#include "net_common.h"
#include "net_tsqueue.h"
#include "net_message.h"

namespace yf
{
    namespace net
    {

        template<typename T>
        class connection : public std::enable_shared_from_this<connection<T>>
        {
        public:
            // A connection is "owned" by either a server or a client, and its
            // behaviour is slightly different bewteen the two.
            enum class owner
            {
                server,
                client
            };



        public:
            // Constructor: specify owner, connect to io_context, transfer the socket
            connection(owner parent, boost::asio::io_context& asioContext, boost::asio::ip::tcp::socket socket,
                       tsqueue<owned_message<T>>& qIn)
                       : m_asioContext(asioContext), m_socket(std::move(socket)), m_qMessagesIn(qIn)
                       {
                           m_nOwnerType = parent;
                       }

            virtual ~connection(){}

            // todo: client
            // This ID is used system wide - its how clients will understand other clients
            // exist across the whole system.
            uint32_t GetID() const
            {
                return id;
            }

        public:
            //only be called by server
            void ConnectToClient(uint32_t uid = 0)
            {
                if (m_nOwnerType == owner::server)
                {
                    if (m_socket.is_open())
                    {
                        id = uid;
                        m_msgTemporaryIn.body.resize(m_msgTemporaryIn.header.size);
                        ReadBody();
                    }
                }
            }

            // Client
            // only be called by clients
            void ConnectToSever(const boost::asio::ip::tcp::resolver::results_type& endpoints)
            {
                // Only clients can connect to servers
                if (m_nOwnerType == owner::client)
                {
                    // request asio attempts to connect to an endpoint
                    boost::asio::async_connect(m_socket, endpoints,
                                               [this](std::error_code ec, boost::asio::ip::tcp::endpoint endpoint)
                                               {
                                                    if(!ec)
                                                    {
                                                        ReadBody();
                                                    }
                                               });
                }
            }

            // can be called by both server and client
            // But i just saw called by client...
            void Disconnect()
            {
                if(isConnected())
                {
                    boost::asio::post(m_asioContext, [this]() { m_socket.close(); });
                }
            }
            bool isConnected() const
            {
                return m_socket.is_open();
            }

            // todo: Prime the connection to wait for incoming messages
            void StartListening()
            {

            }

        public:
            // ASYNC --- Send a message, connections are one-to-one so no need to specify
            // the target, for a client, the target is the server and vice versa
            void Send(const message<T>& msg)
            {
                boost::asio::post(m_asioContext,
                                  [this, msg]()
                                  {
                                      // If the queue has a message in it, then we must
                                      // assume that it is in the process of asynchronously being written.
                                      // Either way add the message to the queue to be output. If no messages
                                      // were available to be written, then start the process of writing the
                                      // message at the front of the queue.

                                      bool bWritingMessage = !m_qMessagesOut.is_empty();
                                      m_qMessagesOut.push_back(msg);
                                      // todo: e... don't get the point. But we will see the performance later.
                                      if (!bWritingMessage)
                                      {
                                          WriteHeader();
                                      }
                                  }
                                  );
            }

            void SendRawMsg (const message<T>& msg)
            {
                boost::asio::post(m_asioContext,
                                  [this, msg]()
                                  {
                                      // If the queue has a message in it, then we must
                                      // assume that it is in the process of asynchronously being written.
                                      // Either way add the message to the queue to be output. If no messages
                                      // were available to be written, then start the process of writing the
                                      // message at the front of the queue.

                                      bool bWritingMessage = !m_qMessagesOut.is_empty();
                                      m_qMessagesOut.push_back(msg); //add the msg!!!!!!
                                      // todo: e... don't get the point. But we will see the performance later.
                                      if (!bWritingMessage)
                                      {
                                          WriteTMBody();
                                      }
                                  }
                );
            }

        private:
            // ASYNC - Prime context to write a message header
            void WriteHeader()
            {
                // If this function is called, we know the outgoing message queue must have
                // at least one message to send. So allocate a transmission buffer to hold
                // the message, and issue the work - asio, send these bytes

                boost::asio::async_write(m_socket,
                                         boost::asio::buffer(&m_qMessagesOut.front().header, sizeof(message_header<T>)),
                                         [this](std::error_code ec, std::size_t length)
                                         {
                                             // asio has now sent the bytes - if there was a problem
                                             // an error would be available...
                                             if (!ec)
                                             {
                                                 // ... no error, so check if the message header just sent also
                                                 // has a message body...

                                                 if (m_qMessagesOut.front().body.size() > 0)
                                                 {
                                                     // ...it does, so issue the task to write the body bytes
                                                     WriteBody();
                                                 }
                                                 else
                                                 {
                                                     // ...it didnt, so we are done with this message. Remove it from
                                                     // the outgoing message queue
                                                     m_qMessagesOut.pop_front();

                                                     // If the queue is not empty, there are more messages to send, so
                                                     // make this happen by issuing the task to send the next header.
                                                     if (!m_qMessagesOut.is_empty())
                                                     {
                                                         WriteHeader();
                                                     }
                                                 }

                                             }
                                             else
                                             {
                                                 // ...asio failed to write the message, we could analyse why but
                                                 // for now simply assume the connection has died by closing the
                                                 // socket. When a future attempt to write to this client fails due
                                                 // to the closed socket, it will be tidied up.

                                                 std::cout << "[" << id << "] Write Header Fail.\n";
                                                 m_socket.close();
                                             }
                                         }
                                         );
            }

            // ASYNC - Prime context to write a message body

            void WriteBody()
            {
                // If this function is called, a header has just been sent, and that header
                // indicated a body existed for this message. Fill a transmission buffer
                // with the body data, and send it!

                boost::asio::async_write(m_socket, boost::asio::buffer(m_qMessagesOut.front().body.data(), m_qMessagesOut.front().body.size()),
                                         [this](std::error_code ec, std::size_t length)
                                         {
                                            if(!ec)
                                            {
                                                // Sending was successful, so we are done with the message
                                                // and remove it from the queue
                                                m_qMessagesOut.pop_front();

                                                // If the queue still has messages in it, then issue the task to
                                                // send the next messages' header.
                                                if (!m_qMessagesOut.is_empty())
                                                {
                                                    WriteHeader();
                                                }
                                            }
                                            else
                                            {
                                                // Sending failed, see WriteHeader() equivalent for description :P
                                                std::cout << "[" << id << "] Write Body Fail.\n";
                                                m_socket.close();
                                            }
                                         }
                                         );
            }

            void WriteTMBody()
            {
                // If this function is called, we will send the body msg to TM!!!!

                boost::asio::async_write(m_socket, boost::asio::buffer(m_qMessagesOut.front().body.data(), m_qMessagesOut.front().body.size()),
                                         [this](std::error_code ec, std::size_t length)
                                         {
                                             if(!ec)
                                             {
                                                 // Sending was successful, so we are done with the message
                                                 // and remove it from the queue
                                                 m_qMessagesOut.pop_front();

                                                 // If the queue still has messages in it, then issue the task to
                                                 // send the next messages' header.
                                                 if (!m_qMessagesOut.is_empty())
                                                 {
                                                     WriteTMBody();
                                                 }
                                             }
                                             else
                                             {
                                                 // Sending failed, see WriteHeader() equivalent for description :P
                                                 std::cout << "[" << id << "] Write Body Fail.\n";
                                                 m_socket.close();
                                             }
                                         }
                );
            }

            // ASYNC - Prime context ready to read a message header

            void ReadHeader()
            {
                // If this function is called, we are expecting asio to wait until it receives
                // enough bytes to form a header of a message. We know the headers are a fixed
                // size, so allocate a transmission buffer large enough to store it. In fact,
                // we will construct the message in a "temporary" message object as it's
                // convenient to work with.
                boost::asio::async_read(m_socket, boost::asio::buffer(&m_msgTemporaryIn.header, sizeof(message_header<T>)),
                                        [this](std::error_code ec, std::size_t length)
                                        {
                                            if (!ec)
                                            {

                                                // A complete message header has been read, check if this message
                                                // has a body to follow...
                                                if (m_msgTemporaryIn.header.size > 0)
                                                {
                                                    // ...it does, so allocate enough space in the messages' body
                                                    // vector, and issue asio with the task to read the body.
                                                    m_msgTemporaryIn.body.resize(m_msgTemporaryIn.header.size);
                                                    ReadBody();
                                                }
                                                else
                                                {
                                                    ReadBody();
                                                    // it doesn't, so add this bodyless message to the connections
                                                    // incoming message queue
                                                    AddToIncomingMessageQueue();
                                                }
                                            }
                                            else
                                            {
                                                // Reading form the client went wrong, most likely a disconnect
                                                // has occurred. Close the socket and let the system tidy it up later.
                                                std::cout << "[" << id << "] Read Header Fail.\n";
                                                m_socket.close();
                                            }
                                        }
                                        );
            }

            // ASYNC - Prime context ready to read a message body

            void ReadBody()
            {
                // If this function is called, a header has already been read, and that header
                // request we read a body, The space for that body has already been allocated
                // in the temporary message object, so just wait for the bytes to arrive...

#if 0
                boost::asio::async_read(m_socket, boost::asio::buffer(m_msgTemporaryIn.body.data(), m_msgTemporaryIn.body.size()),
                                 [this](std::error_code ec, std::size_t length)
                                 {
                                     std::cout <<"length: " <<length << std::endl;
                                     if (!ec)
                                     {
                                         for (int i = 0 ; i < length ; i++)
                                             std::cout <<m_msgTemporaryIn.body[i];

                                         // ...and they have! The message is now complete, so add
                                         // the whole message to incoming queue
                                         AddToIncomingMessageQueue();
                                     }
                                     else
                                     {
                                         // As above!
                                         std::cout << "[" << id << "] Read Body Fail.\n";
                                         m_socket.close();
                                     }
                                 });
#endif

                m_socket.async_read_some(boost::asio::buffer(m_msgTemporaryIn.body.data(), m_msgTemporaryIn.body.size()),
                                        [this](std::error_code ec, std::size_t length)
                                        {
                                            std::cout <<"[net_connection.h] ipc received length: " << length << std::endl;
                                            if (!ec)
                                            {
                                                std::cout << "[" << id << "] ---> IPC1: ";
                                                for (int i = 0 ; i < length ; i++)
                                                    std::cout <<m_msgTemporaryIn.body[i];
                                                // resize the vector based on the length..
                                                m_msgTemporaryIn.body.resize(length);
                                                // ...and they have! The message is now complete, so add
                                                // the whole message to incoming queue
                                                AddToIncomingMessageQueue();
                                            }
                                            else
                                            {
                                                // As above!
                                                std::cout << "[" << id << "] Read Body Fail.\n";
                                                m_socket.close();
                                            }
                                        });
            }

            // Once a full message is received, add it to the incoming queue

            void AddToIncomingMessageQueue()
            {
                // todo: figure out the shared pointer
                // Shove it in queue, converting it to an "owned message", by initialising
                // with the a shared pointer from this connection object
                if(m_nOwnerType == owner::server)
                {
                    m_qMessagesIn.push_back({ this->shared_from_this(), m_msgTemporaryIn });

                }

                else
                    m_qMessagesIn.push_back({ nullptr, m_msgTemporaryIn });

                m_msgTemporaryIn.body.resize(m_msgTemporaryIn.header.size);
//                ReadHeader();
                  ReadBody();
            }


        protected:
            // Each connection has a unique socket to a remote
            boost::asio::ip::tcp::socket m_socket;

            // This context is shared with the whole asio instance
            boost::asio::io_context& m_asioContext;

            // This queue holds all messages to be sent to the remote side of this connection
            tsqueue<message<T>> m_qMessagesOut;

            // This queue holds all messages that have been received from
            // the remote side of this connection. Note it is a reference
            // as the "owner" of this connection is expected to provide a queue.
            tsqueue<owned_message<T>>& m_qMessagesIn;

            // Incoming messages are constructed asynchronously, so we will
            // store the part assembled message here, until it is ready
            message<T> m_msgTemporaryIn;

            // The "owner" decide how some of the connection behaves
            owner m_nOwnerType = owner::server;

            uint32_t id  = 0;

            std::vector<char> vBuffer;

        };
    }
}
