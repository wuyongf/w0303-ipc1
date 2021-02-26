// simple sever .. tm network tcp/ip connection
// send and receive messages at any time...


// CustomServer for TM Network
// CustomClient for TM Listen Node


#include <iostream>

#include "../include/net.h"

enum class CustomMsgTypes : uint32_t
{
    ServerAccept,
    ServerOutput,
    BackToHomePosition,
    MoveToObstacleCheckingPosition,
    MoveToInitVisionPosition,
    MoveToCleanTaskInitPosition,
};

class IPCServer : public yf::net::server_interface<CustomMsgTypes>
{
public:
    IPCServer(uint16_t nPort) : yf::net::server_interface<CustomMsgTypes>(nPort)
    {

    }

protected:
    virtual bool OnClientConnect(std::shared_ptr<yf::net::connection<CustomMsgTypes>> client)
    {
        yf::net::message<CustomMsgTypes> msg;
        msg.header.id = CustomMsgTypes::ServerAccept;
        client->Send(msg);
        return true;
    }

    // Called when a client appears to have disconnected
    virtual void OnClientDisconnect(std::shared_ptr<yf::net::connection<CustomMsgTypes>> client)
    {
        std::cout << "Removing client [" << client->GetID() << "]\n";
    }

    // Called when a message arrives
    virtual void OnMessage(std::shared_ptr<yf::net::connection<CustomMsgTypes>> client, yf::net::message<CustomMsgTypes>& msg)
    {

        switch (msg.header.id)
        {
            case CustomMsgTypes::BackToHomePosition:
            {
                std::cout << "[" << client->GetID() << "]: Server Ping\n";

                // Simply bounce message back to client
                client->Send(msg);
            }
                break;

            case CustomMsgTypes::MoveToObstacleCheckingPosition:
            {
                std::cout << "[" << client->GetID() << "]: Message All\n";

                // Construct a new message and send it to tm5
                yf::net::message<CustomMsgTypes> msg;
                msg.header.id = CustomMsgTypes::MoveToObstacleCheckingPosition;

                MessageAllClients(msg, client);

            }
            break;
        }
    }
};

void th_server_go(IPCServer& server, bool& server_flag)
{

    server.Start();

    while (server_flag)
    {
        server.Update(1, true);
    }

    server.Stop();

}

int main()
{
    yf::net::message<CustomMsgTypes> msg;
    std::string str = "BackToHomePosition";
    msg.body.assign(str.begin(),str.end());

    bool sever_flag = true;

    IPCServer server(12345);

    std::cout << "set server flag True!" << std::endl;
    std::thread t1(th_server_go, std::ref(server), std::ref(sever_flag) );

    // pretend some work... 5s
    std::this_thread::sleep_for(std::chrono::milliseconds(10000));

    // server(IPC) send command to connection1.
    std::cout << "the arm connection ID: "<< server.GetConnectionList().front()->GetID() << std::endl;

    auto tm5 = server.GetConnectionList().front();

    server.MessageClient(tm5, msg);

    // pretend some work... 30s
    std::this_thread::sleep_for(std::chrono::milliseconds(30000));

    std::cout << "set server flag FLASE!" << std::endl;
    sever_flag = false;
    t1.join();

    std::cout << "good! thread finished its job" << std::endl;
    return 0;
}
