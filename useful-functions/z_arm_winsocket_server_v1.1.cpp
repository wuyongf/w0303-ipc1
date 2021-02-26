#include <iostream>
#include <WS2tcpip.h>

#pragma comment (lib, "ws2_32.lib")

//TODO: Connect with TM Robot via Network TCP/IP.
//todo: accpet any ip address but only the specific port "12345" client.

int main() 
{	
	std::cout << "main() start!!!" << std::endl;
	
	//Initiliaze winsock
	WSADATA wsaData;
	
	int iResult;

	iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
	if (iResult != 0) {
		printf("WSAStartup failed: %d\n", iResult);
		return 1;
	}
	
	//Create a socket
	SOCKET listening = socket(AF_INET, SOCK_STREAM, 0);
	if (listening == INVALID_SOCKET) 
	{
		std::cerr << "Can't create a socket" << std::endl;
		return 0;
	}
	
	//Bind an ip address and port to a socket
	sockaddr_in hint;
	hint.sin_family = AF_INET;
	hint.sin_port = htons(12345);
	hint.sin_addr.S_un.S_addr = INADDR_ANY;

	bind(listening, (sockaddr*)&hint, sizeof(hint));
	
	//Tell Winsock the socket is for listening
	listen(listening, SOMAXCONN);

	//wait for a connection, wait until there is a client.
	sockaddr_in client;
	int clientSize = sizeof(client);

	//keep waiting...
	SOCKET clientsocket = accept(listening, (sockaddr*)&client, &clientSize); 

	if (clientsocket == INVALID_SOCKET) 
	{
		std::cerr << "Can't create a clientsocket" << std::endl;
	}

	char host[NI_MAXHOST];		// client's remote name
	char service[NI_MAXHOST];	// service (i.e. port) the client is connect on

	ZeroMemory(host, NI_MAXHOST);
	ZeroMemory(service, NI_MAXSERV);

	if (getnameinfo((sockaddr*)&client, sizeof(client), host, NI_MAXHOST, service, NI_MAXSERV, 0) == 0)
	{
		std::cout << host << " connected on port " << service << std::endl;
	}
	else
	{
		inet_ntop(AF_INET, &client.sin_addr, host, NI_MAXHOST);
		std::cout << host << "connected on port " <<
			ntohs(client.sin_port) << std::endl;
	}

	//close listening socket // if we're going to accept other clients, don't close socket!
	closesocket(listening);

	//while loop: accept and echo message back to client
	char buf[4096];



	while(true)
	{
		ZeroMemory(buf, 4096);

		//wait flag_mv_TM
		do
		{
			std::cout << '\n' << "Press a 's' key to move TM...";
		} while (std::cin.get() != 's');

		//send message to TM
		char* buf1 = "Get isMoving123";
		send(clientsocket, buf1, sizeof(buf1)+7, 0);

		//wait for client's data
		int bytesReceived = recv(clientsocket, buf, 4096, 0);
		if (bytesReceived == SOCKET_ERROR) {
			std::cerr << "Error in recv(). Quiting..." << std::endl;
			break;

		}
		if (bytesReceived == 0) {
			std::cout << "Client disconnected..." << std::endl;
			break;
		}

		//check the data
		std::cout << "received message: " << buf << std::endl;
		
		////echo message back to client
		//send(clientsocket, buf, bytesReceived + 1, 0);
	}

	//close the socket 
	closesocket(clientsocket);

	//shutdown windsock
	WSACleanup();
}