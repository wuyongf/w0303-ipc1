// asio sever testing (network tcp/ip)

//example 1a
#include <iostream>
#include <string>
#include <vector>
#include <thread>
#include <chrono>

//error handling
#include <exception>

#include <boost/asio.hpp>
#include <boost/asio/ts/buffer.hpp>
#include <boost/asio/ts/internet.hpp>

using namespace boost::asio;

std::string RequestSomeData(){

    std::string str;
    std::cout << "Please enter some data... e.g. test2" << std::endl;
    getline(std::cin, str);
    str = str + "\r\n";
    std::cout << "string s: " << str << std::endl;
    return str;

}

std::vector<char> vBuffer(1 * 1024);
std::vector<uint8_t> Buffer(1 * 1024);

//start looping, keep waiting for msg
void GrabSomeData(ip::tcp::socket& socket)
{
    //keep listening msg come from tm5.
    socket.async_read_some(boost::asio::buffer(vBuffer.data(),vBuffer.size()),
                           [&](std::error_code ec, std::size_t length)
                           {
                                if(!ec)
                                {
                                    std::cout <<  "Read " << length <<" Bytes" << std::endl;

                                    for (int i = 0 ; i < length ; i++)
                                        std::cout << vBuffer[i];

                                    GrabSomeData(socket);
                                }
                           }
                           );
}

void thread_worker(boost::asio::io_context& io_context){
    std::cout << "thread start!" << std::endl;
    io_context.run();
    std::cout << "thread end!" << std::endl;
}

int main()
{
    boost::system::error_code ec;

    // asio specific interface.
    boost::asio::io_context io_context;
    // give some fake task to asio so the context doesnt finish.
    std::shared_ptr<boost::asio::io_context::work> work = std::make_shared<boost::asio::io_context::work>(io_context);
    // start the context, start handling the async handle?
    std::thread th_Context([&](){io_context.run();});

    // tm commmunication address

    ip::tcp::endpoint endpoint_tm_net_1(ip::make_address("192.168.2.29",ec), 12345);
    ip::tcp::endpoint endpoint_tm_net(ip::tcp::v4(), 12345); // connect mode: network, tm5-900 is client!
    ip::tcp::endpoint endpoint_tm_ln(ip::tcp::v4(), 5890);  // connect mode: LN. tm5-900 is sever!

    // create a socket
    ip::tcp::socket socket(io_context);

    // tell the socket to wait for connection ---- tm network
    ip::tcp::acceptor acceptor(io_context, endpoint_tm_net);
    // wait...
    acceptor.accept(socket,ec);

    if(!ec)
    {
        std::cout << "Connected!" << std::endl;
    }
    else
    {
        std::cout << "Failed to connect to the address:\n" << ec.message() << std::endl;
    }

    if(socket.is_open())     // if socke.is_open(), we can send the data directly. But TM needs some time to return msg
    {

        GrabSomeData(socket);

        //request "msg" to tm5
        std::string sRequest = RequestSomeData();

        socket.write_some(boost::asio::buffer(sRequest.data(),sRequest.size()),ec);

#if 0   // Read data manually
        // wait for there is data in socket
        socket.wait(socket.wait_read);

        size_t bytes = socket.available();

        std::cout << "Bytes Available: " << bytes << std::endl;

        if (bytes > 0)
        {
            std::vector<char> vBuffer(bytes);
            socket.read_some(boost::asio::buffer(vBuffer.data(),vBuffer.size()),ec);

            for(auto c : vBuffer)
            {
                std::cout << c;
            }
        }
#endif

        std::this_thread::sleep_for(std::chrono::milliseconds(2000)); // wait some time

    }

    system("pause");

    return 0;

}
// boost - example -non test
#if 0
#include <ctime>
#include <iostream>
#include <string>
#include <boost/asio.hpp>

using boost::asio::ip::tcp;

std::string make_daytime_string()
{
    using namespace std; // For time_t, time and ctime;
    time_t now = time(0);
    return ctime(&now);
}

int main()
{
    try
    {
        boost::asio::io_context io_context;

        tcp::acceptor acceptor(io_context, tcp::endpoint(tcp::v4(), 12345));

        for (;;)
        {
            tcp::socket socket(io_context);
            acceptor.accept(socket);

            std::string message = make_daytime_string();

            boost::system::error_code ignored_error;
            boost::asio::write(socket, boost::asio::buffer(message), ignored_error);
        }
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
    }

    return 0;
}
#endif
// boost - simple connection 1 (tested)
#if 0
#include <iostream>
#include <boost/asio.hpp>

using namespace boost::asio;
using ip::tcp;

std::string read_(tcp::socket & socket) {
    boost::asio::streambuf buf;
    boost::asio::read_until( socket, buf, "\n" );
    std::string data = boost::asio::buffer_cast<const char*>(buf.data());
    return data;
}

void send_(tcp::socket & socket, const std::string& message) {
    const std::string msg = message + "\n";
    boost::asio::write( socket, boost::asio::buffer(msg) );
}

int main() {

    try {
        boost::asio::io_service io_service;

        //listen for new connection
        tcp::acceptor acceptor_(io_service, tcp::endpoint(tcp::v4(), 12345));
        // socket creation
        tcp::socket socket_(io_service);
        //waiting for connection
        acceptor_.accept(socket_);

        //write operation
        send_(socket_, "test1");
        std::cout << "Servent sent Hello message to Client!" << std::endl;

        //read operation
        std::string message = read_(socket_);
        std::cout << message << std::endl;
    }

    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
    }

    return 0;
}
#endif

// boost - class connection example
#if 0
//importing libraries
#include <iostream>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/enable_shared_from_this.hpp>

using namespace boost::asio;
using ip::tcp;

class con_handler : public boost::enable_shared_from_this<con_handler>
{
private:
    tcp::socket sock;
    std::string message="Hello From Server!";
    enum { max_length = 1024 };
    char data[max_length];

public:
    typedef boost::shared_ptr<con_handler> pointer;
    con_handler(boost::asio::io_service& io_service): sock(io_service){}
// creating the pointer
    static pointer create(boost::asio::io_service& io_service)
    {
        return pointer(new con_handler(io_service));
    }
//socket creation
    tcp::socket& socket()
    {
        return sock;
    }

    void start()
    {
        sock.async_read_some(
                boost::asio::buffer(data, max_length),
                boost::bind(&con_handler::handle_read,
                            shared_from_this(),
                            boost::asio::placeholders::error,
                            boost::asio::placeholders::bytes_transferred));

        sock.async_write_some(
                boost::asio::buffer(message, max_length),
                boost::bind(&con_handler::handle_write,
                            shared_from_this(),
                            boost::asio::placeholders::error,
                            boost::asio::placeholders::bytes_transferred));
    }

    void handle_read(const boost::system::error_code& err, size_t bytes_transferred)
    {
        if (!err) {
            std::cout << data << std::endl;
        } else {
            std::cerr << "error: " << err.message() << std::endl;
            sock.close();
        }
    }
    void handle_write(const boost::system::error_code& err, size_t bytes_transferred)
    {
        if (!err) {
            std::cout << "Server sent Hello message!"<< std::endl;
        } else {
            std::cerr << "error: " << err.message() << std::endl;
            sock.close();
        }
    }
};

class Server
{
private:
    tcp::acceptor acceptor_;
    void start_accept()
    {
        // socket
        con_handler::pointer connection = con_handler::create(acceptor_.get_io_service());

        // asynchronous accept operation and wait for a new connection.
        acceptor_.async_accept(connection->socket(),
                               boost::bind(&Server::handle_accept, this, connection,
                                           boost::asio::placeholders::error));
    }
public:
//constructor for accepting connection from client
    Server(boost::asio::io_service& io_service): acceptor_(io_service, tcp::endpoint(tcp::v4(), 12345))
    {
        start_accept();
    }
    void handle_accept(con_handler::pointer connection, const boost::system::error_code& err)
    {
        if (!err) {
            connection->start();
        }
        start_accept();
    }
};

int main(int argc, char *argv[])
{
    try
    {
        boost::asio::io_service io_service;
        Server server(io_service);
        io_service.run();
    }
    catch(std::exception& e)
    {
        std::cerr << e.what() << endl;
    }
    return 0;
}
#endif

// tutorial 1 - time, sync
#if 0
#include <iostream>
#include <boost/asio.hpp>

int main(){

    boost::asio::io_context io;

    boost::asio::steady_timer t(io, boost::asio::chrono::seconds(5));

    t.wait();

    std::cout << "hallo" << std::endl;

    return 0;
}
#endif

//tutorial 2 - time, async
#if 0
#include <iostream>
#include <boost/asio.hpp>
void print(const boost::system::error_code& error){
    std::cout << "hallo" << std::endl;
}

int main(){
    boost::asio::io_context io;

    boost::asio::steady_timer t(io,boost::asio::chrono::seconds(10));

    t.async_wait(&print);

//    io.run(); // invoke the async function. however, t will wait the time first, then execute the callback function.
    std::cout << "1"<< std::endl;
    return 0;
}
#endif

//tutorial 3 - binding arguments to a handler.  (undone)
#if 0
#include <iostream>
#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>

#endif

// tutorial - boost tcp -1
#if 0
#include <ctime>
#include <iostream>
#include <string>
#include <boost/asio.hpp>

#include <chrono>
#include <thread>

using boost::asio::ip::tcp;

std::string make_daytime_string(){
    using namespace std;
    time_t now = time(0);
    return ctime(&now);
}

int main(){
    try
    {
        boost::asio::io_context io_context;
        tcp::acceptor acceptor(io_context, tcp::endpoint(tcp::v4(),12345));

        for(;;)
        {
            tcp::socket socket(io_context);
            acceptor.accept(socket);

            std::this_thread::sleep_for(std::chrono::milliseconds(1000));

            std::string message = make_daytime_string();
            boost::system::error_code ignored_error;
            boost::asio::write(socket, boost::asio::buffer(message), ignored_error);
            std::cout << "sent!" << std::endl;
        }
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
    }

    return 0;

}
#endif

// tutorial - boost tcp -2 async
#if 0

#include <ctime>
#include <iostream>
#include <string>
#include <boost/asio.hpp>

#include <chrono>
#include <thread>

using boost::asio::ip::tcp;

class tcp_server{

public:
    tcp_server(boost::asio::io_context& io_context)
        :io_context_(io_context),
         acceptor_(io_context,tcp::endpoint(tcp::v4(), 12345)
         {
             start_accept();
         }

private:
    void start_accept()
    {
        tcp_connection::pointer new_connection = tcp_connection::create(io_context_);
        acceptor_.async_accept(new_connection->socket(),
                               boost::bind(&tcp_server::handle_accept, this, new_connection,
                                           boost::asio::placeholders::error));
    }

    void handle_accept(tcp_connection::pointer new_connection,
                       const boost::system::error_code& error)
    {
        if (!error)
        {
            new_connection->start();
        }

        start_accept();
    }


    boost::asio::io_context& io_context_;
    tcp::acceptor acceptor_;
};

int main()
{
    try
    {
        boost::asio::io_context io_context;
        tcp_server server(io_context);
        io_context.run();
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
    }
}

#endif

// youtube asio tutorial.
#if 0

#include <iostream>

#include <boost/asio.hpp>
#include <boost/asio/ts/buffer.hpp>
#include <boost/asio/ts/internet.hpp>

int main()
{
    boost::system::error_code ec;

    boost::asio::io_context io_context;

    return 0;
}
#endif














