// // asio client testing (Listen node)

#include <cstdlib>
#include <deque>
#include <iostream>
#include <boost/bind/bind.hpp>
#include <boost/asio.hpp>
#include <thread>

#include "../include/chat_message.h"

using boost::asio::ip::tcp;

typedef std::deque<chat_message> chat_message_queue;

class chat_client
{
public:
    chat_client(boost::asio::io_context& io_context, const tcp::resolver::results_type& endpoints)
        : io_context_(io_context), socket_(io_context)
        {
            boost::asio::async_connect(socket_, endpoints,
                                       boost::bind(&chat_client::handle_connect, this,
                                                   boost::asio::placeholders::error));
        }
    void write(const chat_message& msg)
    {
        boost::asio::post(io_context_,boost::bind(&chat_client::do_write, this, msg));
    }
    void close()
    {
        boost::asio::post(io_context_, boost::bind(&chat_client::do_close,this));
    }

private:

    void handle_connect(const boost::system::error_code& error)
    {
        if(!error)
        {
            boost::asio::async_read(socket_,
                                    boost::asio::buffer(read_msg_.data(), chat_message::header_length),
                                    boost::bind(&chat_client::handle_read_header, this,
                                                boost::asio::placeholders::error));
            if (socket_.is_open()){
                std::cout << "connected!" << std::endl;
            }
        }

    }

    void handle_read_header(const boost::system::error_code& error)
    {
        if (!error && read_msg_.decode_header())
        {
            boost::asio::async_read(socket_,
                                    boost::asio::buffer(read_msg_.body(), read_msg_.body_length()),
                                    boost::bind(&chat_client::handle_read_body, this,
                                                boost::asio::placeholders::error));
        }
        else
        {
            do_close();
        }
    }

    void handle_read_body(const boost::system::error_code& error)
    {
        if (!error)
        {
            std::cout.write(read_msg_.body(), read_msg_.body_length());
            std::cout << "\n";
            boost::asio::async_read(socket_,
                                    boost::asio::buffer(read_msg_.data(), chat_message::header_length),
                                    boost::bind(&chat_client::handle_read_header, this,
                                                boost::asio::placeholders::error));
        }
        else
        {
            do_close();
        }
    }

    void do_write(chat_message msg)
    {
        bool write_in_progress = !write_msgs_.empty();
        write_msgs_.push_back(msg);
        if (!write_in_progress)
        {
            boost::asio::async_write(socket_,
                                     boost::asio::buffer(write_msgs_.front().data()+4,
                                                         write_msgs_.front().length()-4),
                                     boost::bind(&chat_client::handle_write, this,
                                                 boost::asio::placeholders::error));
        }
        std::cout << "write_msgs_.front().data()" << write_msgs_.front().data() << std::endl;
    }

    void handle_write(const boost::system::error_code& error)
    {
        if (!error)
        {
            write_msgs_.pop_front();
            if (!write_msgs_.empty())
            {
                boost::asio::async_write(socket_,
                                         boost::asio::buffer(write_msgs_.front().data(),
                                                             write_msgs_.front().length()),
                                         boost::bind(&chat_client::handle_write, this,
                                                     boost::asio::placeholders::error));
            }
        }
        else
            do_close();
    }

    void do_close()
    {
        socket_.close();
    }


private:
    boost::asio::io_context& io_context_;
    tcp::socket socket_;
    chat_message read_msg_;
    chat_message_queue write_msgs_;
};

int main(int argc, char* argv[])
{
    try
    {
        boost::asio::io_context io_context;

        tcp::resolver resolver(io_context);

//        boost::asio::ip::tcp::resolver::query query("192.168.2.29", "5890");
//        boost::asio::ip::tcp::resolver::iterator iter = resolver.resolve(query);
//        boost::asio::ip::tcp::endpoint endpoints = iter->endpoint();
//        std::cout << endpoints.address() << std::endl;
//        std::cin.get();

        tcp::resolver::results_type endpoints = resolver.resolve("192.168.2.29", "5890");

        chat_client c(io_context, endpoints);

        std::thread t(boost::bind(&boost::asio::io_context::run, &io_context));

        char line[chat_message::max_body_length + 1];

        while (std::cin.getline(line, chat_message::max_body_length + 1))
        {
            using namespace std; // For strlen and memcpy.
            chat_message msg;
            msg.body_length(strlen(line));
            std::memcpy(msg.body(), line, msg.body_length());
            msg.encode_header();
            c.write(msg);
        }

        c.close();
        t.join();


    }
    catch (std::exception& e)
    {
        std::cerr << "Exception: " << e.what() << "\n";
    }
    return 0;
}