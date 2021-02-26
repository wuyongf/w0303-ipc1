#pragma once
//glog
#include <glog/logging.h>

#include <stdio.h>
#include <iostream>
#include <modbus.h>

namespace yf
{
    namespace modbus
    {
        class tm_modbus
        {
        public:

            tm_modbus(){};
            virtual ~tm_modbus(){};

            void Start(const char *tm_ip_address, const int& port);
            void Close();

        public:

            void read_robot_status();
            bool read_isError();
            bool read_isProjectRunning();
            bool read_isPause();
            bool read_isEStop();

        public:

            void write_io(const int& io_port_no, const int& value);

        private:

            modbus_t*   mb;
            const char*       tm_ip_address_ = "192.168.2.29";
            int         port_ = 502;

            uint8_t     reg_robot_status[8];
        };
    }
}

void yf::modbus::tm_modbus::Start(const char *tm_ip_address, const int& port)
{
    try
    {
        tm_ip_address_ = tm_ip_address;
        port_ = port;
        mb = modbus_new_tcp(tm_ip_address_, port_);
//        LOG(INFO) << "Initialize tm_modbus_client.";
        std::cout << "Initialize tm_modbus_client." << std::endl;
    }
    catch (std::error_code er)
    {
        std::cerr << er << std::endl;
    }
}

void yf::modbus::tm_modbus::Close()
{
     modbus_free(mb);
//    LOG(INFO) << "Free tm_modbus_client.";
    std::cout << "Free tm_modbus_client." << std::endl;
}

void yf::modbus::tm_modbus::read_robot_status()
{
    modbus_connect(mb);

    modbus_read_input_bits(mb, 7201, 8, reg_robot_status);

    modbus_close(mb);
}

bool yf::modbus::tm_modbus::read_isError()
{
    read_robot_status();

    return reg_robot_status[0];
}

bool yf::modbus::tm_modbus::read_isProjectRunning()
{
    read_robot_status();

    return reg_robot_status[1];
}

bool yf::modbus::tm_modbus::read_isPause()
{
    read_robot_status();

    return reg_robot_status[3];
}

bool yf::modbus::tm_modbus::read_isEStop()
{
    read_robot_status();

    return reg_robot_status[7];
}




