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

            // Arm Status
            //
            void read_robot_status();
            bool read_isError();
            bool read_isProjectRunning();
            bool read_isPause();
            bool read_isEStop();

            // Arm DI/O
            int get_control_box_DO(const int& DO);
            int get_control_box_DI(const int& DI);

            int get_end_module_DO(const int& DO);
            int get_end_module_DI(const int& DI);

            bool set_control_box_DO(const int& DO, const int& value);
            bool set_end_module_DO(const int& DO, const int& value);

            // Arm Coordinate
            float get_robot_coordinate_param(const int& high_16, const int& low_16);

            // Current Base
            float get_current_base_param(const int& high_16, const int& low_16);

        private:

            modbus_t*   mb;
            const char* tm_ip_address_ = "192.168.7.29";
            int         port_ = 502;

            uint8_t     reg_robot_status[8];

            uint8_t     reg_control_box_DO[16];
            uint8_t     reg_control_box_DI[16];

            uint8_t     reg_end_module_DO[4];
            uint8_t     reg_end_module_DI[3];

            uint16_t       reg_robot_coordinate[60];

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
        std::cerr << "tm_modbus Start() Failed!" << std::endl;
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

//@@ input: 0,1,2,3
//
int yf::modbus::tm_modbus::get_control_box_DO(const int &DO)
{
    modbus_connect(mb);

    modbus_read_bits(mb, 0000, 16, reg_control_box_DO);

    int DO_value = reg_control_box_DO[DO];

    modbus_close(mb);

    return DO_value;
}

int yf::modbus::tm_modbus::get_control_box_DI(const int &DI)
{
    modbus_connect(mb);

    modbus_read_input_bits(mb, 0000, 16, reg_control_box_DI);

    int DI_value = reg_control_box_DI[DI];

    modbus_close(mb);

    return DI_value;
}

int yf::modbus::tm_modbus::get_end_module_DO(const int &DO)
{
    modbus_connect(mb);

    modbus_read_bits(mb, 800, 4, reg_end_module_DO);

    int DO_value = reg_end_module_DO[DO];

    modbus_close(mb);

    return DO_value;
}

//@@ input: 0,1,2
//
int yf::modbus::tm_modbus::get_end_module_DI(const int &DI)
{
    modbus_connect(mb);

    modbus_read_input_bits(mb, 800, 3, reg_end_module_DI);

    int DI_value = reg_end_module_DI[DI];

    modbus_close(mb);

    return DI_value;
}

bool yf::modbus::tm_modbus::set_control_box_DO(const int &DO, const int &value)
{
    try
    {
        modbus_connect(mb);

        modbus_write_bit(mb,DO,value);

        modbus_close(mb);

        return true;
    }
    catch (std::error_code ec)
    {
        std::cerr << ec << std::endl;
        return false;
    }
}

bool yf::modbus::tm_modbus::set_end_module_DO(const int &DO, const int &value)
{
    auto DO_end = DO + 800;

    try
    {
        modbus_connect(mb);

        modbus_write_bit(mb,DO_end,value);

        modbus_close(mb);

        return true;
    }
    catch (std::error_code ec)
    {
        std::cerr << ec << std::endl;
        return false;
    }
}


#include "al_common.h"

//@@ input: 0 - 59
//
float yf::modbus::tm_modbus::get_robot_coordinate_param(const int& high_16, const int& low_16)
{

    modbus_connect(mb);

    modbus_read_input_registers (mb, 7001, 60, reg_robot_coordinate);

    uint16_t high16 = reg_robot_coordinate[high_16 - 7001];
    uint16_t low16 = reg_robot_coordinate[low_16 - 7001];

    modbus_close(mb);

    float x_f = unit32_to_float(high16,low16);

    return x_f;
}

float yf::modbus::tm_modbus::get_current_base_param(const int &high_16, const int &low_16)
{

    modbus_connect(mb);

    modbus_read_input_registers (mb, 8300, 60, reg_robot_coordinate);

    uint16_t high16 = reg_robot_coordinate[high_16 - 8300];
    uint16_t low16 = reg_robot_coordinate[low_16 - 8300];

    modbus_close(mb);

    float x_f = unit32_to_float(high16,low16);

    return x_f;
}




