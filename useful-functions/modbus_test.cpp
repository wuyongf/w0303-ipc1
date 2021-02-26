#include "../include/modbus_tm_cllient.h"


// for time control.
#include <chrono>
#include <thread>



int main(void) {

#if 0
    modbus_t *mb;
    uint16_t tab_reg_03[32];
    uint16_t tab_reg_04[32];

    uint8_t tab_reg_8_01[16];
    uint8_t tab_reg_8_02[8];

    mb = modbus_new_tcp("192.168.2.29", 502);
    modbus_connect(mb);

    /* Read 5 registers from the address 0 */
    auto r1 = modbus_read_bits(mb, 0000, 10, tab_reg_8_01);
    auto r2 = modbus_read_input_bits(mb, 7201, 8, tab_reg_8_02);
    auto r3 = modbus_read_registers(mb, 7013, 10, tab_reg_03);
    auto r4 = modbus_read_input_registers(mb, 7013, 10, tab_reg_04);

    auto r5 = modbus_write_bit(mb, 0000, 0);


    std::cout << "r1: " << r1 << std::endl;
    std::cout << "r2: " << r2 << std::endl;
    std::cout << "r3: " << r3 << std::endl;
    std::cout << "r4: " << r4 << std::endl;

    modbus_close(mb);
//    modbus_free(mb);

    //wait 2s...
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    std::cout <<"next io command..." << std::endl;

    modbus_connect(mb);
    auto r6 = modbus_write_bit(mb, 0002, 0);

    modbus_close(mb);
    modbus_free(mb);

#endif

    std::cout <<"-----" << std::endl;

    yf::modbus::tm_modbus tm_modbus;

    const char* ip_address = "192.168.2.29";

    int port = 502;

    tm_modbus.Start(ip_address,port);

    while(false)
    {
        std::cout << "isError?" << tm_modbus.read_isError() << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    }

    tm_modbus.Close();

    std::cout <<"-----" << std::endl;

    return 1;
}