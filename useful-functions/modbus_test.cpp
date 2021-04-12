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

    const char* ip_address = "192.168.7.29";

    int port = 502;

    tm_modbus.Start(ip_address,port);

//    auto DI_0 = tm_modbus.get_end_module_DI(1);

//    std::cout << "DI_0: " << DI_0 << std::endl;

#if 0

/// For control box IO

    /// get control box DI
    auto DI_0 = tm_modbus.get_control_box_DI(3);
    std::cout << "DO_0: " << DI_0 << std::endl;

    /// get control box DO
    auto DO_0 = tm_modbus.get_control_box_DO(0);
    std::cout << "DO_0: " << DO_0 << std::endl;

    /// set control box DO
    tm_modbus.set_control_box_DO(0,0);

#endif

/// For end module IO

    /// get end module DI
    auto DI_0_end = tm_modbus.get_end_module_DI(2);
    std::cout << "DI_2: " << DI_0_end << std::endl;

    /// get end module DO
    auto DO_0_end = tm_modbus.get_end_module_DO(3);
    std::cout << "DO_3: " << DO_0_end << std::endl;

    /// set end module DO
    tm_modbus.set_end_module_DO(0,0);
    tm_modbus.set_end_module_DO(1,0);



//    tm_modbus.read_isProjectRunning();

    tm_modbus.Close();

    std::cout <<"-----" << std::endl;

    return 1;
}