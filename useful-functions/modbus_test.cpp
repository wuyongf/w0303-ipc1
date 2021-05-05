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
//
//    const char* ip_address = "192.168.7.29";
//
//    int port = 502;

    tm_modbus.Start("192.168.7.29",502);

    auto DI_0 = tm_modbus.get_end_module_DI(1);

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


    /// testing
    auto x1 = tm_modbus.get_robot_coordinate_param(7001,7002);

    std::deque<float> x_list;

    for (int n = 1; n < 31; n++ )
    {
        auto x = tm_modbus.get_robot_coordinate_param(7000+2*n-1,7000 + 2*n);

        x_list.push_back(x);
    }

/// For robot coordinate

// robot base and default tool
    auto x = tm_modbus.get_robot_coordinate_param(7037,7038);
    auto y = tm_modbus.get_robot_coordinate_param(7039,7040);
    auto z = tm_modbus.get_robot_coordinate_param(7041,7042);
    auto rx = tm_modbus.get_robot_coordinate_param(7043,7044);
    auto ry = tm_modbus.get_robot_coordinate_param(7045,7046);
    auto rz = tm_modbus.get_robot_coordinate_param(7047,7048);

    std::cout << "x: " << x << std::endl;
    std::cout << "y: " << y << std::endl;
    std::cout << "z: " << z << std::endl;
    std::cout << "rx: " << rx << std::endl;
    std::cout << "ry: " << ry << std::endl;
    std::cout << "rz: " << rz << std::endl;

// current base and default tool

    auto cur_x = tm_modbus.get_robot_coordinate_param(7001,7002);
    auto cur_y = tm_modbus.get_robot_coordinate_param(7003,7004);
    auto cur_z = tm_modbus.get_robot_coordinate_param(7005,7006);
    auto cur_rx = tm_modbus.get_robot_coordinate_param(7007,7008);
    auto cur_ry = tm_modbus.get_robot_coordinate_param(7009,7010);
    auto cur_rz = tm_modbus.get_robot_coordinate_param(7011,7012);

    std::cout << "----------" << std::endl;
    std::cout << "cur_x: " << cur_x << std::endl;
    std::cout << "cur_y: " << cur_y << std::endl;
    std::cout << "cur_z: " << cur_z << std::endl;
    std::cout << "cur_rx: " << cur_rx << std::endl;
    std::cout << "cur_ry: " << cur_ry << std::endl;
    std::cout << "cur_rz: " << cur_rz << std::endl;

// current base

    auto cb_x = tm_modbus.get_current_base_param(8300,8301);
    auto cb_y = tm_modbus.get_current_base_param(8302,8303);
    auto cb_z = tm_modbus.get_current_base_param(8304,8305);
    auto cb_rx = tm_modbus.get_current_base_param(8306,8307);
    auto cb_ry = tm_modbus.get_current_base_param(8308,8309);
    auto cb_rz = tm_modbus.get_current_base_param(8310,8311);

    std::cout << "----------" << std::endl;
    std::cout << "cb_x: " << cb_x << std::endl;
    std::cout << "cb_y: " << cb_y << std::endl;
    std::cout << "cb_z: " << cb_z << std::endl;
    std::cout << "cb_rx: " << cb_rx << std::endl;
    std::cout << "cb_ry: " << cb_ry << std::endl;
    std::cout << "cb_rz: " << cb_rz << std::endl;

#if 0

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

#endif




//    tm_modbus.read_isProjectRunning();

    tm_modbus.Close();

    std::cout << "-----" << std::endl;

    return 1;
}