# CMakeList.txt : CMake project for Arm_Control_Module_v1.0, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.17.5)

project (w0303)

#add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## opencv, for image processing
find_package(OpenCV REQUIRED)

include_directories( ${OpenCV_INCLUDE_DIRS} )

## nanodbc, for sql
find_package(nanodbc CONFIG REQUIRED)

## glog
set(GLOG_INCLUDE_DIR D:/software/include/glog_x64-windows/include/glog)
set(GLOG_LIBRARY_DIR D:/software/include/glog_x64-windows/lib)

find_package(glog 0.4.0 REQUIRED)

include_directories(${GLOG_INCLUDE_DIR})
link_directories(${GLOG_LIBRARY_DIR})

## modbus
find_package(libmodbus CONFIG REQUIRED)

## poco
find_package(Poco CONFIG REQUIRED Foundation Net XML Zip Data JSON)

## Eigen3 library
include_directories( "D:/software/include/Eigen3" )

## Boost library
if (1)

    set(BOOST_ROOT D:/software/include/boost_1_75_0)
    set(BOOSTROOT D:/software/include/boost_1_75_0)
    set(Boost_INCLUDE_DIR "D:/software/include/boost_1_75_0")
    set(Boost_LIBRARY_DIR D:/software/include/boost_1_75_0/stage/lib)

#find_package(Boost 1.75.0 REQUIRED COMPONENTS thread )
find_package(Boost 1.75.0 )

include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIR})

if(NOT Boost_FOUND)
    message(FATAL_ERROR "Could not find boost!")
endif()

add_definitions( -DBOOST_ALL_NO_LIB )

endif ()

# Add source to this project's executable.

## for w0303
add_executable (w0303_entrance "src/w0303_entrance.cpp" )

add_executable (w0303_entrance2 "src/w0303_entrance2.cpp" "include/net_w0303_server.h")
target_link_libraries(w0303_entrance2 PUBLIC glog::glog)

add_executable (w0303_entrance3 "src/w0303_entrance3.cpp")
target_link_libraries(w0303_entrance3 PUBLIC glog::glog nanodbc)

add_executable (sql "src/sql.cpp")
target_link_libraries(sql PUBLIC glog::glog nanodbc)

add_executable (manager_arm "src/manager_arm.cpp" )
##

add_executable (1_entrance "src/backup_1_entrance.cpp" )

add_executable (z_boost_tcpip "useful-functions/boost_tcpip.cpp" )

add_executable (z_winsocket_v1.0 "useful-functions/z_winsocket_server_v1.0.cpp" )

add_executable (z_winsocket_v1.1 "useful-functions/z_winsocket_server_v1.1.cpp" )

add_executable (z_move_semantics "useful-functions/z_move_semantics.cpp" )

add_executable (thread_promise_future "useful-functions/thread_promise_future.cpp" )
add_executable (thread_fuc_object "useful-functions/thread_fuc_object.cpp" )
add_executable (thread_data_racing "useful-functions/thread_data_racing.cpp" )
add_executable (thread_mutex_lock "useful-functions/thread_mutex_lock.cpp" )
add_executable (thread_monitor_object_pattern "useful-functions/thread_monitor_object_pattern.cpp" )
add_executable (thread_concurrent_msg_q "useful-functions/thread_concurrent_msg_q.cpp" )

add_executable (net_simple_server "src/net_simple_sever.cpp" )

add_executable (net_simple_client "src/net_simple_client.cpp" )

#tutorial: chat: client -- server -- msg
add_executable (z_boost_chat_client "useful-functions/boost_chat_client.cpp" )
#target_link_libraries(z_boost_chat_client ${Boost_LIBRARY_DIR})

add_executable (z_boost_server_tutorial "useful-functions/boost_server_tutorial.cpp" )
add_executable (mat_tutorial "useful-functions/eigen_mat_tutorial.cpp" )

add_executable (ln_command "useful-functions/w03003_ln_command.cpp" )
target_link_libraries(ln_command PRIVATE glog::glog)

add_executable (google_glog "useful-functions/w0303_google_glog.cpp" )
target_link_libraries(google_glog PRIVATE glog::glog)

add_executable (checksum_dec_to_hex "useful-functions/w0303_checksum_dec_to_hex.cpp" )
add_executable (w03003_cur_time "useful-functions/w03003_cur_time.cpp" )


## database. how to include .dll file
#d:/nw_sys/nw_sql_application_dll.dll
link_directories(lib)

add_executable(c_testing "useful-functions/chiu_c_testing.cpp")
target_link_libraries(c_testing "d:/nw_sys/nw_sql_application_dll.dll")

#add_executable(dll_libhallo "useful-functions/dll.cpp")
#target_link_libraries(dll_libhallo liblibhallo.dll)


##

add_executable(sql_testing "useful-functions/sql_testing.cpp")
target_link_libraries(sql_testing PRIVATE glog::glog nanodbc modbus Poco::Foundation Poco::Net Poco::XML Poco::Zip Poco::Data Poco::JSON)



add_executable(modbus_test "useful-functions/modbus_test.cpp")
target_link_libraries(modbus_test PRIVATE modbus)



add_executable(w0303_main_test1 "src/w0303_main_test1.cpp" "include/net_w0303_server.h")
target_link_libraries(w0303_main_test1 PUBLIC glog::glog nanodbc modbus Poco::Foundation Poco::Net Poco::XML Poco::Zip Poco::Data Poco::JSON)

add_executable(sql_ha_setting "useful-functions/sql_ha_setting.cpp" "include/net_w0303_server.h")
target_link_libraries(sql_ha_setting PUBLIC glog::glog nanodbc modbus Poco::Foundation Poco::Net Poco::XML Poco::Zip Poco::Data Poco::JSON)


add_executable(mir_poco "src/mir_poco.cpp" )
target_link_libraries(mir_poco PUBLIC glog::glog nanodbc modbus Poco::Foundation Poco::Net Poco::XML Poco::Zip Poco::Data)

# for mir testing
add_executable(mir_state "useful-functions/mir_state.cpp" )
target_link_libraries(mir_state PUBLIC glog::glog nanodbc modbus Poco::Foundation Poco::Net Poco::XML Poco::Zip Poco::Data Poco::JSON)

add_executable(mir_map "useful-functions/mir_map.cpp" )
target_link_libraries(mir_map PUBLIC glog::glog nanodbc modbus Poco::Foundation Poco::Net Poco::XML Poco::Zip Poco::Data Poco::JSON  ${OpenCV_LIBS})

add_executable(poco_ping_testing "useful-functions/poco_ping_testing.cpp" )
target_link_libraries(poco_ping_testing PUBLIC glog::glog nanodbc modbus Poco::Foundation Poco::Net Poco::XML Poco::Zip Poco::Data Poco::JSON  ${OpenCV_LIBS})


add_executable(hex2float "useful-functions/hex2float.cpp")

add_executable(arm_test "useful-functions/arm_test.cpp" "include/net_w0303_server.h")
target_link_libraries(arm_test PUBLIC glog::glog nanodbc modbus Poco::Foundation Poco::Net Poco::XML Poco::Zip Poco::Data Poco::JSON)


add_executable(str_parse_test1 "useful-functions/str_parse_test1.cpp" "include/net_w0303_server.h")
target_link_libraries(str_parse_test1 PUBLIC glog::glog nanodbc modbus Poco::Foundation Poco::Net Poco::XML Poco::Zip Poco::Data Poco::JSON)

add_executable(al_test "useful-functions/al_test.cpp")
target_link_libraries(al_test PUBLIC glog::glog nanodbc modbus Poco::Foundation Poco::Net Poco::XML Poco::Zip Poco::Data Poco::JSON)



# TODO: Add tests and install targets if needed.