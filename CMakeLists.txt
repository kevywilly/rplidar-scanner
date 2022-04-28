cmake_minimum_required(VERSION 3.22)
project(rplidar_scanner)


set(CMAKE_CXX_STANDARD 14)

add_executable(rplidar_scanner main.cpp)

target_include_directories(rplidar_scanner
        PUBLIC
        $ENV{RPLIDAR_SDK_PATH}
        $ENV{RPLIDAR_SDK_PATH}/include
        $ENV{RPLIDAR_SDK_PATH}/src
        )

target_sources(rplidar_scanner
        PUBLIC
        $ENV{RPLIDAR_SDK_PATH}/src/sl_lidar_driver.cpp
        $ENV{RPLIDAR_SDK_PATH}/src/arch/macOS/net_serial.cpp
        $ENV{RPLIDAR_SDK_PATH}/src/arch/macOS/net_socket.cpp
        $ENV{RPLIDAR_SDK_PATH}/src/arch/macOS/timer.cpp
        $ENV{RPLIDAR_SDK_PATH}/src/hal/thread.cpp
        $ENV{RPLIDAR_SDK_PATH}/src/sl_crc.cpp
        $ENV{RPLIDAR_SDK_PATH}/src/sl_serial_channel.cpp
        $ENV{RPLIDAR_SDK_PATH}/src/sl_tcp_channel.cpp
        $ENV{RPLIDAR_SDK_PATH}/src/sl_udp_channel.cpp

        )
target_link_libraries(rplidar_scanner PUBLIC pthread stdc++)

if(APPLE)
add_compile_definitions(_MACOS)
endif()