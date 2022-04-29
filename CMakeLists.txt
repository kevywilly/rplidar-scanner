cmake_minimum_required(VERSION 3.10.2)
project(rplidar_scanner)


set(CMAKE_CXX_STANDARD 14)

add_executable(rplidar_scanner main.cpp src/lidar/lidar.h)

add_library(rplidar SHARED main.cpp src/lidar/lidar.h )

target_include_directories(rplidar_scanner
        PUBLIC
        $ENV{RPLIDAR_SDK_PATH}
        $ENV{RPLIDAR_SDK_PATH}/include
        $ENV{RPLIDAR_SDK_PATH}/src
        )

target_include_directories(rplidar
        PUBLIC
        $ENV{RPLIDAR_SDK_PATH}
        $ENV{RPLIDAR_SDK_PATH}/include
        $ENV{RPLIDAR_SDK_PATH}/src
        )

SET(DEFAULT_SOURCES
        $ENV{RPLIDAR_SDK_PATH}/src/sl_lidar_driver.cpp
        $ENV{RPLIDAR_SDK_PATH}/src/hal/thread.cpp
        $ENV{RPLIDAR_SDK_PATH}/src/sl_crc.cpp
        $ENV{RPLIDAR_SDK_PATH}/src/sl_serial_channel.cpp
        $ENV{RPLIDAR_SDK_PATH}/src/sl_tcp_channel.cpp
        $ENV{RPLIDAR_SDK_PATH}/src/sl_udp_channel.cpp
        )
if(APPLE)
        SET(ARCH_SOURCES
        $ENV{RPLIDAR_SDK_PATH}/src/arch/macOS/net_serial.cpp
        $ENV{RPLIDAR_SDK_PATH}/src/arch/macOS/net_socket.cpp
        $ENV{RPLIDAR_SDK_PATH}/src/arch/macOS/timer.cpp
        )
else()
    SET(ARCH_SOURCES
            $ENV{RPLIDAR_SDK_PATH}/src/arch/linux/net_serial.cpp
            $ENV{RPLIDAR_SDK_PATH}/src/arch/linux/net_socket.cpp
            $ENV{RPLIDAR_SDK_PATH}/src/arch/linux/timer.cpp
            )
endif()
target_sources(rplidar_scanner
        PUBLIC
        ${DEFAULT_SOURCES}
        ${ARCH_SOURCES}
        )
target_sources(rplidar
        PUBLIC
        ${DEFAULT_SOURCES}
        ${ARCH_SOURCES}
        )

target_link_libraries(rplidar_scanner PUBLIC pthread stdc++)

target_link_libraries(rplidar PUBLIC pthread stdc++)



if(APPLE)
    add_compile_definitions(_MACOS)
endif()
