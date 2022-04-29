//
// Created by Kevin Williams on 4/28/22.
//

#include "src/lidar/lidar.h"

#if defined(_MACOS)
    const char * port = "/dev/tty.usbserial-0001";
#else
    const char * port = "/dev/ttyUSB0";
#endif
int main() {
    printf("hello");
    lidar_st lidar = lidar_init({port, 115200});
    lidar_scan(&lidar);
    return 0;
}