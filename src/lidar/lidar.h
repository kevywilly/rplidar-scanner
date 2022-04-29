//
// Created by Kevin Williams on 4/28/22.
//

#ifndef RPLIDAR_SCANNER_LIDAR_H
#define RPLIDAR_SCANNER_LIDAR_H
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>

#include "sl_lidar.h"
#include "sl_lidar_driver.h"
#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#ifdef _WIN32
#include <Windows.h>
#define delay(x)   ::Sleep(x)
#else
#include <unistd.h>
static inline void delay(sl_word_size_t ms){
    while (ms>=1000){
        usleep(1000*1000);
        ms-=1000;
    };
    if (ms!=0)
        usleep(ms*1000);
}
#endif

using namespace sl;

typedef struct lidar_config_st {
    const char * port;
    const uint32_t baudrate;
} lidar_config_st ;

typedef struct lidar_st {
    const char * port;
    const uint32_t baudrate;
    IChannel* channel;
    ILidarDriver *drv;
    bool connected;
    sl_lidar_response_device_info_t devinfo;
} lidar_st;

bool ctrl_c_pressed;

void ctrlc(int)
{
    ctrl_c_pressed = true;
}

void lidar_print_info(lidar_st * lidar);

lidar_st lidar_init(lidar_config_st config) {
    lidar_st lidar {.port=config.port, .baudrate = config.baudrate};
    lidar.connected = false;
    lidar.channel = NULL;
    lidar.drv = *createLidarDriver();
    if (!lidar.drv) {
        fprintf(stderr, "insufficent memory, exit\n");
        return lidar;
    }
    sl_lidar_response_device_info_t devinfo;
    lidar.channel = (*createSerialPortChannel(lidar.port, lidar.baudrate));
    if (SL_IS_OK((lidar.drv)->connect(lidar.channel))) {
        if (SL_IS_OK(lidar.drv->getDeviceInfo(lidar.devinfo)))
        {
            lidar.connected = true;
        }
        else{
            lidar.connected = false;
            delete lidar.drv;
            lidar.drv = NULL;
        }
    }

    if (!lidar.connected) {
        (fprintf(stderr, "Error, cannot bind to the specified serial port %s with baurdrate %d.\n", lidar.port, lidar.baudrate));
        return lidar;
    }

   lidar_print_info(&lidar);

    return lidar;
}

void lidar_print_info(lidar_st * lidar) {

    if(!lidar->connected) {
        printf("Lidar is not connected");
        return;
    }
    printf("SLAMTEC LIDAR S/N: ");
    for (int pos = 0; pos < 16 ;++pos) {
        printf("%02X", lidar->devinfo.serialnum[pos]);
    }
    printf("\n"
           "Firmware Ver: %d.%02d\n"
           "Hardware Rev: %d\n"
            , lidar->devinfo.firmware_version>>8
            , lidar->devinfo.firmware_version & 0xFF
            , (int)lidar->devinfo.hardware_version);
}

int lidar_scan(lidar_st * lidar) {
    if(!lidar->connected) {
        (fprintf(stderr, "Lidar is not connected"));
        return -1;
    }
    signal(SIGINT, ctrlc);

    auto drv = lidar->drv;
    lidar->drv->setMotorSpeed();
    drv->startScan(0,1);
    while(1) {
        sl_lidar_response_measurement_node_hq_t nodes[8192];
        size_t   count = _countof(nodes);

        sl_result op_result = drv->grabScanDataHq(nodes, count);

        if (SL_IS_OK(op_result)) {
            drv->ascendScanData(nodes, count);
            for (int pos = 0; pos < (int)count ; ++pos) {
                printf("%s theta: %03.2f Dist: %08.2f Q: %d \n",
                       (nodes[pos].flag & SL_LIDAR_RESP_HQ_FLAG_SYNCBIT) ?"S ":"  ",
                       (nodes[pos].angle_z_q14 * 90.f) / 16384.f,
                       nodes[pos].dist_mm_q2/4.0f,
                       nodes[pos].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
            }
        }

        if (ctrl_c_pressed){
            break;
        }
    }
    return 0;
}
#endif //RPLIDAR_SCANNER_LIDAR_H
