/**
* Copyright (C), 2015-2015, Chengdu QiPeng technology. Co., Ltd.
*   File name: dbg_serial.h
*      Author: Jun Xu
*     Version: 1.0.0
*        Date: Aug-04-2015
* Description: 
*              this file is the part of PHD.
*              this file implement debug serial port
*            
* Change Logs:
* Date            Author           Notes
* Aug-04-2015     Jun Xu           the first version
*/


#ifndef __DEBUG_MPU_LOG_H
#define __DEBUG_MPU_LOG_H    

#include <stm32f4xx.h>
#include "log.h"
#include "dbg.h"

typedef enum {
    PACKET_DATA_ACCEL = 0,
    PACKET_DATA_GYRO,
    PACKET_DATA_COMPASS,
    PACKET_DATA_QUAT,
    PACKET_DATA_EULER,
    PACKET_DATA_ROT,
    PACKET_DATA_HEADING,
    PACKET_DATA_LINEAR_ACCEL,
    NUM_DATA_PACKETS
} eMPL_packet_e;


int _MLPrintLog (int priority, const char* tag, const char* fmt, ...);
void eMPL_send_quat(long *quat);
void eMPL_send_data(unsigned char type, long *data);


#endif  
