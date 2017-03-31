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


#ifndef __DEBUG_IMP_H
#define __DEBUG_IMP_H    

#include <stm32f4xx.h>
#include "dbg_uart.h"

/* ocb for debug */
extern ocb_t ocb_dbg;

int dbg_init(void);

void dbg_free(void);


#ifdef X_DBG_TO_UART
#define DBG_WRITE(buf, size) ocb_write(&ocb_dbg, buf, size)
#define DBG_PRINT(fmt, ...) { \
        ocb_dbg.buf[0] = 0; \
        snprintf(ocb_dbg.buf, ocb_dbg.buf_sz, fmt, ##__VA_ARGS__); \
        ocb_write(&ocb_dbg, ocb_dbg.buf, strlen(ocb_dbg.buf));\
    }
#else
#define DBG_WRITE(buf, size)
#define DBG_PRINT(fmt, ...)
#endif

#endif  
