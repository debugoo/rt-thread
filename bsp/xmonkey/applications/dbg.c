/**
* Copyright (C), 2015-2015, Chengdu QiPeng technology. Co., Ltd.
*   File name: dbg_serial.c
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

#include "dbg.h"

/* ocb for debug */
ocb_t ocb_dbg;

#define OCB_CONSOLE_BUF_SIZE    (512)
const char* usart_name_1 =  "uart1";
const char* usart_name_2 =  "uart2";
const char* usart_name_3 =  "uart3";
const char* usart_name_4 =  "uart4";
const char* usart_name_5 =  "uart5";

int dbg_init(void) {
    int result;
    ocb_zero(&ocb_dbg);
    result = ocb_init(&ocb_dbg, usart_name_3, OCB_CONSOLE_BUF_SIZE);
    RT_ASSERT(result == RT_EOK);
    
    return result;
}
void dbg_free(void) {
    ocb_free(&ocb_dbg);
}

void dbg_print(const char* fmt, ...) {
  va_list args;
  va_start (args, fmt);
  //sprintf(ocb_dbg->buf, fmt, args);
  //ocb_printf_var(&ocb_dbg, fmt, args);
  va_end (args);
}
void dbg_write(const char* buf, int size) {
  ocb_write(&ocb_dbg, buf, size);
}


//#define DBG_PRINT(fmt, ...) ocb_printf(&ocb_dbg, fmt, ##__VA_ARGS__)
//#define DBG_WRITE(fmt, ...) ocb_write(&ocb_dbg, fmt, ##__VA_ARGS__)
