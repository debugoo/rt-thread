/**
* Copyright (C), 2015-2015, Chengdu QiPeng technology. Co., Ltd.
*   File name: dbg_uart.h
*      Author: Jun Xu
*     Version: 1.0.0
*        Date: Nov-12-2015
* Description: 
*              this file is the part of PHD.
*              this file implement debug serial port
*            
* Change Logs:
* Date            Author           Notes
* Nov-12-2015     Jun Xu           the first version
*/


#ifndef __DEBUG_UART_H
#define __DEBUG_UART_H     

#include <stm32f4xx.h>
#include <rtthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <rtdevice.h>

#ifdef OCB_OLD
/**
 * This function initialize debug serial port
 *
 * @return RT_EOK indicate this works succesfully
 */
int init_dbg_serial(void);

/**
 * This function output the debug text to debug serial port
 *
 * @param
 *
 * @return RT_EOK indicate this works succesfully
 */
void dbg_printf(const char *fmt, ...);

/**
 * This function write bytes to debug serial port
 *
 * @param
 *
 * @return RT_EOK indicate this works succesfully
 */
void dbg_write(const char* buf, int size);

#endif // OCB_OLD


/* output control block */
typedef struct _ocb_t {
    rt_device_t device;
    struct rt_mutex lock;
    const char* usart_name;
    char* buf;
    rt_size_t buf_sz;
} ocb_t;

/**
 * This function zero output control block
 *
 * @param       ocb_t*: the output control block
 *
 * @return void
 */
void ocb_zero(ocb_t* p);

/**
 * This function initialize output control block
 *
 * @param       ocb_t*: the output control block
 * @param  const char*: the name of usart
 * @param    rt_size_t: the buffer used by sprintf to storage output string
 *
 * @return int, 0 indicate operation successfully
 */
int ocb_init(ocb_t* p, const char* _usart_name, rt_size_t _buf_sz);

/**
 * This function free output control block
 *
 * @param       ocb_t*: the output control block
 *
 * @return void
 */
void ocb_free(ocb_t* p);

/**
 * This function output the text to usart from ocb
 *
 * @param       ocb_t*: the output control block
 * @param  const char*: the format string
 * @param          ...: the variable parameter
 *
 * @return void
 */
void ocb_printf(ocb_t* p, const char *fmt, ...);

/**
 * This function write bytes to usart from ocb
 *
 * @param       ocb_t*: the output control block
 * @param  const char*: the buffer from write to usart
 * @param          int: the number of bytes to write
 *
 * @return void
 */
void ocb_write(ocb_t* p, const char* buf, int size);

/**
 * This function output the text to usart from ocb (va_list version)
 *
 * @param       ocb_t*: the output control block
 * @param  const char*: the format string
 * @param      va_list: the variable parameter of va_list
 *
 * @return void
 # @marks
 *         because of the va_list cannot be transfer between functions,
 *         so, we need push va_list direct to this function
 */
void ocb_printf_var(ocb_t* p, const char *fmt, va_list args);
void ocb_printf_str(ocb_t* p, const char *buf);
#endif  
