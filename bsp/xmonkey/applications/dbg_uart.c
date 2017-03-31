/**
* Copyright (C), 2015-2015, Chengdu QiPeng technology. Co., Ltd.
*   File name: dbg_uart.c
*      Author: Jun Xu
*     Version: 1.0.0
*        Date: Nov-12-2015
* Description: 
*              this file is the part of PHD.
*              this file implement debug uart port
*            
* Change Logs:
* Date            Author           Notes
* Nov-12-2015     Jun Xu           the first version
*/

#include "dbg_uart.h"
#include <string.h>

#ifdef OCB_OLD

static rt_device_t _usart_dev_dbg = RT_NULL;
#define CONSOLEBUF_SIZE_DBG (512)
#define USART_NAME_DBG  "uart2"

int init_dbg_serial() {
    _usart_dev_dbg = rt_device_find(USART_NAME_DBG);
    if (_usart_dev_dbg == RT_NULL)
    {
        return RT_EIO;
    }
    //return rt_device_open(_usart_dev_dbg, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_STREAM);
    return rt_device_open(_usart_dev_dbg, RT_DEVICE_FLAG_STREAM|RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX|RT_DEVICE_FLAG_INT_TX);
}
void dbg_write(const char* buf, int size) {
    rt_device_write(_usart_dev_dbg, 0, buf, size);
}
void dbg_printf(const char *fmt, ...)
{
    va_list args;
    rt_size_t length;
    rt_size_t write;
    static char x_log_buf[CONSOLEBUF_SIZE_DBG];

    va_start(args, fmt);
    /* the return value of vsnprintf is the number of bytes that would be
     * written to buffer had if the size of the buffer been sufficiently
     * large excluding the terminating null byte. If the output string
     * would be larger than the rt_log_buf, we have to adjust the output
     * length. */
    length = rt_vsnprintf(x_log_buf, sizeof(x_log_buf) - 1, fmt, args);
    if (length > CONSOLEBUF_SIZE_DBG - 1)
        length = CONSOLEBUF_SIZE_DBG - 1;

    {
        rt_uint16_t old_flag = _usart_dev_dbg->open_flag;

        //_usart_dev_dbg->open_flag |= RT_DEVICE_FLAG_STREAM;
        write = rt_device_write(_usart_dev_dbg, 0, x_log_buf, length);
        RT_ASSERT(write == length);
        _usart_dev_dbg->open_flag = old_flag;
    }
    va_end(args);
}
#endif // OCB_OLD


void ocb_zero(ocb_t* p) {
    p->device = RT_NULL;
    p->buf = RT_NULL;
    p->buf_sz = 0;
    p->usart_name = RT_NULL;
}

int ocb_init(ocb_t* p, const char* _usart_name, rt_size_t _buf_sz) {
    rt_err_t err;
    RT_ASSERT(p);
    RT_ASSERT(_usart_name);

    p->usart_name = _usart_name;
    p->buf_sz = _buf_sz;

    p->device = rt_device_find(p->usart_name);
    if (p->device == RT_NULL) {
        return RT_EIO;
    }
    err = rt_mutex_init(&p->lock, "DBGLK", RT_IPC_FLAG_FIFO);
    //RT_ASSERT(err == RT_EOK);
    if (err != RT_EOK)
    {
        return err;
    }

    p->buf = malloc(p->buf_sz);
    if (p->buf == RT_NULL) {
        return RT_ENOMEM;
    }
    //return rt_device_open(p->device, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_STREAM);
    return rt_device_open(p->device, RT_DEVICE_FLAG_STREAM|RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX|RT_DEVICE_FLAG_INT_TX);
}

void ocb_free(ocb_t* p) {
    if (RT_NULL != p->buf) {
        free(p->buf);
        p->buf = 0;
        p->buf_sz = 0;
    }
    if (RT_NULL != p->device) {
        rt_device_close(p->device);
        p->device = RT_NULL;
    }
    p->usart_name = RT_NULL;
}

double a, b,c;
void testxxx(ocb_t* p, va_list args) {

    //va_list args;
    //va_start(args, fmt);
    a = va_arg(args, double);
    b = va_arg(args, double);
    c = va_arg(args, double);
    //va_end(args);
}
double x, y, z;
char xxbuf[64];

void testxx(ocb_t* p, const char *fmt, ...) {

    va_list args;
    /*
    va_start(args, fmt);
    x = va_arg(args, double);
    y = va_arg(args, double);
    z = va_arg(args, double);
    va_end(args);
    */

    va_start(args, fmt);
    testxxx(p,  args);
    //sprintf(xxbuf, "%f, %f, %f\n", args);
    va_end(args);
}
void ocb_printf(ocb_t* p, const char *fmt, ...)
{
    va_list args;
    rt_size_t length;
    rt_size_t write;
    
    RT_ASSERT(RT_NULL != p->buf);
    RT_ASSERT(RT_NULL != p->device);

    va_start(args, fmt);
    /* the return value of vsnprintf is the number of bytes that would be
     * written to buffer had if the size of the buffer been sufficiently
     * large excluding the terminating null byte. If the output string
     * would be larger than the rt_log_buf, we have to adjust the output
     * length. */
    length = sprintf(p->buf, fmt, args);
    /*
    {
        rt_uint16_t old_flag = p->device->open_flag;

        p->device->open_flag |= RT_DEVICE_FLAG_STREAM;
        write = rt_device_write(p->device, 0, p->buf, strlen(p->buf));
        //RT_ASSERT(write == length);
        p->device->open_flag = old_flag;
    }
    */
    //length = rt_vsnprintf(p->buf, p->buf_sz - 1, fmt, args);
    if (length > p->buf_sz - 1)
        length = p->buf_sz - 1;

    {
        rt_uint16_t old_flag = p->device->open_flag;

        //p->device->open_flag |= RT_DEVICE_FLAG_STREAM;
        write = rt_device_write(p->device, 0, p->buf, length);
        RT_ASSERT(write == length);
        p->device->open_flag = old_flag;
    }
    va_end(args);
}
void ocb_printf_var(ocb_t* p, const char *fmt, va_list args)
{
    rt_size_t length;
    rt_size_t write;
    
    RT_ASSERT(RT_NULL != p->buf);
    RT_ASSERT(RT_NULL != p->device);

    /* the return value of vsnprintf is the number of bytes that would be
     * written to buffer had if the size of the buffer been sufficiently
     * large excluding the terminating null byte. If the output string
     * would be larger than the rt_log_buf, we have to adjust the output
     * length. */
    length = sprintf(p->buf, fmt, args);
    /*
    {
        rt_uint16_t old_flag = p->device->open_flag;

        p->device->open_flag |= RT_DEVICE_FLAG_STREAM;
        write = rt_device_write(p->device, 0, p->buf, strlen(p->buf));
        //RT_ASSERT(write == length);
        p->device->open_flag = old_flag;
    }
    */
    //length = rt_vsnprintf(p->buf, p->buf_sz - 1, fmt, args);
    if (length > p->buf_sz - 1)
        length = p->buf_sz - 1;

    {
        rt_uint16_t old_flag = p->device->open_flag;

        p->device->open_flag |= RT_DEVICE_FLAG_STREAM;
        write = rt_device_write(p->device, 0, p->buf, length);
        RT_ASSERT(write == length);
        p->device->open_flag = old_flag;
    }
}
void ocb_printf_str(ocb_t* p, const char *buf)
{
    rt_size_t length;
    rt_size_t write;
    
    RT_ASSERT(RT_NULL != p->buf);
    RT_ASSERT(RT_NULL != p->device);

    length = strlen(buf);
    RT_ASSERT(length <= p->buf_sz);

    if (length > p->buf_sz - 1)
        length = p->buf_sz - 1;

    {
        rt_uint16_t old_flag = p->device->open_flag;

        p->device->open_flag |= RT_DEVICE_FLAG_STREAM;
        write = rt_device_write(p->device, 0, p->buf, length);
        RT_ASSERT(write == length);
        p->device->open_flag = old_flag;
    }
}

void ocb_write(ocb_t* p, const char* buf, int size) {
    rt_mutex_take(&p->lock, RT_WAITING_FOREVER);
    RT_ASSERT(size < p->buf_sz);
    RT_ASSERT(RT_NULL != p->device);
    rt_device_write(p->device, 0, buf, size);
    rt_mutex_release(&p->lock);
}


