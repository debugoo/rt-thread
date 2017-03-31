/**
* Copyright (C), 2015-2015, Chengdu QiPeng technology. Co., Ltd.
*   File name: i2cx.h
*      Author: XuJun
*     Version: 1.0.0
*        Date: 10-05-2016
* Description: 
*              implements i2c driver with hardware i2c of ST
*              only implements master transmitter/receiver(not included slave func)
* Missed:
*              1. i2c slave
*              2. rt_device_control
*            
* Change Logs:
* Date            Author           Notes
* 10-05-2015      XuJun            the first version
*/

#ifndef __I2C_X_H__
#define __I2C_X_H__

#include <rtconfig.h>
#ifdef RT_USING_I2CX

#include <rtthread.h>
#include <stm32f4xx.h>
#include <rtdevice.h>

/* device mode */
#define RT_I2C_MODE_ADDR_10BIT  (1u << 0)
#define RT_I2C_MODE_IGNORE_NACK (1u << 1)
#define RT_I2C_MODE_NO_READ_ACK (1u << 2)

/* control command */
#define RT_I2C_DEV_CTRL_10BIT        0x20
#define RT_I2C_DEV_CTRL_ADDR         0x21
#define RT_I2C_DEV_CTRL_TIMEOUT      0x22
#define RT_I2C_DEV_CTRL_RW           0x23

#define RT_I2C_DEV_CTRL_GETERR       0x24
#define RT_I2C_DEV_CTRL_RESET        0x25
#define RT_I2C_DEV_CTRL_SYNC         0x26
#define RT_I2C_DEV_CTRL_SUSPEND      0x27
#define RT_I2C_DEV_CTRL_SETSEM       0x28
#define RT_I2C_DEV_CTRL_CONFIG       0x29

/* I2C SPEED */
#define RT_I2C_SPEED_STD            (100000)
#define RT_I2C_SPEED_FAST           (400000)


/* address */
#define I2C_ADDR_BIT_RX             (1<<0)

/* 32 bits error code: i2c result, the HSB bit value 1 when error occurs 
 * start from 200
 */
#define RT_EI2C                 200

#define I2C_E_PENDING           (RT_EI2C+1)



#define I2C_TRANS_FLAG_TX       (0x01)

struct rt_i2c_trans
{
    rt_uint8_t  slave_addr;
    rt_uint8_t  mode;
    rt_uint8_t  *buf;
    rt_uint16_t len;
    rt_uint16_t transferred;
};

struct i2c_configuration
{
    rt_uint8_t master_addr;
    rt_uint32_t speed;
    rt_tick_t timeout;
};

typedef void (*dma_config_fptr)(rt_device_t dev, rt_uint8_t *buf, rt_uint16_t size);
/*for i2c bus driver*/
struct rt_i2c_bus_device
{
    struct rt_device        parent;

    rt_uint8_t              addr;
    struct rt_semaphore     sem;
    struct rt_completion    completion;
    rt_err_t                error;
    struct i2c_configuration config;
    dma_config_fptr         dma_tx_conf;
    dma_config_fptr         dma_rx_conf;
    DMA_Stream_TypeDef*     dma_tx_stream;
    DMA_Stream_TypeDef*     dma_rx_stream;
    DMA_Stream_TypeDef*     dma_stream;

    I2C_TypeDef             *i2c;
    struct rt_i2c_trans     trans;
    rt_uint32_t             mode;
    struct rt_timer         tim;
};

#define I2C1_BUS_NAME        "i2c1"
#define I2C2_BUS_NAME        "i2c2"

#if defined(RT_USING_I2C1)
extern struct rt_i2c_bus_device i2c_bus1;
#endif

#if defined(RT_USING_I2C2)
extern struct rt_i2c_bus_device i2c_bus2;
#endif

rt_err_t rt_hw_i2c_init(rt_uint32_t mode, 
                        struct i2c_configuration* config);

#endif /*RT_USING_I2CX*/
#endif /*__I2C_X_H__*/
