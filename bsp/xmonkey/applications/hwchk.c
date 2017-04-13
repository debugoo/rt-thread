/*****************************************************************************
 This confidential and proprietary software may be only used as authorized
 by a licensing agreement from xujun.
 (C) COPYRIGHT 2015 xujun. ALL RIGHTS RESERVED
 
 File           :    hwchk.c
 Author         :    xujun
 Date           :    2017/04/06
 Version        :    1.0
 Description    :    running it to confirm HW work correctly
 History        :    
 
 *****************************************************************************/

#include "hwchk.h"

static rt_device_t i2c_bus = RT_NULL;

#define I2C_ADDR_MPU9250    (0x68)

#define REG_MPU9250_WHOAMI  (0x75)

rt_err_t bus_init(void)
{
    /* open I2C bus */
    i2c_bus  = rt_device_find(I2C1_BUS_NAME);
    if (i2c_bus  == RT_NULL)
        return -RT_ERROR;
    err = rt_device_open(i2c_bus , RT_DEVICE_OFLAG_RDWR);
    return err;
}
rt_err_t bus_deinit(void)
{
    return rt_device_close(i2c_bus);
}


rt_err_t hw_check_mpu9250(void)
{
    rt_uint8_t tmp;
    rt_uint8_t addr;
    rt_size_t size;

    addr = I2C_ADDR_MPU9250;
    tmp = REG_MPU9250_WHOAMI;
    size = rt_device_write(i2c_bus, addr, &tmp, 1);
    if (size != 1)
        return -RT_ERROR;

    size = rt_device_read(i2c_bus, addr, &tmp, 1);
    if (size != 0)
        return -RT_ERROR;

    if (tmp != 0x71)
        return -RT_ERROR;
    
    return RT_EOK;
}

rt_err_t hw_check_bmp180(struct hw_check* p)
{
    return -RT_ERROR;
}

rt_err_t hw_check_sht20(struct hw_check* p)
{
    return -RT_ERROR;
}

rt_err_t hw_check_sim868(struct hw_check* p)
{
    return -RT_ERROR;
}


