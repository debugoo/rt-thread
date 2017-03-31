/**
* Copyright (C), 2015-2015, Chengdu QiPeng technology. Co., Ltd.
*   File name: SHT20.h
*      Author: xujun
*     Version: 1.0.0
*        Date: 16-06-2016
* Description: 
*              implement sht20
*            
*   Reference:           
*            Based on datesheet "Sensirion Humidity SHT20 Datasheet"
* 
* Change Logs:
* Date            Author           Notes
* 16-06-2016      xujun            the first version
*/

#include "sht20.h"
#include <stm32f4xx.h>
#include <stdio.h>
#include <string.h>
#include "i2cx.h"
#include <rtthread.h>

rt_err_t SHT20_Get_RH(float* RH)
{
    rt_device_t i2c = (rt_device_t)&i2c_bus1;
    rt_uint16_t tmp;
    rt_uint8_t cmd;
    rt_uint8_t addr;
    rt_uint8_t meas[2];
    rt_size_t size;

    addr = SHT20_I2C_ADDR;
    cmd = SHT20_CMD_MEAS_RH;
    size = rt_device_write(i2c, addr, &cmd, 1);
    if (size < 0)
        return size;
    if (size != 1)
        return -RT_ERROR;
    /* Wait for measurement for RH max = 29ms at 12 bit resolution */
    rt_thread_delay(5);
    
    size = rt_device_read(i2c, addr, meas, 2);
    if (size < 0)
        return size;
    if (size != 2)
        return -RT_ERROR;

    tmp = meas[0];
    tmp <<= 8;
    tmp |= meas[1];
    tmp &= SHT20_SIGNAL_MASK;
    *RH = SHT20_Caculate_RH((float)tmp);
    return RT_EOK;
}

rt_err_t SHT20_Get_T(float* T)
{
    rt_device_t i2c = (rt_device_t)&i2c_bus1;
    rt_uint16_t tmp;
    rt_uint8_t cmd;
    rt_uint8_t addr;
    rt_uint8_t meas[2];
    rt_size_t size;

    addr = SHT20_I2C_ADDR;
    cmd = SHT20_CMD_MEAS_T;
    size = rt_device_write(i2c, addr, &cmd, 1);
    if (size < 0)
        return size;
    if (size != 1)
        return -RT_ERROR;

    /* Wait for measurement for Tmax = 85ms at 14 bit resolution */
    rt_thread_delay(10);
    
    size = rt_device_read(i2c, addr, meas, 2);
    if (size < 0)
        return size;
    if (size != 2)
        return -RT_ERROR;

    tmp = meas[0];
    tmp <<= 8;
    tmp |= meas[1];
    tmp &= SHT20_SIGNAL_MASK;
    *T = SHT20_Caculate_T((float)tmp);
    return RT_EOK;
}

float SHT20_Caculate_RH(float signal)
{
    return SHT20_FORMULA_RH_A + SHT20_FORMULA_RH_B*(signal/SHT20_FORMULA_RH_C);
}

float SHT20_Caculate_T(float signal)
{
    return SHT20_FORMULA_TEMP_A + SHT20_FORMULA_TEMP_B*(signal/SHT20_FORMULA_TEMP_C);
}
