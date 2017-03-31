/**
* Copyright (C), 2015-2015, Chengdu QiPeng technology. Co., Ltd.
*   File name: BMP180.h
*      Author: xujun
*     Version: 1.0.0
*        Date: 16-06-2016
* Description: 
*              implement BMP180
*            
*   Reference:           
*            Based on datesheet "BMP180 Datasheet" provided by BOSCH
* 
* Change Logs:
* Date            Author           Notes
* 16-06-2016      xujun            the first version
*/

#include "bmp180.h"
#include "stm32f4xx.h"
#include <rtthread.h>
#include <math.h>
#include "i2cx.h"
#include "debug.h"

rt_uint16_t BMP180_ConvTime[4] = {BMP180_CONV_TIME_P_OSS_0,
                          BMP180_CONV_TIME_P_OSS_1,
                          BMP180_CONV_TIME_P_OSS_2,
                          BMP180_CONV_TIME_P_OSS_3};


rt_err_t BMP180_Get_EEPROM(struct BMP180_EEPROM* p)
{
    rt_device_t i2c = (rt_device_t)&i2c_bus1;
    rt_uint8_t reg;
    rt_uint8_t addr;
    rt_uint8_t eeprom[EEPROM_BYTES_LENGTH];
    int i;
    rt_uint16_t* outp;
    rt_size_t size;

    addr = BMP180_I2C_ADDRESS;
    reg = BMP180_REG_EEPROM;
    size = rt_device_write(i2c, addr, &reg, 1);
    if (size < 0)
        return size;
    if (size != 1)
        return -RT_ERROR;

    size = rt_device_read(i2c, addr, eeprom, EEPROM_BYTES_LENGTH);
    if (size < 0)
        return size;
    if (size != EEPROM_BYTES_LENGTH)
        return -RT_ERROR;

    outp = (rt_uint16_t*)p;
    for (i = 0; i < (EEPROM_BYTES_LENGTH/2); ++i)
    {
        outp[i] = eeprom[i*2];
        outp[i] <<= 8;
        outp[i] += eeprom[i*2+1];
    }
    return RT_EOK;
}

rt_err_t BMP180_Get_T(rt_uint16_t* UT)
{
    rt_device_t i2c = (rt_device_t)&i2c_bus1;
    rt_uint16_t tmp;
    rt_uint8_t addr;
    rt_uint8_t reg[3];
    rt_uint8_t meas[2];
    rt_uint8_t ctrl_meas = BMP180_SCO + BMP180_MEAS_T;
    rt_size_t size;

    addr = BMP180_I2C_ADDRESS;
    reg[0] = BMP180_REG_CTRL;
    reg[1] = ctrl_meas;
    reg[2] = 0xAA;
    size = rt_device_write(i2c, addr, &reg, 2);
    if (size < 0)
        return size;
    if (size != 2)
        return -RT_ERROR;

    rt_thread_delay(BMP180_CONV_TIME_T);
    reg[0] = BMP180_REG_OUT_MSB;
    size = rt_device_write(i2c, addr, &reg, 1);
    if (size < 0)
        return size;
    if (size != 1)
        return -RT_ERROR;
    
    size = rt_device_read(i2c, addr, meas, 2);
    if (size < 0)
        return size;
    if (size != 2)
        return -RT_ERROR;

    tmp = meas[0];
    tmp <<= 8;
    tmp |= meas[1];
    *UT = tmp;
    return RT_EOK;
}

rt_err_t BMP180_Get_P(rt_uint8_t oss, rt_uint32_t* UP)
{
    rt_device_t i2c = (rt_device_t)&i2c_bus1;
    rt_uint32_t tmp;
    rt_uint8_t addr;
    rt_uint8_t reg[2];
    rt_uint8_t meas[3];
    rt_uint8_t ctrl_meas = BMP180_SCO + (oss<<6) + BMP180_MEAS_P;
    XASSERT(BMP180_OSS_IS_VALID(oss));
    rt_size_t size;

    addr = BMP180_I2C_ADDRESS;
    reg[0] = BMP180_REG_CTRL;
    reg[1] = ctrl_meas;
    size = rt_device_write(i2c, addr, &reg, 2);
    if (size < 0)
        return size;
    if (size != 2)
        return -RT_ERROR;
    
    rt_thread_delay(BMP180_ConvTime[oss]);
    reg[0] = BMP180_REG_OUT_MSB;
    size = rt_device_write(i2c, addr, &reg, 1);
    if (size < 0)
        return size;
    if (size != 1)
        return -RT_ERROR;
    
    size = rt_device_read(i2c, addr, meas, 3);
    if (size < 0)
        return size;
    if (size != 3)
        return -RT_ERROR;

    tmp = meas[0];
    tmp <<= 8;
    tmp |= meas[1];
    tmp <<= 8;
    tmp |= meas[2];
    tmp >>= (8 - oss);
    *UP = tmp;
    return RT_EOK;
}

rt_int32_t BMP180_Caculate_B5(struct BMP180_EEPROM* ee, rt_int32_t UT)
{
    rt_int32_t B5 = 0;
    rt_uint16_t AC5 = ee->AC5;
    rt_uint16_t AC6 = ee->AC6;
    rt_int16_t MC = ee->MC;
    rt_int16_t MD = ee->MD;
    rt_int32_t X1 = (UT - (rt_int32_t)AC6)*((rt_int32_t)AC5) >> 15;
    rt_int32_t X2 = ((rt_int32_t)MC << 11) / (X1+(rt_int32_t)MD);
    B5 = X1 + X2;
    return B5;
}

float BMP180_Caculate_T(struct BMP180_Context* c, rt_int32_t UT)
{
    rt_int32_t B5;
    float temp;
    B5 = BMP180_Caculate_B5(c->ee, UT);
    c->B5 = B5;
    temp = (B5 + 8) >> 4;
    temp /= 10;
    return temp;
}

rt_int32_t BMP180_Caculate_P(struct BMP180_Context* c, rt_int32_t UP)
{
    rt_int16_t AC1 = c->ee->AC1;
    rt_int16_t AC2 = c->ee->AC2;
    rt_int16_t AC3 = c->ee->AC3;
    rt_uint16_t AC4 = c->ee->AC4;
    rt_int16_t B1 = c->ee->B1;
    rt_int16_t B2 = c->ee->B2;
    rt_int32_t B5 = c->B5;
    rt_uint8_t oss = c->oss;
    
    rt_int32_t X1, X2, X3;
    rt_int32_t B3, B6;
    rt_uint32_t B4, B7;
    rt_int32_t p;
    
    B6 = B5 - 4000;
    X1 = ((rt_int32_t)B2 * ((B6*B6) >> 12)) >> 11;
    X2 = ((rt_int32_t)AC2 * B6) >> 11;
    X3 = X1 + X2;
    B3 = ((((rt_int32_t)AC1*4 + X3)<<oss) + 2) / 4;
    
    X1 = ((rt_int32_t)AC3 * B6) >> 13;
    X2 = ((rt_int32_t)B1 * ((B6*B6)>>12)) >> 16;
    X3 = ((X1+X2) + 2) >> 2;
    B4 = ((rt_uint32_t)AC4 * (rt_uint32_t)(X3+32768)) >> 15;
    B7 = ((rt_uint32_t)UP - B3) * (rt_uint32_t)(50000UL >> oss);
    
    if (B7 < 0x80000000)
    {
        p = (B7 * 2) / B4;
    } else {
        p = (B7 / B4) * 2;
    }
    X1 = (p >> 8) * (p >> 8);
    X1 = (X1 * 3038) >> 16;
    X2 = (-7357 * p) >> 16;
    //p = p * (X1 + X2 + (rt_int32_t)3731) >> 4;
    p = p + ((X1 + X2 +(rt_int32_t)3791)>>4);
    return p;
}

float BMP180_Caculate_Altitude(rt_int32_t P)
{
    float a;
    float p0 = 101325.0;
    a = 44330.0*(1.0-pow(((float)P/p0), 1.0/5.255));
    return a;
}
