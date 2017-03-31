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

#ifndef __BMP180_H
#define __BMP180_H

#include <stm32f4xx.h>
#include <rtthread.h>

/* I2C address */
#define BMP180_I2C_ADDRESS       (0xEE)

/* Registers */
#define BMP180_REG_ID            (0xD0)
#define BMP180_REG_RESET         (0xE0)
#define BMP180_REG_CTRL          (0xF4)
#define BMP180_REG_OUT_MSB       (0xF6)
#define BMP180_REG_OUT_LSB       (0xF7)
#define BMP180_REG_OUT_XLSB      (0xF8)
#define BMP180_REG_EEPROM        (0xAA)
#define BMP180_REG_IS_VALID(t) (t == BMP180_REG_ID || \
                                t == BMP180_REG_RESET || \
                                t == BMP180_REG_CTRL || \
                                t == BMP180_REG_OUT_MSB || \
                                t == BMP180_REG_OUT_LSB || \
                                t == BMP180_REG_OUT_XLSB || \
                                t == BMP180_REG_EEPROM \
                               )

/* Max conversion time, in us */
#define BMP180_CONV_TIME_T       (1)
    /* 4500 us */
#define BMP180_CONV_TIME_P_OSS_0 (1)
    /* 4500 us */
#define BMP180_CONV_TIME_P_OSS_1 (1)
    /* 7500 us */
#define BMP180_CONV_TIME_P_OSS_2 (2)
    /* 13500 us */
#define BMP180_CONV_TIME_P_OSS_3 (3)
    /* 25500 us */
#define BMP180_CONV_TIME_IS_VALID(t) (t == BMP180_CONV_TIME_T || \
                                      t == BMP180_CONV_TIME_P_OSS_0 || \
                                      t == BMP180_CONV_TIME_P_OSS_1 || \
                                      t == BMP180_CONV_TIME_P_OSS_2 || \
                                      t == BMP180_CONV_TIME_P_OSS_3 \
                                      )

/* EEPROM */
#define EEPROM_BYTES_LENGTH      (0x16)
extern rt_uint8_t BMP180_Data_EEPROM[EEPROM_BYTES_LENGTH];

/* Command, at bit<5> */
#define BMP180_SCO               (1<<5)

/* OSS: oversampling, at bit<7:6> */
#define BMP180_OSS_0             (0x00)
#define BMP180_OSS_1             (0x01)
#define BMP180_OSS_2             (0x02)
#define BMP180_OSS_3             (0x03)
#define BMP180_OSS_IS_VALID(oss) (oss == BMP180_OSS_0 || \
                                  oss == BMP180_OSS_1 || \
                                  oss == BMP180_OSS_2 || \
                                  oss == BMP180_OSS_3 \
                                  )
/* Measurement, at bit<4:0> */
#define BMP180_MEAS_T            (0x0E)
#define BMP180_MEAS_P            (0x14)
#define BMP180_MEAS_IS_VALID(oss) (oss == BMP180_MEAS_T || \
                                   oss == BMP180_MEAS_P)

#define BMP180_CTRL_MEAS_T       (BMP180_SCO | BMP180_MEAS_T)
#define BMP180_CTRL_MEAS_P_OSS0  (BMP180_SCO | BMP180_MEAS_P | BMP180_OSS_0)
#define BMP180_CTRL_MEAS_P_OSS1  (BMP180_SCO | BMP180_MEAS_P | BMP180_OSS_1)
#define BMP180_CTRL_MEAS_P_OSS2  (BMP180_SCO | BMP180_MEAS_P | BMP180_OSS_2)
#define BMP180_CTRL_MEAS_P_OSS3  (BMP180_SCO | BMP180_MEAS_P | BMP180_OSS_3)

/* EEPROM data */
struct BMP180_EEPROM
{
    rt_int16_t AC1;
    rt_int16_t AC2;
    rt_int16_t AC3;
    rt_uint16_t AC4;
    rt_uint16_t AC5;
    rt_uint16_t AC6;
    rt_int16_t B1;
    rt_int16_t B2;
    rt_int16_t MB;
    rt_int16_t MC;
    rt_int16_t MD;
};

struct BMP180_Context
{
    struct BMP180_EEPROM* ee;
    rt_int32_t B5; /* temp B5 */
    rt_uint8_t oss; /* oversampling */
};

/******************************************************************************/
/* Functions */

/**
  * @brief  Read EEPROM data
  * @param  void
  * @retval I2C_LastError.
  */
rt_err_t BMP180_Get_EEPROM(struct BMP180_EEPROM* p);

/**
  * @brief  Measure for temprature
  * @param  rt_uint16_t*: uncompensated temprature
  * @retval I2C_LastError.
  */
rt_err_t BMP180_Get_T(rt_uint16_t* UT);

/**
  * @brief  Measure for pressure by BMP180
  * @param  u8: oversampling
  *         rt_uint32_t*: uncompensated pressure
  * @retval I2C_LastError.
  */
rt_err_t BMP180_Get_P(u8 oss, rt_uint32_t* UP);

/******************************************************************************/
/* Formula of BMP180 */
/**
  * @brief  Caculate B5 from UT
  * @param  struct BMP180_EEPROM*: eeprom data
  *         rt_int32_t: uncompensated temprature
  * @retval float: physical temprature
  */
rt_int32_t BMP180_Caculate_B5(struct BMP180_EEPROM* ee, rt_int32_t UT);
/**
  * @brief  Caculate temprature from UT
  * @param  struct BMP180_Context*: context
  *         rt_int32_t: uncompensated temprature
  * @retval float: physical temprature
  */
float BMP180_Caculate_T(struct BMP180_Context* c, rt_int32_t UT);
/**
  * @brief  Caculate pressure from UP
  * @param  struct BMP180_Context*: context
  *         rt_int32_t: uncompensated pressure
  * @retval float: physical pressure
  */
rt_int32_t BMP180_Caculate_P(struct BMP180_Context* c, rt_int32_t UP);
/**
  * @brief  Caculate altitude from P
  * @param  struct BMP180_Context*: context
  *         rt_int32_t: pressure
  * @retval float: physical pressure
  */
float BMP180_Caculate_Altitude(rt_int32_t P);


#endif
