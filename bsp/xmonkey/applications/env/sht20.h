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

#ifndef __SHT20_H
#define __SHT20_H

#include <rtthread.h>

/* I2C address */
#define SHT20_I2C_ADDR           (0x40<<1)

/* Command */
#define SHT20_CMD_WAIT_MEAS_T    (0xE3)
#define SHT20_CMD_WAIT_MEAS_H    (0xE5)
#define SHT20_CMD_MEAS_T         (0xF3)
#define SHT20_CMD_MEAS_RH        (0xF5)
#define SHT20_CMD_W              (0xE6)
#define SHT20_CMD_R              (0xE7)
#define SHT20_CMD_RESET          (0xFE)

/* The last w bit is set to 0 */
#define SHT20_SIGNAL_MASK     (0xFFFC)

/*
 * Relative humidity formula
 *
 * X = Srh
 * T = -6 + 125 * (X / 2^16)
 *      |     |    |      |
 *      A     B    X      C
 */
#define SHT20_FORMULA_RH_A    (-6.0)
#define SHT20_FORMULA_RH_B    (125.0)
#define SHT20_FORMULA_RH_C    (1<<16)

/*
 * Temprature formula
 *
 * X = St
 * T = -46.85 + 175.72 * (X / 2^16)
 *          |        |    |      |
 *          A        B    X      C
 */
#define SHT20_FORMULA_TEMP_A    (-46.85)
#define SHT20_FORMULA_TEMP_B    (175.72)
#define SHT20_FORMULA_TEMP_C    (1<<16)

/**
  * @brief  Measure for relative humidity
  * @param  void
  * @retval rt_err_t.
  */
rt_err_t SHT20_Get_RH(float* RH);

/**
  * @brief  Measure for temprature
  * @param  void
  * @retval rt_err_t.
  */
rt_err_t SHT20_Get_T(float* T);

/**
  * @brief  Caculate RH Result from RH Signal
  * @param  float: signal of RH
  * @retval rt_err_t.
  */
float SHT20_Caculate_RH(float signal);

/**
  * @brief  Caculate T Result from T Signal
  * @param  float: signal of T
  * @retval rt_err_t.
  */
float SHT20_Caculate_T(float signal);

#endif
