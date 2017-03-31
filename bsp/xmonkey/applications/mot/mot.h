/**
 * Copyright (C), 2015-2015, Chengdu QiPeng technology. Co., Ltd.
 *   File name: mot.cpp
 *      Author: xujun
 *     Version: 1.0.0
 *        Date: 12-06-2016
 * Description: 
 *              implements test case of MPU9250
 *            
 *   Reference:           
 *              refer to <>
 * 
 * Change Logs:
 * Date            Author           Notes
 * 12-06-2016      xujun            created
 */

#ifndef __MOT_IMP_H
#define __MOT_IMP_H

#include <stm32f4xx.h>
#include <rtthread.h>

struct mot_data
{
    float acc;
    rt_uint32_t step_count;
    rt_uint32_t walk_time;
};
/**
 * This function start the test thread of motion processing
 *
 * @return UBLOX_SUCCESS indicate start successfully, otherwise occurs error
 */
int start_mot_thread(void);

void register_mot_notify(rt_err_t (*entry)(void));
void register_env_notify(rt_err_t (*entry)(void));



#endif
