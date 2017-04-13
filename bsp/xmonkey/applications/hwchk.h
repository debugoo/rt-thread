/*****************************************************************************
 This confidential and proprietary software may be only used as authorized
 by a licensing agreement from xujun.
 (C) COPYRIGHT 2015 xujun. ALL RIGHTS RESERVED
 
 File           :    hwchk.h
 Author         :    xujun
 Date           :    2017/04/06
 Version        :    1.0
 Description    :    running it to confirm HW work correctly
 History        :    
 
 *****************************************************************************/


#ifndef __HWTEST_H
#define __HWTEST_H

#include <rtthread.h>


extern rt_err_t bus_init(void);
extern rt_err_t bus_deinit(void);


extern rt_err_t hw_check_mpu9250(void);

extern rt_err_t hw_check_bmp180(void);

extern rt_err_t hw_check_sht20(void);

extern rt_err_t hw_check_sim868(void);





#endif

