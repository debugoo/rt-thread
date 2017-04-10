
/*****************************************************************************
 This confidential and proprietary software may be only used as authorized
 by a licensing agreement from xujun.
 (C) COPYRIGHT 2015 . ALL RIGHTS RESERVED
 
 File           :    led_panel.h
 Author         :    xujun
 Date           :    2017/04/07
 Version        :    1.0
 Description    :    to be define
 History        :    
 
 *****************************************************************************/

#ifndef __LED_PANEL_H
#define __LED_PANEL_H

#include <rtthread.h>

struct led_bit_ops
{
    void (*turnon)(rt_uint8_t idx);
    void (*turnoff)(rt_uint8_t idx);
};

extern void led_panel_init(rt_uint8_t *bits);
extern void led_panel_all_close(rt_uint8_t *bits);
extern void led_panel_all_open(rt_uint8_t *bits);
extern void led_panel_open(rt_uint8_t *bits, rt_uint8_t index);
extern void led_panel_close(rt_uint8_t *bits, rt_uint8_t index);
extern void led_panel_blink(rt_uint8_t index);

#endif