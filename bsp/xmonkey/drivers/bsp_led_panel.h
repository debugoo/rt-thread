/*****************************************************************************
 This confidential and proprietary software may be only used as authorized
 by a licensing agreement from xujun.
 (C) COPYRIGHT 2015 . ALL RIGHTS RESERVED
 
 File           :    bsp_led_panel.h
 Author         :    xujun
 Date           :    2017/04/17
 Version        :    1.0
 Description    :    to be define
 History        :    
 
 *****************************************************************************/
#ifndef __BSP_LED_PANEL__
#define __BSP_LED_PANEL__

extern void bsp_led_panel_init(void);
extern void bsp_led_panel_turn_on(rt_uint8_t index);
extern void bsp_led_panel_turn_off(rt_uint8_t index);


#endif

