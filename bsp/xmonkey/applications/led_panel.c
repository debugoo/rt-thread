/*****************************************************************************
 This confidential and proprietary software may be only used as authorized
 by a licensing agreement from .
 (C) COPYRIGHT 2015 . ALL RIGHTS RESERVED

 File           :    led_panel.c
 Author         :
 Date           :    2017/04/07
 Version        :    1.0
 Description    :    to be define
 History        :

 *****************************************************************************/
#include "led_panel.h"

void led_panel_init(rt_uint8_t *bits)
{
    *bits=0x00;
}
void led_panel_all_close(rt_uint8_t *bits)
{
    *bits=0x00;
}
void led_panel_all_open(rt_uint8_t *bits)
{
    *bits=0x0F;
}
void led_panel_open(rt_uint8_t *bits, rt_uint8_t index)
{
    if (index <= 3)
        *bits |= 1<<index;
}
void led_panel_close(rt_uint8_t *bits, rt_uint8_t index)
{
    if (index <= 3)
    *bits &= ~(1<<index);
}


