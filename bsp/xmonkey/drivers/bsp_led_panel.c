/*****************************************************************************
 This confidential and proprietary software may be only used as authorized
 by a licensing agreement from xujun.
 (C) COPYRIGHT 2015 . ALL RIGHTS RESERVED
 
 File           :    bsp_led_panel.c
 Author         :    xujun
 Date           :    2017/04/17
 Version        :    1.0
 Description    :    to be define
 History        :    
 
 *****************************************************************************/
#include <rtthread.h>
#include <led_panel.h>
#include "pinx.h"


/* led hw */
#define LED0_PIN PB4
#define LED1_PIN PB2
#define LED2_PIN PB3
#define LED3_PIN PB11

#define LED_COUNT  4
static const rt_uint16_t led_panel_pins[LED_COUNT] = {
    LED1_PIN, LED3_PIN, LED2_PIN, LED0_PIN 
};

static struct rt_timer led_panel_update_timer;
static rt_uint32_t led_panel_update_tick = 0;

void bsp_led_panel_turn_on(rt_uint8_t index)
{
    rt_pin_write(led_panel_pins[index], 0);
}
void bsp_led_panel_turn_off(rt_uint8_t index)
{
    rt_pin_write(led_panel_pins[index], 1);
}
static struct led_hw_ops led_panel_ops = {bsp_led_panel_turn_on, bsp_led_panel_turn_off};

static void led_panel_update_timeout(void* p)
{
    led_panel_update(led_panel_update_tick++);
}

void bsp_led_panel_init(void)
{
    int i = 0;
    for (i = 0; i < LED_COUNT; ++i)
    {
        rt_pin_mode(led_panel_pins[i], PIN_MODE_OUT_PP);
    }
    led_panel_init(&led_panel_ops);
    rt_timer_init(&led_panel_update_timer, "ledt", led_panel_update_timeout, RT_NULL, 10, RT_TIMER_FLAG_PERIODIC);
    rt_timer_start(&led_panel_update_timer);
}

