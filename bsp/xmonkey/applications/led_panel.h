
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

/* Hardware relative IO operations
 * Be the point of TDD insertion too
 */
struct led_hw_ops
{
    void (*turn_on)(rt_uint8_t idx);
    void (*turn_off)(rt_uint8_t idx);
    
    /* add hardware initialize interface */
};

/* LED working mode */
#define LED_MODE_SWITCH_OFF (0)
#define LED_MODE_SWITCH_ON  (1)

#define LED_MODE_FLIP_1     (2)  /* 翻转，1节拍：1010.... */
#define LED_MODE_FLIP_2     (3)  /* 翻转，2节拍：1100.... */
#define LED_MODE_FLIP_4     (4)  /* 翻转，4节拍：11110000.... */
#define LED_MODE_FLIP_8     (5)  /* 翻转，8节拍 */
#define LED_MODE_FLIP_16    (6)  /* 翻转，16节拍 */
#define LED_MODE_FLIP_32    (7)  /* 翻转，32节拍 */
#define LED_MODE_FLIP_64    (8)  /* 翻转，64节拍 */

#define LED_MODE_PATTERN_1  (9)  /* 模式，1000 */
#define LED_MODE_PATTERN_2  (10) /* 模式，11001010 */
#define LED_MODE_PATTERN_3  (11) /* 模式，00000001 */
#define LED_MODE_PATTERN_4  (12) /* 模式，00001111 00000101 */


/* LED IMAGE */
#define LED_IMAGE_0         (0x01)
#define LED_IMAGE_1         (0x02)
#define LED_IMAGE_2         (0x04)
#define LED_IMAGE_3         (0x08)
#define LED_IMAGE_ALL       (LED_IMAGE_0 | LED_IMAGE_1 | LED_IMAGE_2 | LED_IMAGE_3)

/* Group working mode */
#define LED_GROUP_1         (0)
#define LED_GROUP_2         (1)


/*
 * 驱动，每个节拍需要由外部线程/硬件定时器调用此函数更新LED状态
 * TDD测试驱动可采用连续调用来加快测试
 */
extern void led_panel_update(rt_uint32_t tick);

/*
 * 初始化
 * 置所有LED为关闭状态，会调用led_hw_ops直接输出
 */
extern void led_panel_init(struct led_hw_ops* ops);

/*
 * 设置工作模式
 * 一次可设置任意个LED
 */
extern void led_panel_mode(rt_uint8_t image, rt_uint8_t mode);

/*
 * 设置LED成组工作模式
 * 本模块内置了两个组工作模式
 * group_1: 流水灯
 * group_2: 交叉闪烁
 */
extern void led_panel_group(rt_uint8_t group);

extern void led_panel_on(rt_uint8_t image);
extern void led_panel_off(rt_uint8_t image);


#endif
