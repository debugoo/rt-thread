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

static struct led_hw_ops* _ops = RT_NULL;
#define LED_COUNT   (4)


struct bit_pattern
{
    rt_uint32_t bitmap;     /* 位模式串 */
    rt_uint8_t  pattern_sz; /* 位模式串长度 */
    rt_uint8_t  bit_width;  /* 位宽度，单位为节拍 */
};

static const struct bit_pattern pat_on      = {0x01, 1, 0xFF};
static const struct bit_pattern pat_off     = {0x00, 1, 0xFF};
static const struct bit_pattern pat_flip_1  = {0x01, 2, 0x1};
static const struct bit_pattern pat_flip_2  = {0x01, 2, 0x2};
static const struct bit_pattern pat_flip_4  = {0x01, 2, 0x4};
static const struct bit_pattern pat_flip_8  = {0x01, 2, 0x8};
static const struct bit_pattern pat_flip_16 = {0x01, 2, 0x10};
static const struct bit_pattern pat_flip_32 = {0x01, 2, 0x20};
static const struct bit_pattern pat_flip_64 = {0x01, 2, 0x40};
static const struct bit_pattern pat_fix_1   = {0x08, 4, 0x2};
static const struct bit_pattern pat_fix_2   = {0xCA, 8, 0x2};
static const struct bit_pattern pat_fix_3   = {0x01, 8, 0x2};
static const struct bit_pattern pat_fix_4   = {0x0F05, 16, 0x2}; /* 0000 1111 0000 0101 */

static const struct bit_pattern* single_mode[] =
{
    &pat_off,
    &pat_on,
    &pat_flip_1,
    &pat_flip_2,
    &pat_flip_4,
    &pat_flip_8,
    &pat_flip_16,
    &pat_flip_32,
    &pat_flip_64,
    &pat_fix_1,
    &pat_fix_2,
    &pat_fix_3,
    &pat_fix_4,
};


struct group_pattern
{
    struct bit_pattern led[LED_COUNT];
};
static const struct group_pattern group_1 = 
{ {
        {0x7F, 7, 2},
        {0x3E, 7, 2},
        {0x1C, 7, 2},
        {0x08, 7, 2},
} };
static const struct group_pattern group_2 = 
{ {
        {0x01, 2, 2},
        {0x02, 2, 2},
        {0x01, 2, 2},
        {0x02, 2, 2},
} };
        
static const struct group_pattern *group[] = 
{
    &group_1,
    &group_2,
};

/* 流水灯 */
struct bit_pattern pat_group_1[LED_COUNT] =
{
    {0x7F, 7, 2},
    {0x3F, 7, 2},
    {0x1C, 7, 2},
    {0x08, 7, 2},
};

const struct bit_pattern * pat_active[LED_COUNT];


#define IS_VALID_MODE(mode) \
    (mode >= LED_MODE_SWITCH_OFF && \
     mode <= LED_MODE_PATTERN_4)


void led_panel_init(struct led_hw_ops* ops)
{
    _ops = ops;

    pat_active[0] = &pat_off;
    pat_active[1] = &pat_off;
    pat_active[2] = &pat_off;
    pat_active[3] = &pat_off;
}

static void set_pattern(rt_uint8_t index, rt_bool_t on)
{
    on?_ops->turn_on(index):_ops->turn_off(index);
}
/* 单个更新
 */
static void led_panel_update_single(rt_uint32_t tick, rt_uint8_t index)
{
    const struct bit_pattern *pat = pat_active[index];
    /* expand pattern */
    rt_uint32_t pat_tick = pat->pattern_sz * pat->bit_width;
    /* alignment at pattern boundary */
    rt_uint32_t reminder = tick%pat_tick;
    if (reminder%pat->bit_width == 0)
    {
        /* 在位边界上,开始更新 */
        rt_uint8_t pos = reminder/pat->bit_width;
        rt_bool_t onoff = ((1<<pos)&pat->bitmap);
        set_pattern(index, onoff);
    }
}
static void led_panel_update_group(rt_uint32_t tick)
{
    rt_uint8_t i;
    for (i = 0; i < LED_COUNT; ++i)
    {
        led_panel_update_single(tick, i);
    }
}


void led_panel_update(rt_uint32_t tick)
{
    rt_uint8_t i;
    for (i = 0; i < LED_COUNT; ++i)
    {
        led_panel_update_single(tick, i);
    }
}

void led_panel_mode(rt_uint8_t image, rt_uint8_t mode)
{
    int i;
    if (!IS_VALID_MODE(mode))
        return;

    for (i = 0; i < LED_COUNT; ++i)
    {
        if ((image>>i)&1)
        {
            pat_active[i] = single_mode[mode];
        }
    }
}

#define IS_VALID_GROUP(group) \
    ((group>=LED_GROUP_1) && (group<=LED_GROUP_2))

void led_panel_group(rt_uint8_t index)
{
    int i = 0;
    const struct group_pattern* tmp;
    if (!IS_VALID_GROUP(index))
        return;
    tmp = group[index];
    for (i = 0; i < LED_COUNT; ++i)
    {
        pat_active[i] = &tmp->led[i];
    }
}

