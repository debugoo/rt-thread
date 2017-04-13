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
static rt_uint8_t _mode[LED_COUNT];

/* for custom bitmap */
static rt_bool_t _custom;
rt_uint8_t (*custom_get_bitmap)(rt_uint32_t tick);
rt_uint8_t default_custom_get_bitmap(rt_uint32_t tick)
{
    return 0;
}




#define IS_VALID_MODE_SWITCH(mode) \
    (mode == LED_MODE_SWITCH_OFF || mode == LED_MODE_SWITCH_ON)

#define IS_VALID_MODE_FLIP(mode) \
    (mode == LED_MODE_FLIP_1 || \
     mode == LED_MODE_FLIP_2 || \
     mode == LED_MODE_FLIP_4 || \
     mode == LED_MODE_FLIP_8 || \
     mode == LED_MODE_FLIP_16 || \
     mode == LED_MODE_FLIP_32 || \
     mode == LED_MODE_FLIP_64)

#define IS_VALID_MODE_PATTERN(mode) \
    (mode == LED_MODE_PATTERN_1 || mode == LED_MODE_PATTERN_2)

#define IS_VALID_MODE(mode) \
    (IS_VALID_MODE_SWITCH(mode) || \
     IS_VALID_MODE_FLIP(mode) || \
     IS_VALID_MODE_PATTERN(mode))


void led_panel_init(struct led_hw_ops* ops)
{
    _ops = ops;
    
    _mode[0] = LED_MODE_SWITCH_OFF;
    _mode[1] = LED_MODE_SWITCH_OFF;
    _mode[2] = LED_MODE_SWITCH_OFF;
    _mode[3] = LED_MODE_SWITCH_OFF;

    _custom = RT_FALSE;
    custom_get_bitmap = default_custom_get_bitmap;
}

/* 开关更新
 */
static void led_panel_update_switch(rt_uint8_t index, rt_uint8_t mode)
{
    if (mode == LED_MODE_SWITCH_OFF)
        _ops->turn_off(index);

    if (mode == LED_MODE_SWITCH_ON)
        _ops->turn_on(index);
}

/* 翻转更新
 * 在TICK上执行边界对齐 
 */
static void led_panel_update_flip(rt_uint32_t tick, rt_uint8_t index, rt_uint8_t mode)
{
    rt_uint32_t interval = 1<<(mode - LED_MODE_FLIP_1);
    rt_uint32_t T = interval<<1;
    if (tick % T == 0)
    {
        _ops->turn_on(index);
    }
    else if (tick % interval == 0)
    {
        _ops->turn_off(index);
    }
}

/* 模式更新
 * 在TICK上执行边界对齐 
 */
struct led_pattern
{
    rt_uint32_t pattern;
    rt_uint8_t len;
};
static struct led_pattern pat[2] = 
    {
        {0x08, 4}, /* for pattern: 1000 */
        //{0x02, 5}, /* for pattern: 00010 */
        {0xF0CC, 16}, /* for pattern: 1111 0000 1100 1100 */
    };
static void set_pattern(rt_uint8_t index, rt_bool_t onoff)
{
    onoff?_ops->turn_on(index):_ops->turn_off(index);
}
static void led_panel_update_pattern(rt_uint32_t tick, rt_uint8_t index, rt_uint8_t mode)
{
    int pattern_index = mode - LED_MODE_PATTERN_1;
    struct led_pattern* p = &pat[pattern_index];
    rt_uint32_t reminder = tick%(p->len);
    rt_bool_t onoff = ((1<<reminder)&p->pattern);
    set_pattern(index, onoff);
}

static struct led_pattern round_pat[4] = 
{
    {0xFF, 16},
    {0x3E, 16},
    {0x1C, 16},
    {0x08, 16},
};

static void led_panel_update_custom(rt_uint8_t bitmap)
{
    int i = 0;
    for (i = 0; i < LED_COUNT; ++i)
    {
        rt_uint8_t onoff = (bitmap>>i)&1;
        set_pattern(i, onoff);
    }
    /*
    int pattern_index = mode - LED_MODE_PATTERN_1;
    struct led_pattern* p = &round_pat[pattern_index];
    rt_uint32_t reminder = tick%(p->len);
    rt_bool_t onoff = ((1<<reminder)&p->pattern);
    set_pattern(index, onoff);
    */
}

void led_panel_update(rt_uint32_t tick)
{
    if (_custom == RT_TRUE)
    {
        led_panel_update_custom(custom_get_bitmap(tick));
    }
    else
    {
        rt_uint8_t i;
        for (i = 0; i < LED_COUNT; ++i)
        {
            if (IS_VALID_MODE_SWITCH(_mode[i]))
                led_panel_update_switch(i, _mode[i]);

            if (IS_VALID_MODE_FLIP(_mode[i]))
                led_panel_update_flip(tick, i, _mode[i]);

            if (IS_VALID_MODE_PATTERN(_mode[i]))
                led_panel_update_pattern(tick, i, _mode[i]);
        }
    }
}

void led_panel_mode(rt_uint8_t image, rt_uint8_t mode)
{
    if (!IS_VALID_MODE(mode))
        return;
    
    if (image&LED_IMAGE_0)
        _mode[0] = mode;
    
    if (image&LED_IMAGE_1)
        _mode[1] = mode;
    
    if (image&LED_IMAGE_2)
        _mode[2] = mode;
    
    if (image&LED_IMAGE_3)
        _mode[3] = mode;
}


void led_panel_custom(rt_uint8_t (*get_bitmap)(rt_uint32_t tick))
{
    if (get_bitmap == RT_NULL)
    {
        custom_get_bitmap = default_custom_get_bitmap;
        _custom = RT_FALSE;
        return;
    }
    custom_get_bitmap = get_bitmap;
    _custom = RT_TRUE;
}


