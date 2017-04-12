/*****************************************************************************
 This confidential and proprietary software may be only used as authorized
 by a licensing agreement from .
 (C) COPYRIGHT 2015 . ALL RIGHTS RESERVED

 File           :    led_panel_utests.c
 Author         :
 Date           :    2017/04/07
 Version        :    1.0
 Description    :    to be define
 History        :

 *****************************************************************************/

#include "CppUTest/TestHarness.h"

extern "C"
{
#include "rtthread.h"
#include "led_panel.h"

static rt_uint8_t led_image;

struct image_counter
{
    rt_uint32_t on;
    rt_uint32_t off;
};
static struct image_counter cnt[4];

static void turn_on(rt_uint8_t idx)
{
    led_image |= 1<<idx;
    cnt[idx].on ++;
}
static void turn_off(rt_uint8_t idx)
{
    led_image &= ~(1<<idx);
    cnt[idx].off ++;
}

}

TEST_GROUP(led_panel)
{
struct led_hw_ops ops;
void setup()
{
    int i;
    ops.turn_on = turn_on;
    ops.turn_off = turn_off;
    led_image = 0x0F;
    for (i = 0; i < 4; ++i)
    {
        cnt[i].off = 0;
        cnt[i].on = 0;
    }
    
    led_panel_init(&ops);
}
void teardown()
{
}
};

TEST(led_panel, init)
{
    led_image = 0x0F;
    led_panel_init(&ops);
    led_panel_update(0);
    BYTES_EQUAL(0x00, led_image);
}

/* Invalid mode */
TEST(led_panel, invalid_mode)
{
    led_panel_mode(LED_IMAGE_0, 0xFF);
    led_panel_update(0);
    BYTES_EQUAL(0x00, led_image);
}

/* switch on one by one */
TEST(led_panel, switch_on_led0)
{
    led_panel_mode(LED_IMAGE_0, LED_MODE_SWITCH_ON);
    led_panel_update(0);
    BYTES_EQUAL(0x01, led_image);
}

TEST(led_panel, switch_on_led1)
{
    led_panel_mode(LED_IMAGE_1, LED_MODE_SWITCH_ON);
    led_panel_update(0);
    BYTES_EQUAL(0x02, led_image);
}

TEST(led_panel, switch_on_led2)
{
    led_panel_mode(LED_IMAGE_2, LED_MODE_SWITCH_ON);
    led_panel_update(0);
    BYTES_EQUAL(0x04, led_image);
}

TEST(led_panel, switch_on_led3)
{
    led_panel_mode(LED_IMAGE_3, LED_MODE_SWITCH_ON);
    led_panel_update(0);
    BYTES_EQUAL(0x08, led_image);
}

/* switch on combined */
TEST(led_panel, switch_on_led01)
{
    led_panel_mode(LED_IMAGE_0|LED_IMAGE_1, LED_MODE_SWITCH_ON);
    led_panel_update(0);
    BYTES_EQUAL(0b11, led_image);
}

TEST(led_panel, switch_on_led02)
{
    led_panel_mode(LED_IMAGE_0|LED_IMAGE_2, LED_MODE_SWITCH_ON);
    led_panel_update(0);
    BYTES_EQUAL(0b101, led_image);
}

TEST(led_panel, switch_on_led03)
{
    led_panel_mode(LED_IMAGE_0|LED_IMAGE_3, LED_MODE_SWITCH_ON);
    led_panel_update(0);
    BYTES_EQUAL(0b1001, led_image);
}

TEST(led_panel, switch_on_led12)
{
    led_panel_mode(LED_IMAGE_1|LED_IMAGE_2, LED_MODE_SWITCH_ON);
    led_panel_update(0);
    BYTES_EQUAL(0b110, led_image);
}

TEST(led_panel, switch_on_led13)
{
    led_panel_mode(LED_IMAGE_1|LED_IMAGE_3, LED_MODE_SWITCH_ON);
    led_panel_update(0);
    BYTES_EQUAL(0b1010, led_image);
}

TEST(led_panel, switch_on_led23)
{
    led_panel_mode(LED_IMAGE_2|LED_IMAGE_3, LED_MODE_SWITCH_ON);
    led_panel_update(0);
    BYTES_EQUAL(0b1100, led_image);
}

TEST(led_panel, switch_on_led012)
{
    led_panel_mode(LED_IMAGE_0|LED_IMAGE_1|LED_IMAGE_2, LED_MODE_SWITCH_ON);
    led_panel_update(0);
    BYTES_EQUAL(0b111, led_image);
}

TEST(led_panel, switch_on_led013)
{
    led_panel_mode(LED_IMAGE_0|LED_IMAGE_1|LED_IMAGE_3, LED_MODE_SWITCH_ON);
    led_panel_update(0);
    BYTES_EQUAL(0b1011, led_image);
}

TEST(led_panel, switch_on_led023)
{
    led_panel_mode(LED_IMAGE_0|LED_IMAGE_2|LED_IMAGE_3, LED_MODE_SWITCH_ON);
    led_panel_update(0);
    BYTES_EQUAL(0b1101, led_image);
}

TEST(led_panel, switch_on_led123)
{
    led_panel_mode(LED_IMAGE_1|LED_IMAGE_2|LED_IMAGE_3, LED_MODE_SWITCH_ON);
    led_panel_update(0);
    BYTES_EQUAL(0b1110, led_image);
}

TEST(led_panel, switch_on_led0123)
{
    led_panel_mode(LED_IMAGE_0|LED_IMAGE_1|LED_IMAGE_2|LED_IMAGE_3, LED_MODE_SWITCH_ON);
    led_panel_update(0);
    BYTES_EQUAL(0b1111, led_image);
}

TEST(led_panel, switch_on_all)
{
    led_panel_mode(LED_IMAGE_ALL, LED_MODE_SWITCH_ON);
    led_panel_update(0);
    BYTES_EQUAL(0b1111, led_image);
}

/* switch off one by one */
TEST(led_panel, switch_off_led0)
{
    led_panel_mode(LED_IMAGE_ALL, LED_MODE_SWITCH_ON);
    led_panel_mode(LED_IMAGE_0, LED_MODE_SWITCH_OFF);
    led_panel_update(0);
    BYTES_EQUAL(0b1110, led_image);
}

TEST(led_panel, switch_off_led1)
{
    led_panel_mode(LED_IMAGE_ALL, LED_MODE_SWITCH_ON);
    led_panel_mode(LED_IMAGE_1, LED_MODE_SWITCH_OFF);
    led_panel_update(0);
    BYTES_EQUAL(0b1101, led_image);
}

TEST(led_panel, switch_off_led2)
{
    led_panel_mode(LED_IMAGE_ALL, LED_MODE_SWITCH_ON);
    led_panel_mode(LED_IMAGE_2, LED_MODE_SWITCH_OFF);
    led_panel_update(0);
    BYTES_EQUAL(0b1011, led_image);
}

TEST(led_panel, switch_off_led3)
{
    led_panel_mode(LED_IMAGE_ALL, LED_MODE_SWITCH_ON);
    led_panel_mode(LED_IMAGE_3, LED_MODE_SWITCH_OFF);
    led_panel_update(0);
    BYTES_EQUAL(0b111, led_image);
}

/* switch off combined */
TEST(led_panel, switch_off_led01)
{
    led_panel_mode(LED_IMAGE_ALL, LED_MODE_SWITCH_ON);
    led_panel_mode(LED_IMAGE_0|LED_IMAGE_1, LED_MODE_SWITCH_OFF);
    led_panel_update(0);
    BYTES_EQUAL(0b1100, led_image);
}

TEST(led_panel, switch_off_led02)
{
    led_panel_mode(LED_IMAGE_ALL, LED_MODE_SWITCH_ON);
    led_panel_mode(LED_IMAGE_0|LED_IMAGE_2, LED_MODE_SWITCH_OFF);
    led_panel_update(0);
    BYTES_EQUAL(0b1010, led_image);
}

TEST(led_panel, switch_off_led03)
{
    led_panel_mode(LED_IMAGE_ALL, LED_MODE_SWITCH_ON);
    led_panel_mode(LED_IMAGE_0|LED_IMAGE_3, LED_MODE_SWITCH_OFF);
    led_panel_update(0);
    BYTES_EQUAL(0b110, led_image);
}

TEST(led_panel, switch_off_led012)
{
    led_panel_mode(LED_IMAGE_ALL, LED_MODE_SWITCH_ON);
    led_panel_mode(LED_IMAGE_0|LED_IMAGE_1|LED_IMAGE_2, LED_MODE_SWITCH_OFF);
    led_panel_update(0);
    BYTES_EQUAL(0b1000, led_image);
}

TEST(led_panel, switch_off_led013)
{
    led_panel_mode(LED_IMAGE_ALL, LED_MODE_SWITCH_ON);
    led_panel_mode(LED_IMAGE_0|LED_IMAGE_1|LED_IMAGE_3, LED_MODE_SWITCH_OFF);
    led_panel_update(0);
    BYTES_EQUAL(0b100, led_image);
}

TEST(led_panel, switch_off_led023)
{
    led_panel_mode(LED_IMAGE_ALL, LED_MODE_SWITCH_ON);
    led_panel_mode(LED_IMAGE_0|LED_IMAGE_2|LED_IMAGE_3, LED_MODE_SWITCH_OFF);
    led_panel_update(0);
    BYTES_EQUAL(0b10, led_image);
}

TEST(led_panel, switch_off_led123)
{
    led_panel_mode(LED_IMAGE_ALL, LED_MODE_SWITCH_ON);
    led_panel_mode(LED_IMAGE_1|LED_IMAGE_2|LED_IMAGE_3, LED_MODE_SWITCH_OFF);
    led_panel_update(0);
    BYTES_EQUAL(0b1, led_image);
}

TEST(led_panel, switch_off_led0123)
{
    led_panel_mode(LED_IMAGE_ALL, LED_MODE_SWITCH_ON);
    led_panel_mode(LED_IMAGE_0|LED_IMAGE_1|LED_IMAGE_2|LED_IMAGE_3, LED_MODE_SWITCH_OFF);
    led_panel_update(0);
    BYTES_EQUAL(0, led_image);
}

TEST(led_panel, switch_off_all)
{
    led_panel_mode(LED_IMAGE_ALL, LED_MODE_SWITCH_ON);
    led_panel_mode(LED_IMAGE_ALL, LED_MODE_SWITCH_OFF);
    led_panel_update(0);
    BYTES_EQUAL(0, led_image);
}

/*
 * flip mode
 */
TEST(led_panel, flip_1)
{
    rt_uint32_t i;
    led_panel_mode(LED_IMAGE_0, LED_MODE_FLIP_1);
    for (i = 0; i < 100; ++i)
    {
        led_panel_update(i);
    }
    CHECK_EQUAL(50, cnt[0].on);
    CHECK_EQUAL(50, cnt[0].off);
}

TEST(led_panel, flip_2)
{
    rt_uint32_t i;
    led_panel_mode(LED_IMAGE_0, LED_MODE_FLIP_2);
    for (i = 0; i < 100; ++i)
    {
        led_panel_update(i);
    }
    CHECK_EQUAL(25, cnt[0].on);
    CHECK_EQUAL(25, cnt[0].off);
}

TEST(led_panel, flip_4)
{
    rt_uint32_t i;
    led_panel_mode(LED_IMAGE_0, LED_MODE_FLIP_4);
    for (i = 0; i < 800; ++i)
    {
        led_panel_update(i);
    }
    CHECK_EQUAL(100, cnt[0].on);
    CHECK_EQUAL(100, cnt[0].off);
}

TEST(led_panel, flip_8)
{
    rt_uint32_t i;
    led_panel_mode(LED_IMAGE_0, LED_MODE_FLIP_8);
    for (i = 0; i < 1600; ++i)
    {
        led_panel_update(i);
    }
    CHECK_EQUAL(100, cnt[0].on);
    CHECK_EQUAL(100, cnt[0].off);
}

TEST(led_panel, flip_16)
{
    rt_uint32_t i;
    led_panel_mode(LED_IMAGE_0, LED_MODE_FLIP_16);
    for (i = 0; i < 3200; ++i)
    {
        led_panel_update(i);
    }
    CHECK_EQUAL(100, cnt[0].on);
    CHECK_EQUAL(100, cnt[0].off);
}

TEST(led_panel, flip_32)
{
    rt_uint32_t i;
    led_panel_mode(LED_IMAGE_0, LED_MODE_FLIP_32);
    for (i = 0; i < 6400; ++i)
    {
        led_panel_update(i);
    }
    CHECK_EQUAL(100, cnt[0].on);
    CHECK_EQUAL(100, cnt[0].off);
}

TEST(led_panel, flip_64)
{
    rt_uint32_t i;
    led_panel_mode(LED_IMAGE_0, LED_MODE_FLIP_64);
    for (i = 0; i < 12800; ++i)
    {
        led_panel_update(i);
    }
    CHECK_EQUAL(100, cnt[0].on);
    CHECK_EQUAL(100, cnt[0].off);
}

/*
 * pattern mode
 */


 TEST(led_panel, pattern_1)
 {
     rt_uint32_t i;
     led_panel_mode(LED_IMAGE_0, LED_MODE_PATTERN_1);
     for (i = 0; i < 100; ++i)
     {
         led_panel_update(i);
     }
     CHECK_EQUAL(25, cnt[0].on);
     CHECK_EQUAL(75, cnt[0].off);
 }

 TEST(led_panel, pattern_2)
 {
     rt_uint32_t i;
     led_panel_mode(LED_IMAGE_0, LED_MODE_PATTERN_2);
     for (i = 0; i < 160; ++i)
     {
         led_panel_update(i);
     }
     CHECK_EQUAL(80, cnt[0].on);
     CHECK_EQUAL(80, cnt[0].off);
 }

 IGNORE_TEST(led_panel, fast_switch_on)
 {
 }

 IGNORE_TEST(led_panel, fast_switch_off)
 {
 }



