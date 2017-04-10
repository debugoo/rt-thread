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
}

TEST_GROUP(led_panel)
{
void setup()
{
}
void teardown()
{
}
};

TEST(led_panel, init)
{
    rt_uint8_t bits = 0x0F;
    led_panel_init(&bits);
    BYTES_EQUAL(0x00, bits);
}

TEST(led_panel, allclose)
{
    rt_uint8_t bits = 0x0F;
    led_panel_all_close(&bits);
    BYTES_EQUAL(0x00, bits);
}

TEST(led_panel, allopen)
{
    rt_uint8_t bits = 0x00;
    led_panel_all_open(&bits);
    BYTES_EQUAL(0x0F, bits);
}

TEST(led_panel, open_led0_clean)
{
    rt_uint8_t bits = 0x00;
    led_panel_open(&bits, 0);
    BYTES_EQUAL(0x01, bits);
}

TEST(led_panel, open_led0_with_open_1)
{
    rt_uint8_t bits = 0b10;
    led_panel_open(&bits, 0);
    BYTES_EQUAL(0b10 + 1, bits);
}

TEST(led_panel, open_led0_with_open_12)
{
    rt_uint8_t bits = 0b110;
    led_panel_open(&bits, 0);
    BYTES_EQUAL(0b110 + 1, bits);
}

TEST(led_panel, open_led0_with_open_13)
{
    rt_uint8_t bits = 0b1010;
    led_panel_open(&bits, 0);
    BYTES_EQUAL(0b1010 + 1, bits);
}

TEST(led_panel, open_led0_with_open_23)
{
    rt_uint8_t bits = 0b1100;
    led_panel_open(&bits, 0);
    BYTES_EQUAL(0b1100 + 1, bits);
}

TEST(led_panel, open_led0_with_open_123)
{
    rt_uint8_t bits = 0b1110;
    led_panel_open(&bits, 0);
    BYTES_EQUAL(0b1110 + 1, bits);
}

TEST(led_panel, open_out_of_range_1)
{
    rt_uint8_t bits = 0;
    led_panel_open(&bits, 4);
    BYTES_EQUAL(0, bits);
}

TEST(led_panel, open_out_of_range_2)
{
    rt_uint8_t bits = 0;
    led_panel_open(&bits, 5);
    BYTES_EQUAL(0, bits);
}

TEST(led_panel, open_out_of_range_3)
{
    rt_uint8_t bits = 0;
    led_panel_open(&bits, 6);
    BYTES_EQUAL(0, bits);
}

TEST(led_panel, open_out_of_range_4)
{
    rt_uint8_t bits = 0;
    led_panel_open(&bits, 7);
    BYTES_EQUAL(0, bits);
}

TEST(led_panel, close_led0_with_open_0)
{
    rt_uint8_t bits = 0b01;
    led_panel_close(&bits, 0);
    BYTES_EQUAL(0b00, bits);
}

TEST(led_panel, close_led0_with_open_01)
{
    rt_uint8_t bits = 0b11;
    led_panel_close(&bits, 0);
    BYTES_EQUAL(0b10, bits);
}

TEST(led_panel, close_led0_with_open_012)
{
    rt_uint8_t bits = 0b111;
    led_panel_close(&bits, 0);
    BYTES_EQUAL(0b110, bits);
}

TEST(led_panel, close_led0_with_open_0123)
{
    rt_uint8_t bits = 0b1111;
    led_panel_close(&bits, 0);
    BYTES_EQUAL(0b1110, bits);
}


TEST(led_panel, close_out_of_range_1)
{
    rt_uint8_t bits = 0xFF;
    led_panel_close(&bits, 4);
    BYTES_EQUAL(0xFF, bits);
}

TEST(led_panel, close_out_of_range_2)
{
    rt_uint8_t bits = 0xFF;
    led_panel_close(&bits, 5);
    BYTES_EQUAL(0xFF, bits);
}

TEST(led_panel, close_out_of_range_3)
{
    rt_uint8_t bits = 0xFF;
    led_panel_close(&bits, 6);
    BYTES_EQUAL(0xFF, bits);
}

TEST(led_panel, close_out_of_range_4)
{
    rt_uint8_t bits = 0xFF;
    led_panel_close(&bits, 7);
    BYTES_EQUAL(0xFF, bits);
}

TEST(led_panel, open_and_close_0)
{
    rt_uint8_t bits = 0x0;
    led_panel_open(&bits, 0);
    led_panel_close(&bits, 0);
    BYTES_EQUAL(0x00, bits);
}

