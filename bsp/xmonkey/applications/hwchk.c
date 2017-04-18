/*****************************************************************************
 This confidential and proprietary software may be only used as authorized
 by a licensing agreement from xujun.
 (C) COPYRIGHT 2015 xujun. ALL RIGHTS RESERVED
 
 File           :    hwchk.c
 Author         :    xujun
 Date           :    2017/04/06
 Version        :    1.0
 Description    :    running it to confirm HW work correctly
 History        :    
 
 *****************************************************************************/

#include "hwchk.h"
#include "led_panel.h"
//#include "pinx.h"
#include "i2cx.h"
#include "inv_mpu.h"



static rt_device_t i2c_bus = RT_NULL;

#define I2C_ADDR_MPU9250    (0x68<<1) /* 1101000 */

#define REG_MPU9250_WHOAMI  (0x75)


/*
#define LED0_PIN PB4
#define LED1_PIN PB2
#define LED2_PIN PB3
#define LED3_PIN PB11

#define LED_COUNT  4
static const rt_uint16_t pins[LED_COUNT] = {
    //LED0_PIN, LED1_PIN, LED2_PIN, LED3_PIN 
    LED1_PIN, LED3_PIN, LED2_PIN, LED0_PIN 
};

static struct rt_timer t;
static rt_uint32_t tick = 0;

static void turn_on(rt_uint8_t index)
{
    rt_pin_write(pins[index], 0);
}
static void turn_off(rt_uint8_t index)
{
    rt_pin_write(pins[index], 1);
}
static struct led_hw_ops ops = {turn_on, turn_off};

static void timeout(void* p)
{
    led_panel_update(tick++);
}

static void led_init(void)
{
    int i = 0;
    for (i = 0; i < LED_COUNT; ++i)
    {
        rt_pin_mode(pins[i], PIN_MODE_OUT_PP);
    }
    led_panel_init(&ops);
    rt_timer_init(&t, "ledt", timeout, RT_NULL, 10, RT_TIMER_FLAG_PERIODIC);
    rt_timer_start(&t);
}
*/

/* 初始化失败则所有LED */
rt_err_t bus_init(void)
{
    /* open I2C bus */
    rt_err_t err;
    i2c_bus  = rt_device_find(I2C1_BUS_NAME);
    if (i2c_bus  == RT_NULL)
        return -RT_ERROR;
    err = rt_device_open(i2c_bus , RT_DEVICE_OFLAG_RDWR);
    return err;
}
rt_err_t bus_deinit(void)
{
    return rt_device_close(i2c_bus);
}


rt_err_t hw_check_mpu9250(void)
{
    rt_uint8_t tmp;
    rt_uint8_t addr;
    rt_size_t size;
    struct int_param_s int_param;
    int result;

    result = mpu_init(&int_param);
    if (result)
    {
        return -RT_ERROR;
    }

    addr = I2C_ADDR_MPU9250;
    tmp = REG_MPU9250_WHOAMI;
    size = rt_device_write(i2c_bus, addr, &tmp, 1);
    if (size != 1)
        return -RT_ERROR;

    size = rt_device_read(i2c_bus, addr, &tmp, 1);
    if (size != 1)
        return -RT_ERROR;

    if (tmp != 0x71)
        return -RT_ERROR;
    
    return RT_EOK;
}

rt_err_t hw_check_bmp180(void)
{
    return -RT_ERROR;
}

rt_err_t hw_check_sht20(void)
{
    return -RT_ERROR;
}

rt_err_t hw_check_sim868(void)
{
    return -RT_ERROR;
}

void check_phase_led(void)
{
    led_panel_mode(LED_IMAGE_ALL, LED_MODE_FLIP_1);
    rt_thread_delay(500);
}
void check_phase_gap(void)
{
    led_panel_group(LED_GROUP_1);
    rt_thread_delay(300);
}
rt_err_t (*chk_func[4])(void) =
{
    hw_check_mpu9250,
    hw_check_bmp180,
    hw_check_sht20,
    hw_check_sim868,
};

void check_phase_module(void)
{
    int i;
    if (RT_EOK != bus_init())
    {
        /* hw check initialize failed */
        led_panel_group(LED_GROUP_2);
        while(1);
    }
    led_panel_group(LED_GROUP_1);
    for (i = 0; i < 4; ++i)
    {
        if( RT_EOK != chk_func[i]())
        {
            /* hw check failed */
            led_panel_mode(LED_IMAGE_ALL, LED_MODE_SWITCH_OFF);        
            led_panel_mode(i+1, LED_MODE_FLIP_1);
            while(1) {
                rt_thread_delay(1000);
            }
        }
    }
    /* hw check passed */
    bus_deinit();
    led_panel_mode(LED_IMAGE_ALL, LED_MODE_SWITCH_ON);
    while(1);
}

void hw_check(void)
{
    //led_init();
    check_phase_led();
    check_phase_gap();
    check_phase_module();
}

