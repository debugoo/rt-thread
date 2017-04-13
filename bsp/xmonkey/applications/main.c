#include <rtthread.h>
#include "stm32f4xx.h"
#include "string.h"
#include "pinx.h"
#include "led_panel.h"

#define LED0_PIN PB4
#define LED1_PIN PB2
#define LED2_PIN PB3
#define LED3_PIN PB11

#define LED_COUNT  4
static const rt_uint16_t pins[LED_COUNT] = {
    //LED0_PIN, LED1_PIN, LED2_PIN, LED3_PIN 
    LED1_PIN, LED3_PIN, LED2_PIN, LED0_PIN 
};

struct rt_timer t;
rt_uint32_t tick = 0;
static void timeout(void* p)
{
    led_panel_update(tick++);
}
#define LED_PIN PC13
static rt_err_t prolog(void)
{
    int i = 0;
    for (i = 0; i < LED_COUNT; ++i)
    {
        rt_pin_mode(pins[i], PIN_MODE_OUT_PP);
    }
    rt_timer_init(&t, "ledt", timeout, RT_NULL, 10, RT_TIMER_FLAG_PERIODIC);
    return RT_EOK;
}
static rt_err_t epilog(void)
{
    return RT_EOK;
}
void turn_on(rt_uint8_t index)
{
    rt_pin_write(pins[index], 0);
}
void turn_off(rt_uint8_t index)
{
    rt_pin_write(pins[index], 1);
}
struct led_hw_ops ops = {turn_on, turn_off};

rt_uint8_t modes[4] = {LED_MODE_SWITCH_ON, LED_MODE_FLIP_8, LED_MODE_PATTERN_1, LED_MODE_PATTERN_2};
rt_uint32_t custpat[4] = {0xFF, 0x3E, 0x1C, 0x08};

static rt_uint8_t custom(rt_uint32_t tick)
{
    rt_uint32_t reminder = tick%16;
    int i;
    rt_uint8_t tmp = 0;
    for (i = 0; i < 4; ++i)
    {
        rt_uint8_t bit = ((1<<reminder)&custpat[i])?1:0;
        tmp |= bit<<i;
    }
    return tmp;
}


static void main_run(void* parameter)
{
    rt_uint8_t mode = LED_MODE_FLIP_1;
    RT_ASSERT(prolog() == RT_EOK);
    led_panel_init(&ops);

    led_panel_mode(LED_IMAGE_0, LED_MODE_SWITCH_ON);
    led_panel_mode(LED_IMAGE_1, LED_MODE_FLIP_8);
    led_panel_mode(LED_IMAGE_2, LED_MODE_PATTERN_1);
    led_panel_mode(LED_IMAGE_3, LED_MODE_PATTERN_2);
    led_panel_mode(LED_IMAGE_ALL, LED_MODE_FLIP_1);
    led_panel_custom(custom);
    rt_timer_start(&t);
    while(1);
    while(1)
    {
        led_panel_mode(LED_IMAGE_ALL, mode);
        if (++mode > LED_MODE_PATTERN_2)
        {
            mode = LED_MODE_FLIP_1;
        }

        rt_thread_delay(1000);
    }
    RT_ASSERT(epilog() == RT_EOK);
}

ALIGN(RT_ALIGN_SIZE)
static rt_uint8_t       thread_stack[ 256 ];
static struct rt_thread thread_handle;

rt_err_t start_thread_led(void)
{
    rt_err_t err;

    err = rt_thread_init(&thread_handle,
                        "LEDTEST",
                        main_run,
                        RT_NULL,
                        (rt_uint8_t*)&thread_stack[0],
                        sizeof(thread_stack),
                        21,
                        5);
    if (err != RT_EOK)
    {
        return err;
    }

    return rt_thread_startup(&thread_handle);
}
