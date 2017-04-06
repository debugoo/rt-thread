#include <rtthread.h>
#include "stm32f4xx.h"
#include "string.h"
#include "pinx.h"

#define LED0_PIN PB4
#define LED1_PIN PB2
#define LED2_PIN PB3
#define LED3_PIN PB11

#define LED_COUNT  4
static const rt_uint16_t pins[LED_COUNT] = {
    LED0_PIN, LED1_PIN, LED2_PIN, LED3_PIN 
};

#define LED_PIN PC13
static rt_err_t prolog(void)
{
    int i = 0;
    for (i = 0; i < LED_COUNT; ++i)
    {
        rt_pin_mode(pins[i], PIN_MODE_OUT_PP);
    }
    return RT_EOK;
}
static rt_err_t epilog(void)
{
    return RT_EOK;
}
static void main_run(void* parameter)
{
    rt_uint8_t onoff = 0;
    int i;
    RT_ASSERT(prolog() == RT_EOK);
    while(1)
    {
        onoff = onoff?0:1;
        for (i = 0; i < LED_COUNT; ++i)
        {
            rt_pin_write(pins[i], onoff);
        }
        rt_thread_delay(50);
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
