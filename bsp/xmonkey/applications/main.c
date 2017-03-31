#include <rtthread.h>
#include "stm32f4xx.h"
#include "string.h"
#include "pinx.h"


#define LED_PIN PC13
static rt_err_t prolog(void)
{
    rt_pin_mode(LED_PIN, PIN_MODE_OUT_PP);
    return RT_EOK;
}
static rt_err_t epilog(void)
{
    return RT_EOK;
}
static void main_run(void* parameter)
{
    rt_uint8_t onoff = 0;
    RT_ASSERT(prolog() == RT_EOK);
    while(1)
    {
        onoff = onoff?0:1;
        rt_pin_write(LED_PIN, onoff);
        rt_thread_delay(50);
    }
    RT_ASSERT(epilog() == RT_EOK);
}

ALIGN(RT_ALIGN_SIZE)
static rt_uint8_t       thread_stack[ 128 ];
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
