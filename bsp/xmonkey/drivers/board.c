/*
 * File      : board.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2009 RT-Thread Develop Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      first implementation
 */

#include <rthw.h>
#include <rtthread.h>


//#ifdef ENABLE_BOARD

#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "board.h"
#include "uartx.h"
#include "pinx.h"
#include "i2cx.h"
#include "rtcx.h"
#include "gpio.h"

#ifdef RT_USING_FINSH
#include <finsh.h>
#else
#define FINSH_FUNCTION_EXPORT(...)
#define FINSH_FUNCTION_EXPORT_ALIAS(...)
#endif

/**
 * @addtogroup STM32
 */

/*@{*/

/*******************************************************************************
* Function Name  : NVIC_Configuration
* Description    : Configures Vector Table base location.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void NVIC_Configuration(void)
{
#ifdef  VECT_TAB_RAM
    /* Set the Vector Table base location at 0x20000000 */
    NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);
#else  /* VECT_TAB_FLASH  */
    /* Set the Vector Table base location at 0x08000000 */
    NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
#endif

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
}

/*******************************************************************************
 * Function Name  : SysTick_Configuration
 * Description    : Configures the SysTick for OS tick.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void  SysTick_Configuration(void)
{
    RCC_ClocksTypeDef  rcc_clocks;
    rt_uint32_t         cnts;

    RCC_GetClocksFreq(&rcc_clocks);

    cnts = (rt_uint32_t)rcc_clocks.HCLK_Frequency / RT_TICK_PER_SECOND;
    cnts = cnts / 8;

    SysTick_Config(cnts);
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
}

/**
 * This is the timer interrupt service routine.
 *
 */
void SysTick_Handler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    rt_tick_increase();

    /* leave interrupt */
    rt_interrupt_leave();
}

/*
#define PIN_PWR_EN_LCD    PC4
#define PIN_PWR_EN_VBUS   PC5
#define PIN_PWR_EN_GPS    PC6
#define PIN_PWR_EN_SENS   PC7
static void power_init(void)
{
    rt_pin_mode(PIN_PWR_EN_LCD,  PIN_MODE_OUT_PP);
    rt_pin_mode(PIN_PWR_EN_VBUS, PIN_MODE_OUT_PP);
    rt_pin_mode(PIN_PWR_EN_GPS,  PIN_MODE_OUT_PP);
    rt_pin_mode(PIN_PWR_EN_SENS, PIN_MODE_OUT_PP);
}
static void power_on_sensor(void)
{
    rt_pin_write(PIN_PWR_EN_SENS, PIN_HIGH);
}
static void power_off_sensor(void)
{
    rt_pin_write(PIN_PWR_EN_SENS, PIN_LOW);
}
*/
/**
 * This function will initial STM32 board.
 */
void rt_hw_board_init()
{
    /* NVIC Configuration */
    NVIC_Configuration();

    /* Configure the SysTick */
    SysTick_Configuration();

#ifdef RT_USING_PINX
    rt_hw_pin_init();
    //power_init();
    //power_on_sensor();
#endif

#ifdef RT_USING_UARTX
    rt_hw_usart_init();
#endif


#ifdef RT_USING_RTCX
    rt_hw_rtc_init();
#endif

#ifdef RT_USING_I2CX
    if(1)
    {
        //power_on_sensor();
        struct i2c_configuration conf = {0xC1, RT_I2C_SPEED_FAST, 50};
        RT_ASSERT(rt_hw_i2c_init(0, &conf) == RT_EOK);
    }
#endif

#ifdef RT_USING_CONSOLE
    rt_console_set_device(CONSOLE_DEVICE);
#endif

#ifdef RT_USING_COMPONENTS_INIT
    rt_components_board_init();
#endif

}

void rt_system_reset(void)
{
    /* if timeout, need to do system reset */
    __set_FAULTMASK(1);
    NVIC_SystemReset();
}
FINSH_FUNCTION_EXPORT_ALIAS(rt_system_reset, reset, reset the whole system);


/*@}*/

//#endif /*ENABLE_BOARD*/
