/*
 * File      : pin.h
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2015, RT-Thread Development Team
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * Change Logs:
 * Date           Author       Notes
 * 2015-01-20     Bernard      the first version
 * 2015-05-19     XuJun        bind to stm32
 */

#ifndef __PINX_H__
#define __PINX_H__

#include <rtconfig.h>
#ifdef RT_USING_PINX

#include <rtthread.h>
#include <rtdevice.h>
#include <stm32f4xx.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PIN_DEVICE_NAME     "pin"

enum STM32_PINS
{
    /* GPIOA */
    PA0 = 0, PA1, PA2, PA3, PA4, PA5, PA6, PA7, PA8, PA9, PA10, PA11, PA12, PA13, PA14, PA15,
    /* GPIOB */
    PB0, PB1, PB2, PB3, PB4, PB5, PB6, PB7, PB8, PB9, PB10, PB11, PB12, PB13, PB14, PB15,
    /* GPIOC */
    PC0, PC1, PC2, PC3, PC4, PC5, PC6, PC7, PC8, PC9, PC10, PC11, PC12, PC13, PC14, PC15,
    /* GPIOD */
    PD0, PD1, PD2, PD3, PD4, PD5, PD6, PD7, PD8, PD9, PD10, PD11, PD12, PD13, PD14, PD15,
    /* GPIOE */
    PE0, PE1, PE2, PE3, PE4, PE5, PE6, PE7, PE8, PE9, PE10, PE11, PE12, PE13, PE14, PE15,
    /* GPIOF */
    PF0, PF1, PF2, PF3, PF4, PF5, PF6, PF7, PF8, PF9, PF10, PF11, PF12, PF13, PF14, PF15,
    /* GPIOG */
    PG0, PG1, PG2, PG3, PG4, PG5, PG6, PG7, PG8, PG9, PG10, PG11, PG12, PG13, PG14, PG15,
};
#define IS_VALID_PIN(pin) (pin>=PA0 && pin<=PG15)
void rt_hw_pin_init(void);

/* pin device and operations for RT-Thread */
struct rt_device_pin
{
    struct rt_device parent;
    const struct rt_pin_ops *ops;
};

#define PIN_LOW                 0x00
#define PIN_HIGH                0x01

#define PIN_MODE_OUTPUT         0x00
#define PIN_MODE_INPUT          0x01
#define PIN_MODE_INPUT_PULLUP   0x02

#define PIN_MODE_OUT_PP      0x03
#define PIN_MODE_OUT_OD      0x04
#define PIN_MODE_AF_PP       0x05
#define PIN_MODE_AF_OD       0x06

struct rt_device_pin_mode
{
    rt_uint16_t pin;
    rt_uint16_t mode;
};
struct rt_device_pin_status
{
    rt_uint16_t pin;
    rt_uint16_t status;
};

struct rt_pin_ops
{
    void (*pin_mode)(struct rt_device *device, rt_base_t pin, rt_base_t mode);
    void (*pin_write)(struct rt_device *device, rt_base_t pin, rt_base_t value);
    int (*pin_read)(struct rt_device *device, rt_base_t pin);

    /* TODO: add GPIO interrupt */
};

int rt_device_pin_register(const char *name, const struct rt_pin_ops *ops, void *user_data);

void rt_pin_mode(rt_base_t pin, rt_base_t mode);
void rt_pin_write(rt_base_t pin, rt_base_t value);
int  rt_pin_read(rt_base_t pin);

#ifdef __cplusplus
}
#endif

#endif /*RT_USING_PINX*/
#endif
