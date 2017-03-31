/*
 * File      : pin.c
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
#include <rtconfig.h>
#ifdef RT_USING_UARTX

/*
#include <sys/types.h>
#if __MISC_VISIBLE
typedef u_char my_u_char;
#endif
*/

//#ifdef ENABLE_PINX
#include <pinx.h>
#include <rtthread.h>
#include <stm32f4xx.h>
#ifdef RT_USING_FINSH
#include <finsh.h>
#endif

/******************************************************************************
 * ops
 *****************************************************************************/
struct stm32_gpio
{
    rt_uint32_t     rcc;
    GPIO_TypeDef    *port;
};
static struct stm32_gpio _tbl_gpio[] = 
{
    {RCC_AHB1Periph_GPIOA, GPIOA},
    {RCC_AHB1Periph_GPIOB, GPIOB},
    {RCC_AHB1Periph_GPIOC, GPIOC},
    {RCC_AHB1Periph_GPIOD, GPIOD},
    {RCC_AHB1Periph_GPIOE, GPIOE},
    {RCC_AHB1Periph_GPIOF, GPIOF},
    {RCC_AHB1Periph_GPIOG, GPIOG},
    {RCC_AHB1Periph_GPIOH, GPIOH},
    {RCC_AHB1Periph_GPIOI, GPIOI},
};
#define PORT_PIN_COUNT  (16)
static rt_uint16_t _tbl_pins[] = 
{
    GPIO_Pin_0,
    GPIO_Pin_1,
    GPIO_Pin_2,
    GPIO_Pin_3,
    GPIO_Pin_4,
    GPIO_Pin_5,
    GPIO_Pin_6,
    GPIO_Pin_7,
    GPIO_Pin_8,
    GPIO_Pin_9,
    GPIO_Pin_10,
    GPIO_Pin_11,
    GPIO_Pin_12,
    GPIO_Pin_13,
    GPIO_Pin_14,
    GPIO_Pin_15,
};
struct stm32_gpio* get_gpio(rt_uint16_t pin)
{
    if (!IS_VALID_PIN((int)pin))
    {
        return RT_NULL;
    }
    return &_tbl_gpio[pin/PORT_PIN_COUNT];
}
rt_uint16_t get_pin(rt_uint16_t pin)
{
    if (!IS_VALID_PIN((int)pin))
    {
        return 0;
    }
    return _tbl_pins[pin%PORT_PIN_COUNT];
}

void stm32_pin_mode(struct rt_device *device, rt_base_t pin, rt_base_t mode)
{
    switch(mode)
    {
        case PIN_MODE_OUT_PP:
            {
                GPIO_InitTypeDef GPIO_InitStructure;
                struct stm32_gpio* gpio;
                gpio = get_gpio(pin);
                RT_ASSERT(gpio);

                RCC_AHB1PeriphClockCmd(gpio->rcc, ENABLE);
                GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
                GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
                GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
                GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
                GPIO_InitStructure.GPIO_Pin  = get_pin(pin);
                GPIO_Init(gpio->port, &GPIO_InitStructure);
            }
            break;
        case PIN_MODE_OUT_OD:
            {
                GPIO_InitTypeDef GPIO_InitStructure;
                struct stm32_gpio* gpio;
                gpio = get_gpio(pin);
                RT_ASSERT(gpio);

                RCC_AHB1PeriphClockCmd(gpio->rcc, ENABLE);
                GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
                GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
                GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
                GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
                GPIO_InitStructure.GPIO_Pin  = get_pin(pin);
                GPIO_Init(gpio->port, &GPIO_InitStructure);
            }
            break;
        case PIN_MODE_AF_PP:
            {
                GPIO_InitTypeDef GPIO_InitStructure;
                struct stm32_gpio* gpio;
                gpio = get_gpio(pin);
                RT_ASSERT(gpio);
        
                RCC_AHB1PeriphClockCmd(gpio->rcc, ENABLE);
                GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
                GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
                GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
                GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
                GPIO_InitStructure.GPIO_Pin  = get_pin(pin);
                GPIO_Init(gpio->port, &GPIO_InitStructure);
            }
            break;
        case PIN_MODE_AF_OD:
            {
                GPIO_InitTypeDef GPIO_InitStructure;
                struct stm32_gpio* gpio;
                gpio = get_gpio(pin);
                RT_ASSERT(gpio);
        
                RCC_AHB1PeriphClockCmd(gpio->rcc, ENABLE);
                GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
                GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
                GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
                GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
                GPIO_InitStructure.GPIO_Pin  = get_pin(pin);
                GPIO_Init(gpio->port, &GPIO_InitStructure);
            }
            break;
        case PIN_MODE_INPUT:
            {
                GPIO_InitTypeDef GPIO_InitStructure;
                struct stm32_gpio* gpio;
                gpio = get_gpio(pin);
                RT_ASSERT(gpio);

                RCC_AHB1PeriphClockCmd(gpio->rcc, ENABLE);
                GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
                GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
                GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
                GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
                GPIO_InitStructure.GPIO_Pin  = get_pin(pin);
                GPIO_Init(gpio->port, &GPIO_InitStructure);
            }
            break;
        default:
            RT_ASSERT(RT_FALSE);
            break;
    }
}
void stm32_pin_write(struct rt_device *device, rt_base_t pin, rt_base_t value)
{
    struct stm32_gpio* gpio = get_gpio(pin);
    RT_ASSERT(gpio);
    if(value)
    {
        GPIO_SetBits(gpio->port, get_pin(pin));
    }
    else
    {
        GPIO_ResetBits(gpio->port, get_pin(pin));
    }
}
int stm32_pin_read(struct rt_device *device, rt_base_t pin)
{
    struct stm32_gpio* gpio = get_gpio(pin);
    RT_ASSERT(gpio);
    return GPIO_ReadInputDataBit(gpio->port, get_pin(pin));
}

static struct rt_pin_ops stm32_pin_ops = 
{
    stm32_pin_mode,
    stm32_pin_write,
    stm32_pin_read
};
void rt_hw_pin_init(void)
{
    rt_device_pin_register(PIN_DEVICE_NAME, &stm32_pin_ops, RT_NULL);
}
/******************************************************************************
 * driver
 *****************************************************************************/
static struct rt_device_pin _hw_pin;
static rt_size_t _pin_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size)
{
    struct rt_device_pin_status *status;
    struct rt_device_pin *pin = (struct rt_device_pin *)dev;

    /* check parameters */
    RT_ASSERT(pin != RT_NULL);

    status = (struct rt_device_pin_status *) buffer;
    if (status == RT_NULL || size != sizeof(*status)) return 0;

    status->status = pin->ops->pin_read(dev, status->pin);
    return size;
}

static rt_size_t _pin_write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size)
{
    struct rt_device_pin_status *status;
    struct rt_device_pin *pin = (struct rt_device_pin *)dev;

    /* check parameters */
    RT_ASSERT(pin != RT_NULL);

    status = (struct rt_device_pin_status *) buffer;
    if (status == RT_NULL || size != sizeof(*status)) return 0;

    pin->ops->pin_write(dev, (rt_base_t)status->pin, (rt_base_t)status->status);

    return size;
}

static rt_err_t  _pin_control(rt_device_t dev, rt_uint8_t cmd, void *args)
{
    struct rt_device_pin_mode *mode;
    struct rt_device_pin *pin = (struct rt_device_pin *)dev;

    /* check parameters */
    RT_ASSERT(pin != RT_NULL);

    mode = (struct rt_device_pin_mode *) args;
    if (mode == RT_NULL) return -RT_ERROR;

    pin->ops->pin_mode(dev, (rt_base_t)mode->pin, (rt_base_t)mode->mode);

    return 0;
}

int rt_device_pin_register(const char *name, const struct rt_pin_ops *ops, void *user_data)
{
    _hw_pin.parent.type         = RT_Device_Class_Miscellaneous;
    _hw_pin.parent.rx_indicate  = RT_NULL;
    _hw_pin.parent.tx_complete  = RT_NULL;

    _hw_pin.parent.init         = RT_NULL;
    _hw_pin.parent.open         = RT_NULL;
    _hw_pin.parent.close        = RT_NULL;
    _hw_pin.parent.read         = _pin_read;
    _hw_pin.parent.write        = _pin_write;
    _hw_pin.parent.control      = _pin_control;

    _hw_pin.ops                 = ops;
    _hw_pin.parent.user_data    = user_data;

    /* register a character device */
    rt_device_register(&_hw_pin.parent, "pin", RT_DEVICE_FLAG_RDWR);

    return 0;
}

/* RT-Thread Hardware PIN APIs */
void rt_pin_mode(rt_base_t pin, rt_base_t mode)
{
    _hw_pin.ops->pin_mode(&_hw_pin.parent, pin, mode);
}
FINSH_FUNCTION_EXPORT_ALIAS(rt_pin_mode, pinMode, set hardware pin mode);

void rt_pin_write(rt_base_t pin, rt_base_t value)
{
    _hw_pin.ops->pin_write(&_hw_pin.parent, pin, value);
}
FINSH_FUNCTION_EXPORT_ALIAS(rt_pin_write, pinWrite, write value to hardware pin);

int  rt_pin_read(rt_base_t pin)
{
    return _hw_pin.ops->pin_read(&_hw_pin.parent, pin);
}
FINSH_FUNCTION_EXPORT_ALIAS(rt_pin_read, pinRead, read status from hardware pin);

#endif /*RT_USING_PINX*/

//#endif /*ENABLE_PINX*/
