/**
* Copyright (C), 2015-2015, Chengdu QiPeng technology. Co., Ltd.
*   File name: uartx.c
*      Author: XuJun
*     Version: 1.0.0
*        Date: 18-05-2016
* Description: 
*              re-implements uart device driver
*              using hardware feature of UART as much more
*            
* Change Logs:
* Date            Author           Notes
* 10-05-2015      XuJun            the first version
*/
#include <rtconfig.h>
#ifdef RT_USING_UARTX

#include "uartx.h"
#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>
#include "stm32f4xx.h"
/****************************************************************************
 * hardware
 ***************************************************************************/


/* USART1 */
/*
#define UART1_GPIO_TX        GPIO_Pin_9
#define UART1_GPIO_RX        GPIO_Pin_10
#define UART1_GPIO           GPIOA
*/
#define UART1_GPIO_TX        GPIO_Pin_6
#define UART1_GPIO_RX        GPIO_Pin_7
#define UART1_GPIO           GPIOB

/* USART2 */
#define UART2_GPIO_TX        GPIO_Pin_2
#define UART2_GPIO_RX        GPIO_Pin_3
#define UART2_GPIO           GPIOA

/* USART3 */
#define UART3_GPIO_TX        GPIO_Pin_10
#define UART3_GPIO_RX        GPIO_Pin_11
#define UART3_GPIO           GPIOB

/* UART4 */
#define UART4_GPIO_TX        GPIO_Pin_0
#define UART4_GPIO_RX        GPIO_Pin_1
#define UART4_GPIO           GPIOA

/* UART5 */
#define UART5_GPIO_TX        GPIO_Pin_10
#define UART5_GPIO_RX        GPIO_Pin_11
#define UART5_GPIO           GPIOC

#define STM32_UART_RESPECTIVE
/* STM32 uart driver */
struct stm32_uart
{
    USART_TypeDef* uart_device;
    IRQn_Type irq;
#ifdef STM32_UART_RESPECTIVE
    uint8_t pre;
    uint8_t sub;
#endif
};

static rt_err_t stm32_configure(struct rt_serial_device *serial, struct serial_configure *cfg)
{
    struct stm32_uart* uart;
    USART_InitTypeDef USART_InitStructure;

    RT_ASSERT(serial != RT_NULL);
    RT_ASSERT(cfg != RT_NULL);

    uart = (struct stm32_uart *)serial->parent.user_data;

    USART_InitStructure.USART_BaudRate = cfg->baud_rate;

    if (cfg->data_bits == DATA_BITS_8){
        USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    } else if (cfg->data_bits == DATA_BITS_9) {
        USART_InitStructure.USART_WordLength = USART_WordLength_9b;
    }

    if (cfg->stop_bits == STOP_BITS_1){
        USART_InitStructure.USART_StopBits = USART_StopBits_1;
    } else if (cfg->stop_bits == STOP_BITS_2){
        USART_InitStructure.USART_StopBits = USART_StopBits_2;
    }

    if (cfg->parity == PARITY_NONE){
        USART_InitStructure.USART_Parity = USART_Parity_No;
    } else if (cfg->parity == PARITY_ODD) {
        USART_InitStructure.USART_Parity = USART_Parity_Odd;
    } else if (cfg->parity == PARITY_EVEN) {
        USART_InitStructure.USART_Parity = USART_Parity_Even;
    }

    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(uart->uart_device, &USART_InitStructure);

    /* Enable USART */
    USART_Cmd(uart->uart_device, ENABLE);

    /* by xj: avoid the loss of first sended byte */
    USART_ClearFlag(uart->uart_device, USART_FLAG_TC);

    return RT_EOK;
}

static rt_err_t stm32_control(struct rt_serial_device *serial, int cmd, void *arg)
{
    struct stm32_uart* uart;

    RT_ASSERT(serial != RT_NULL);
    uart = (struct stm32_uart *)serial->parent.user_data;

    switch (cmd)
    {
    case RT_DEVICE_CTRL_IE_PE:
        /* ignore this cmd */
        break;
    case RT_DEVICE_CTRL_IE_TXE:
        USART_ITConfig(uart->uart_device, USART_IT_TXE,  (arg)?ENABLE:DISABLE);
        break;
    case RT_DEVICE_CTRL_IE_TC:
        USART_ITConfig(uart->uart_device, USART_IT_TC,   (arg)?ENABLE:DISABLE);
        break;
    case RT_DEVICE_CTRL_IE_RXNE:
        USART_ITConfig(uart->uart_device, USART_IT_RXNE, (arg)?ENABLE:DISABLE);
        break;
    case RT_DEVICE_CTRL_IE_IDLE:
        USART_ITConfig(uart->uart_device, USART_IT_IDLE, (arg)?ENABLE:DISABLE);
        break;
    case RT_DEVICE_CTRL_IRQ:
        if (arg)
        {
            NVIC_EnableIRQ(uart->irq);
        }
        else
        {
            NVIC_DisableIRQ(uart->irq);
        }
        break;
    }

    return RT_EOK;
}

static const struct rt_uart_ops stm32_uart_ops =
{
    stm32_configure,
    stm32_control,
    RT_NULL,
};
#if defined(RT_USING_UART1)
/* UART1 device driver structure */
struct stm32_uart uart1 =
{
    USART1,
    USART1_IRQn,
#ifdef STM32_UART_RESPECTIVE
    0,
    0,
#endif
};
struct rt_serial_device serial1;

void USART1_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    rt_hw_serial_isr(&serial1);

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* RT_USING_UART1 */

#if defined(RT_USING_UART2)
/* UART1 device driver structure */
struct stm32_uart uart2 =
{
    USART2,
    USART2_IRQn,
#ifdef STM32_UART_RESPECTIVE
    3,
    3,
#endif
};
struct rt_serial_device serial2;

void USART2_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    rt_hw_serial_isr(&serial2);

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* RT_USING_UART2 */

#if defined(RT_USING_UART3)
/* UART3 device driver structure */
struct stm32_uart uart3 =
{
    USART3,
    USART3_IRQn,
#ifdef STM32_UART_RESPECTIVE
    3,
    3,
#endif
};
struct rt_serial_device serial3;

void USART3_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    rt_hw_serial_isr(&serial3);

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* RT_USING_UART3 */

#if defined(RT_USING_UART4)
/* UART4 device driver structure */
struct stm32_uart uart4 =
{
    UART4,
    UART4_IRQn,
#ifdef STM32_UART_RESPECTIVE
    3,
    3,
#endif
};
struct rt_serial_device serial4;

void UART4_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    rt_hw_serial_isr(&serial4);

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* RT_USING_UART4 */

#if defined(RT_USING_UART5)
/* UART5 device driver structure */
struct stm32_uart uart5 =
{
    UART5,
    UART5_IRQn,
#ifdef STM32_UART_RESPECTIVE
    3,
    3,
#endif
};
struct rt_serial_device serial5;

void UART5_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    rt_hw_serial_isr(&serial5);

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* RT_USING_UART5 */

static void RCC_Configuration(void)
{
#if defined(RT_USING_UART1)
    /* Enable UART GPIO clocks */
    //RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    /* Enable UART clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
#endif /* RT_USING_UART1 */

#if defined(RT_USING_UART2)
    /* Enable UART GPIO clocks */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    /* Enable UART clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
#endif /* RT_USING_UART2 */

#if defined(RT_USING_UART3)
    /* Enable UART GPIO clocks */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    /* Enable UART clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
#endif /* RT_USING_UART3 */

#if defined(RT_USING_UART4)
    /* Enable UART GPIO clocks */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    /* Enable UART clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
#endif /* RT_USING_UART4 */

#if defined(RT_USING_UART5)
    /* Enable UART GPIO clocks */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOD, ENABLE);
    /* Enable UART clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
#endif /* RT_USING_UART5 */
}

static void GPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;

#if defined(RT_USING_UART1)
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_6|GPIO_Pin_7;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_USART1);
#endif /* RT_USING_UART1 */

#if defined(RT_USING_UART2)
    /* Configure USART Rx/tx PIN */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_2|GPIO_Pin_3;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2);
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2);
#endif /* RT_USING_UART2 */

#if defined(RT_USING_UART3)
    /* Configure USART Rx/tx PIN */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_10|GPIO_Pin_11;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3);
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3);
#endif /* RT_USING_UART3 */

#if defined(RT_USING_UART4)
    /* Configure UART Rx/tx PIN */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0|GPIO_Pin_1;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_UART4);
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_UART4);
#endif /* RT_USING_UART4 */

#if defined(RT_USING_UART5)
    /* Configure UART Rx/tx PIN */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_12;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_2;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOC,GPIO_PinSource12,GPIO_AF_UART5);
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource2,GPIO_AF_UART5);
#endif /* RT_USING_UART5 */
}

static void NVIC_Configuration(struct stm32_uart* uart)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    //NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    /* Enable the USART1 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = uart->irq;
    //NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    //NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
#ifdef STM32_UART_RESPECTIVE
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = uart->pre;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = uart->sub;
#else
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
#endif
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void rt_hw_usart_init(void)
{
    struct stm32_uart* uart;
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    struct serial_configure config_sim = RT_SERIAL_CONFIG_DEFAULT;
    config_sim.bufsz = 512;

    RCC_Configuration();
    GPIO_Configuration();
#if defined(RT_USING_UART1)
    uart = &uart1;
    config.baud_rate = BAUD_RATE_115200;

    serial1.ops    = &stm32_uart_ops;
    //serial1.config = config;
    serial1.config = config_sim;

    NVIC_Configuration(&uart1);

    /* register UART1 device */
    rt_hw_serial_register(&serial1, UART1_NAME,
                          RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_INT_TX,
                          uart);
#endif /* RT_USING_UART1 */

#if defined(RT_USING_UART2)
    uart = &uart2;

    config.baud_rate = BAUD_RATE_115200;
    serial2.ops    = &stm32_uart_ops;
    serial2.config = config;

    NVIC_Configuration(&uart2);

    /* register UART2 device */
    rt_hw_serial_register(&serial2, UART2_NAME,
                          RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_INT_TX,
                          uart);
#endif /* RT_USING_UART2 */

#if defined(RT_USING_UART3)
    uart = &uart3;

    config.baud_rate = BAUD_RATE_115200;

    serial3.ops    = &stm32_uart_ops;
    serial3.config = config;

    NVIC_Configuration(&uart3);

    /* register UART3 device */
    rt_hw_serial_register(&serial3, UART3_NAME,
                          RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_INT_TX,
                          uart);
#endif /* RT_USING_UART3 */

#if defined(RT_USING_UART4)
    uart = &uart4;

    config.baud_rate = BAUD_RATE_115200;

    serial4.ops    = &stm32_uart_ops;
    serial4.config = config;

    NVIC_Configuration(&uart4);

    /* register UART4 device */
    rt_hw_serial_register(&serial4, UART4_NAME,
                          RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_INT_TX,
                          uart);
#endif /* RT_USING_UART4 */

#if defined(RT_USING_UART5)
    uart = &uart5;

    config.baud_rate = BAUD_RATE_115200;

    serial5.ops    = &stm32_uart_ops;
    serial5.config = config;

    NVIC_Configuration(&uart5);

    /* register UART5 device */
    rt_hw_serial_register(&serial5, UART5_NAME,
                          RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_INT_TX,
                          uart);
#endif /* RT_USING_UART5 */
}

/****************************************************************************
 * driver
 ***************************************************************************/
/*
 * Serial poll routines 
 */
rt_inline int _serial_poll_rx(struct rt_serial_device *serial, rt_uint8_t *data, int length)
{
    int size;
    struct stm32_uart* uart;

    RT_ASSERT(serial != RT_NULL);
    uart = (struct stm32_uart *)serial->parent.user_data;
    size = length;

    while (length)
    {
        if (uart->uart_device->SR & USART_FLAG_RXNE)
        {
            *data = (uart->uart_device->DR & 0xff); 
            data ++; length --;
        }
        if (uart->uart_device->SR & USART_FLAG_IDLE)
        {
            uart->uart_device->SR;
            uart->uart_device->DR;
            break;
        }
    }

    return size - length;
}

rt_inline int _serial_poll_tx(struct rt_serial_device *serial, const rt_uint8_t *data, int length)
{
    int size;
    struct stm32_uart* uart;
    RT_ASSERT(serial != RT_NULL);
    uart = (struct stm32_uart *)serial->parent.user_data;

    size = length;
    while (length)
    {
        /*
         * to be polite with serial console add a line feed
         * to the carriage return character
         */
        if (*data == CHAR_NEWLINE && (serial->parent.open_flag & RT_DEVICE_FLAG_STREAM))
        {
            while(!(uart->uart_device->SR & USART_FLAG_TXE));
            uart->uart_device->DR = CHAR_CARRIAGE_RETURN;
        }
    
        while(!(uart->uart_device->SR & USART_FLAG_TXE));
        uart->uart_device->DR = *data;
    
        ++ data;
        -- length;
    }
    
    while(!(uart->uart_device->SR & USART_FLAG_TC));
    return size - length;
}

/*
 * Serial interrupt routines
 */
rt_inline int _serial_int_rx(struct rt_serial_device *serial, rt_uint8_t *data, int length)
{
    int size;
    struct rt_serial_rx_fifo* rx_fifo;

    RT_ASSERT(serial != RT_NULL);
    size = length; 
    
    rx_fifo = (struct rt_serial_rx_fifo*) serial->serial_rx;
    RT_ASSERT(rx_fifo != RT_NULL);

    /* read from software FIFO */
    while (length)
    {
        int ch;
        rt_base_t level;

        /* disable interrupt */
        level = rt_hw_interrupt_disable();
        if (rx_fifo->get_index != rx_fifo->put_index)
        {
            ch = rx_fifo->buffer[rx_fifo->get_index];
            rx_fifo->get_index += 1;
            if (rx_fifo->get_index >= serial->config.bufsz) rx_fifo->get_index = 0;
        }
        else
        {
            /* no data, enable interrupt and break out */
            rt_hw_interrupt_enable(level);
            break;
        }

        /* enable interrupt */
        rt_hw_interrupt_enable(level);

        *data = ch & 0xff;
        data ++; length --;
    }

    return size - length;
}

rt_inline int _serial_int_tx(struct rt_serial_device *serial, const rt_uint8_t *data, int length)
{
    struct rt_serial_tx_fifo *tx_fifo;
    
    RT_ASSERT(serial != RT_NULL);
    
    tx_fifo = (struct rt_serial_tx_fifo*)serial->serial_tx;
    RT_ASSERT(tx_fifo != RT_NULL);
    
    tx_fifo->ptr = (char*)data;
    tx_fifo->len = length;
    tx_fifo->bytes = 0;

    rt_completion_reset(&(tx_fifo->completion));
    if (tx_fifo->completion.flag == 1)
    {
        RT_ASSERT(RT_FALSE);
    }
    serial->ops->control(serial, RT_DEVICE_CTRL_IE_TXE, (void *)1);
    rt_completion_wait(&(tx_fifo->completion), RT_WAITING_FOREVER);

    if (tx_fifo->bytes != tx_fifo->len)
    {
        RT_ASSERT(RT_FALSE);
    }
    return length;
}

/*
 * Serial DMA routines
 */
rt_inline int _serial_dma_rx(struct rt_serial_device *serial, rt_uint8_t *data, int length)
{
    rt_base_t level;
    int result = RT_EOK;
    struct rt_serial_rx_dma *rx_dma;

    RT_ASSERT((serial != RT_NULL) && (data != RT_NULL));
    rx_dma = (struct rt_serial_rx_dma*)serial->serial_rx;
    RT_ASSERT(rx_dma != RT_NULL);

    level = rt_hw_interrupt_disable();
    if (rx_dma->activated != RT_TRUE)
    {
        rx_dma->activated = RT_TRUE;
        serial->ops->dma_transmit(serial, data, length, RT_SERIAL_DMA_RX);
    }
    else result = -RT_EBUSY;
    rt_hw_interrupt_enable(level);

    if (result == RT_EOK) return length;

    rt_set_errno(result);
    return 0;
}

rt_inline int _serial_dma_tx(struct rt_serial_device *serial, const rt_uint8_t *data, int length)
{
    rt_base_t level;
    rt_err_t result;
    struct rt_serial_tx_dma *tx_dma;

    tx_dma = (struct rt_serial_tx_dma*)(serial->serial_tx);
    
    result = rt_data_queue_push(&(tx_dma->data_queue), data, length, RT_WAITING_FOREVER); 
    if (result == RT_EOK)
    {
        level = rt_hw_interrupt_disable();
        if (tx_dma->activated != RT_TRUE)
        {
            tx_dma->activated = RT_TRUE;
            rt_hw_interrupt_enable(level);

            /* make a DMA transfer */
            serial->ops->dma_transmit(serial, data, length, RT_SERIAL_DMA_TX);
        }
        else
        {
            rt_hw_interrupt_enable(level);
        }

        return length;
    }
    else
    {
        rt_set_errno(result);
        return 0;
    }
}

/* RT-Thread Device Interface */
/*
 * This function initializes serial device.
 */
static rt_err_t rt_serial_init(struct rt_device *dev)
{
    rt_err_t result = RT_EOK;
    struct rt_serial_device *serial;

    RT_ASSERT(dev != RT_NULL);
    serial = (struct rt_serial_device *)dev;

    /* initialize rx/tx */
    serial->serial_rx = RT_NULL;
    serial->serial_tx = RT_NULL;

    /* apply configuration */
    if (serial->ops->configure)
        result = serial->ops->configure(serial, &serial->config);

    return result;
}

static rt_err_t rt_serial_open(struct rt_device *dev, rt_uint16_t oflag)
{
    struct rt_serial_device *serial;

    RT_ASSERT(dev != RT_NULL);
    serial = (struct rt_serial_device *)dev;

    /* check device flag with the open flag */
    if ((oflag & RT_DEVICE_FLAG_DMA_RX) && !(dev->flag & RT_DEVICE_FLAG_DMA_RX)) 
        return -RT_EIO;
    if ((oflag & RT_DEVICE_FLAG_DMA_TX) && !(dev->flag & RT_DEVICE_FLAG_DMA_TX))
        return -RT_EIO;
    if ((oflag & RT_DEVICE_FLAG_INT_RX) && !(dev->flag & RT_DEVICE_FLAG_INT_RX))
        return -RT_EIO;
    if ((oflag & RT_DEVICE_FLAG_INT_TX) && !(dev->flag & RT_DEVICE_FLAG_INT_TX))
        return -RT_EIO;

    /* get open flags */
    dev->open_flag = oflag & 0xff;
    
    /* initialize the Rx/Tx structure according to open flag */
    if (serial->serial_rx == RT_NULL)
    {
        if (oflag & RT_DEVICE_FLAG_DMA_RX)
        {
            struct rt_serial_rx_dma* rx_dma;

            rx_dma = (struct rt_serial_rx_dma*) rt_malloc (sizeof(struct rt_serial_rx_dma));
            RT_ASSERT(rx_dma != RT_NULL);
            rx_dma->activated = RT_FALSE;

            serial->serial_rx = rx_dma;
            dev->open_flag |= RT_DEVICE_FLAG_DMA_RX;
        }
        else if (oflag & RT_DEVICE_FLAG_INT_RX)
        {
            struct rt_serial_rx_fifo* rx_fifo;

            rx_fifo = (struct rt_serial_rx_fifo*) rt_malloc (sizeof(struct rt_serial_rx_fifo) + 
                serial->config.bufsz);
            RT_ASSERT(rx_fifo != RT_NULL);
            rx_fifo->buffer = (rt_uint8_t*) (rx_fifo + 1);
            rt_memset(rx_fifo->buffer, 0, RT_SERIAL_RB_BUFSZ);
            rx_fifo->put_index = 0;
            rx_fifo->get_index = 0;
            rx_fifo->counter = 0;
            rx_fifo->counter_ind = 0;

            serial->serial_rx = rx_fifo;
            dev->open_flag |= RT_DEVICE_FLAG_INT_RX;
            /* configure low level device */
            serial->ops->control(serial, RT_DEVICE_CTRL_IE_RXNE, (void *)1);
            serial->ops->control(serial, RT_DEVICE_CTRL_IE_IDLE, (void *)1);
            serial->ops->control(serial, RT_DEVICE_CTRL_IRQ, (void *)1);
        }
        else
        {
            serial->serial_rx = RT_NULL;
        }
    }

    if (serial->serial_tx == RT_NULL)
    {
        if (oflag & RT_DEVICE_FLAG_DMA_TX)
        {
            struct rt_serial_tx_dma* tx_dma;

            tx_dma = (struct rt_serial_tx_dma*) rt_malloc (sizeof(struct rt_serial_tx_dma));
            RT_ASSERT(tx_dma != RT_NULL);
            
            rt_data_queue_init(&(tx_dma->data_queue), 8, 4, RT_NULL);
            serial->serial_tx = tx_dma;

            dev->open_flag |= RT_DEVICE_FLAG_DMA_TX;
        }
        else if (oflag & RT_DEVICE_FLAG_INT_TX)
        {
            struct rt_serial_tx_fifo *tx_fifo;

            tx_fifo = (struct rt_serial_tx_fifo*) rt_malloc(sizeof(struct rt_serial_tx_fifo));
            RT_ASSERT(tx_fifo != RT_NULL);

            rt_completion_init(&(tx_fifo->completion));
            tx_fifo->ptr = RT_NULL;
            tx_fifo->len = 0;
            tx_fifo->bytes = 0;
            serial->serial_tx = tx_fifo;

            dev->open_flag |= RT_DEVICE_FLAG_INT_TX;
            /* configure low level device */
            //serial->ops->control(serial, RT_DEVICE_CTRL_IE_TXE, (void *)1);
            serial->ops->control(serial, RT_DEVICE_CTRL_IE_TC, (void *)1);
            serial->ops->control(serial, RT_DEVICE_CTRL_IRQ, (void *)1);
        }
        else
        {
            serial->serial_tx = RT_NULL;
        }
    }

    return RT_EOK;
}

static rt_err_t rt_serial_close(struct rt_device *dev)
{
    struct rt_serial_device *serial;

    RT_ASSERT(dev != RT_NULL);
    serial = (struct rt_serial_device *)dev;

    /* this device has more reference count */
    if (dev->ref_count > 1) return RT_EOK;
    
    if (dev->open_flag & RT_DEVICE_FLAG_INT_RX)
    {
        struct rt_serial_rx_fifo* rx_fifo;

        rx_fifo = (struct rt_serial_rx_fifo*)serial->serial_rx;
        RT_ASSERT(rx_fifo != RT_NULL);

        rt_free(rx_fifo);
        serial->serial_rx = RT_NULL;
        dev->open_flag &= ~RT_DEVICE_FLAG_INT_RX;
        /* configure low level device */
        serial->ops->control(serial, RT_DEVICE_CTRL_IE_RXNE, (void *)0);
        serial->ops->control(serial, RT_DEVICE_CTRL_IE_IDLE, (void *)0);
        serial->ops->control(serial, RT_DEVICE_CTRL_IRQ, (void *)0);
    }
    else if (dev->open_flag & RT_DEVICE_FLAG_DMA_RX)
    {
        struct rt_serial_rx_dma* rx_dma;

        rx_dma = (struct rt_serial_rx_dma*)serial->serial_rx;
        RT_ASSERT(rx_dma != RT_NULL);

        rt_free(rx_dma);
        serial->serial_rx = RT_NULL;
        dev->open_flag &= ~RT_DEVICE_FLAG_DMA_RX;
    }

    if (dev->open_flag & RT_DEVICE_FLAG_INT_TX)
    {
        struct rt_serial_tx_fifo* tx_fifo;

        tx_fifo = (struct rt_serial_tx_fifo*)serial->serial_tx;
        RT_ASSERT(tx_fifo != RT_NULL);

        rt_free(tx_fifo);
        serial->serial_tx = RT_NULL;
        dev->open_flag &= ~RT_DEVICE_FLAG_INT_TX;
        /* configure low level device */
        serial->ops->control(serial, RT_DEVICE_CTRL_IE_TXE, (void *)0);
        serial->ops->control(serial, RT_DEVICE_CTRL_IE_TC, (void *)0);
        serial->ops->control(serial, RT_DEVICE_CTRL_IRQ, (void *)0);
    }
    else if (dev->open_flag & RT_DEVICE_FLAG_DMA_TX)
    {
        struct rt_serial_tx_dma* tx_dma;

        tx_dma = (struct rt_serial_tx_dma*)serial->serial_tx;
        RT_ASSERT(tx_dma != RT_NULL);

        rt_free(tx_dma);
        serial->serial_tx = RT_NULL;
        dev->open_flag &= ~RT_DEVICE_FLAG_DMA_TX;
    }

    return RT_EOK;
}

static rt_size_t rt_serial_read(struct rt_device *dev,
                                rt_off_t          pos,
                                void             *buffer,
                                rt_size_t         size)
{
    struct rt_serial_device *serial;

    RT_ASSERT(dev != RT_NULL);
    if (size == 0) return 0;

    serial = (struct rt_serial_device *)dev;

    if (dev->open_flag & RT_DEVICE_FLAG_INT_RX)
    {
        return _serial_int_rx(serial, buffer, size);
    }
    else if (dev->open_flag & RT_DEVICE_FLAG_DMA_RX)
    {
        return _serial_dma_rx(serial, buffer, size);
    }

    return _serial_poll_rx(serial, buffer, size);
}

static rt_size_t rt_serial_write(struct rt_device *dev,
                                 rt_off_t          pos,
                                 const void       *buffer,
                                 rt_size_t         size)
{
    struct rt_serial_device *serial;

    RT_ASSERT(dev != RT_NULL);
    if (size == 0) return 0;

    serial = (struct rt_serial_device *)dev;

    if (dev->open_flag & RT_DEVICE_FLAG_INT_TX)
    {
        return _serial_int_tx(serial, buffer, size);
    }
    else if (dev->open_flag & RT_DEVICE_FLAG_DMA_TX)
    {
        return _serial_dma_tx(serial, buffer, size);
    }
    else
    {
        return _serial_poll_tx(serial, buffer, size);
    }
}

static rt_err_t rt_serial_control(struct rt_device *dev,
                                  rt_uint8_t        cmd,
                                  void             *args)
{
    struct rt_serial_device *serial;

    RT_ASSERT(dev != RT_NULL);
    serial = (struct rt_serial_device *)dev;

    switch (cmd)
    {
        case RT_DEVICE_CTRL_SUSPEND:
            /* suspend device */
            dev->flag |= RT_DEVICE_FLAG_SUSPENDED;
            break;

        case RT_DEVICE_CTRL_RESUME:
            /* resume device */
            dev->flag &= ~RT_DEVICE_FLAG_SUSPENDED;
            break;

        case RT_DEVICE_CTRL_CONFIG:
            /* configure device */
            serial->ops->configure(serial, (struct serial_configure *)args);
            break;
        case RT_DEVICE_CTRL_INT_IND:
            {
                rt_uint16_t int_ind = (rt_uint16_t)args;
                struct rt_serial_rx_fifo* rx_fifo;

                rx_fifo = (struct rt_serial_rx_fifo*)serial->serial_rx;
                RT_ASSERT(rx_fifo != RT_NULL);
                rx_fifo->counter_ind = int_ind;
            }
            break;

        default :
            /* control device */
            serial->ops->control(serial, cmd, args);
            break;
    }

    return RT_EOK;
}

/*
 * serial register
 */
rt_err_t rt_hw_serial_register(struct rt_serial_device *serial,
                               const char              *name,
                               rt_uint32_t              flag,
                               void                    *data)
{
    struct rt_device *device;
    RT_ASSERT(serial != RT_NULL);

    device = &(serial->parent);

    device->type        = RT_Device_Class_Char;
    device->rx_indicate = RT_NULL;
    device->tx_complete = RT_NULL;

    device->init        = rt_serial_init;
    device->open        = rt_serial_open;
    device->close       = rt_serial_close;
    device->read        = rt_serial_read;
    device->write       = rt_serial_write;
    device->control     = rt_serial_control;
    device->user_data   = data;

    /* register a character device */
    return rt_device_register(device, name, flag);
}

void rt_hw_serial_isr_int_rx(struct rt_serial_device *serial)
{
    struct stm32_uart* uart;
    uart = (struct stm32_uart *)serial->parent.user_data;

    if(USART_GetITStatus(uart->uart_device, USART_IT_RXNE) != RESET)
    {
        char ch;
        rt_base_t level;
        struct rt_serial_rx_fifo* rx_fifo;
        
        rx_fifo = (struct rt_serial_rx_fifo*)serial->serial_rx;
        RT_ASSERT(rx_fifo != RT_NULL);
        ch = (uart->uart_device->DR & 0xff);

        /* disable interrupt */
        level = rt_hw_interrupt_disable();
        {
            rx_fifo->buffer[rx_fifo->put_index] = ch;
            rx_fifo->put_index += 1;
            rx_fifo->counter += 1;
            if (rx_fifo->put_index >= serial->config.bufsz) rx_fifo->put_index = 0;
            
            /* if the next position is read index, discard this 'read char' */
            if (rx_fifo->put_index == rx_fifo->get_index)
            {
                rx_fifo->get_index += 1;
                if (rx_fifo->get_index >= serial->config.bufsz) rx_fifo->get_index = 0;
            }
        }
        /* enable interrupt */
        rt_hw_interrupt_enable(level);
    }
    if (USART_GetITStatus(uart->uart_device, USART_IT_IDLE) != RESET)
    {
        /* invoke callback */
        if (serial->parent.rx_indicate != RT_NULL)
        {
            rt_size_t rx_length;
            rt_base_t level;
            struct rt_serial_rx_fifo* rx_fifo;
            rx_fifo = (struct rt_serial_rx_fifo*)serial->serial_rx;
            RT_ASSERT(rx_fifo != RT_NULL);
        
            /* get rx length */
            level = rt_hw_interrupt_disable();
            rx_length = (rx_fifo->put_index >= rx_fifo->get_index)? (rx_fifo->put_index - rx_fifo->get_index):
                (serial->config.bufsz - (rx_fifo->get_index - rx_fifo->put_index));
            rt_hw_interrupt_enable(level);
        
            if (rx_fifo->counter_ind)
            {
                serial->parent.rx_indicate(&serial->parent, rx_fifo->counter);
                rx_fifo->counter = 0;
            }
            else
            {
                serial->parent.rx_indicate(&serial->parent, rx_length);
            }
        }

        /* clear interrupt */
        uart->uart_device->SR;
        uart->uart_device->DR;
        //USART_ClearITPendingBit(uart->uart_device, USART_IT_IDLE);
    }
    /* error */
    if (USART_GetFlagStatus(uart->uart_device, USART_FLAG_ORE) == SET)
    {
        uart->uart_device->SR;
        uart->uart_device->DR;
    }
    if (USART_GetFlagStatus(uart->uart_device, USART_FLAG_NE) == SET)
    {
        uart->uart_device->SR;
        uart->uart_device->DR;
    }
    if (USART_GetFlagStatus(uart->uart_device, USART_FLAG_FE) == SET)
    {
        uart->uart_device->SR;
        uart->uart_device->DR;
    }
    if (USART_GetFlagStatus(uart->uart_device, USART_FLAG_PE) == SET)
    {
        uart->uart_device->SR;
        uart->uart_device->DR;
    }
}
void rt_hw_serial_isr_int_tx(struct rt_serial_device *serial)
{
    struct stm32_uart* uart;
    uart = (struct stm32_uart *)serial->parent.user_data;

    if (USART_GetITStatus(uart->uart_device, USART_IT_TXE) != RESET)
    {
        struct rt_serial_tx_fifo* tx_fifo;
        tx_fifo = (struct rt_serial_tx_fifo*)serial->serial_tx;
        if (tx_fifo->bytes < tx_fifo->len)
        {
            /*
             * to be polite with serial console add a line feed
             * to the carriage return character
             */
            if (tx_fifo->ptr[tx_fifo->bytes] == CHAR_NEWLINE && 
                (serial->parent.open_flag & RT_DEVICE_FLAG_STREAM))
            {
                while(!(uart->uart_device->SR & USART_FLAG_TXE));
                uart->uart_device->DR = CHAR_CARRIAGE_RETURN;
            }
            
            while(!(uart->uart_device->SR & USART_FLAG_TXE));


            uart->uart_device->DR = tx_fifo->ptr[tx_fifo->bytes++];
        }
        else
        {
            serial->ops->control(serial, RT_DEVICE_CTRL_IE_TXE, (void *)0);
            rt_completion_done(&(tx_fifo->completion));
        }
    }
    if (USART_GetITStatus(uart->uart_device, USART_IT_TC) != RESET)
    {
        USART_ClearITPendingBit(uart->uart_device, USART_IT_TC);
    }
}
void rt_hw_serial_isr_poll_rx(struct rt_serial_device *serial)
{
}
void rt_hw_serial_isr_poll_tx(struct rt_serial_device *serial)
{
}

/* ISR for serial interrupt */
void rt_hw_serial_isr(struct rt_serial_device *serial)
{
    if (serial->parent.open_flag & RT_DEVICE_FLAG_INT_RX)
    {
        rt_hw_serial_isr_int_rx(serial);
    }
    if (serial->parent.open_flag & RT_DEVICE_FLAG_INT_TX)
    {
        rt_hw_serial_isr_int_tx(serial);
    }
    
    if (serial->parent.open_flag & RT_DEVICE_FLAG_DMA_RX)
    {
        RT_ASSERT(RT_FALSE);
    }
    if (serial->parent.open_flag & RT_DEVICE_FLAG_DMA_TX)
    {
        RT_ASSERT(RT_FALSE);
    }
}

#endif /*RT_USING_UARTX*/
