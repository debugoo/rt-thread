/**
* Copyright (C), 2015-2015, Chengdu QiPeng technology. Co., Ltd.
*   File name: uartx.h
*      Author: XuJun
*     Version: 1.0.0
*        Date: 18-05-2016
* Description: 
*              re-implements uart device driver
*              using hardware feature of UART as much more
* Missed:
*              1. DMA implements
*            
* Change Logs:
* Date            Author           Notes
* 10-05-2015      XuJun            the first version
*/

#ifndef __UART_X_H__
#define __UART_X_H__

/* this driver is re-implements serial and:
 *   1. it will conflict with serial implements by RTT
 *   2. application should using either SERIALX or SERIAL
 */
#include <rtconfig.h>
#ifdef RT_USING_UARTX

#include <rtthread.h>
#include <rtdevice.h>

/****************************************************************************
 * hardware
 ***************************************************************************/
extern void rt_hw_usart_init(void);

/****************************************************************************
 * driver
 ***************************************************************************/
#define UART1_NAME                      "uart1"
#define UART2_NAME                      "uart2"
#define UART3_NAME                      "uart3"
#define UART4_NAME                      "uart4"
#define UART5_NAME                      "uart5"

#define BAUD_RATE_2400                  2400
#define BAUD_RATE_4800                  4800
#define BAUD_RATE_9600                  9600
#define BAUD_RATE_38400                 38400
#define BAUD_RATE_57600                 57600
#define BAUD_RATE_115200                115200
#define BAUD_RATE_230400                230400
#define BAUD_RATE_460800                460800
#define BAUD_RATE_921600                921600

#define DATA_BITS_5                     5
#define DATA_BITS_6                     6
#define DATA_BITS_7                     7
#define DATA_BITS_8                     8
#define DATA_BITS_9                     9

#define STOP_BITS_1                     0
#define STOP_BITS_2                     1
#define STOP_BITS_3                     2
#define STOP_BITS_4                     3

#define PARITY_NONE                     0
#define PARITY_ODD                      1
#define PARITY_EVEN                     2

#define BIT_ORDER_LSB                   0
#define BIT_ORDER_MSB                   1

#define NRZ_NORMAL                      0       /* Non Return to Zero : normal mode */
#define NRZ_INVERTED                    1       /* Non Return to Zero : inverted mode */

#ifndef RT_SERIAL_RB_BUFSZ
#define RT_SERIAL_RB_BUFSZ              64
#endif

#define RT_DEVICE_CTRL_CONFIG           0x03    /* configure device */
#define RT_DEVICE_CTRL_SET_INT          0x10    /* enable receive irq */
#define RT_DEVICE_CTRL_CLR_INT          0x11    /* disable receive irq */
#define RT_DEVICE_CTRL_GET_INT          0x12

/* control interrupt enabled bits */
#define RT_DEVICE_CTRL_IE_PE            0x21
#define RT_DEVICE_CTRL_IE_TXE           0x22
#define RT_DEVICE_CTRL_IE_TC            0x23
#define RT_DEVICE_CTRL_IE_RXNE          0x24
#define RT_DEVICE_CTRL_IE_IDLE          0x25
#define RT_DEVICE_CTRL_IE_ERROR         0x26
#define RT_DEVICE_CTRL_IRQ              0x27
#define RT_DEVICE_CTRL_INT_IND          0x28

#define RT_SERIAL_EVENT_RX_IND          0x01    /* Rx indication */
#define RT_SERIAL_EVENT_TX_DONE         0x02    /* Tx complete   */
#define RT_SERIAL_EVENT_RX_DMADONE      0x03    /* Rx DMA transfer done */
#define RT_SERIAL_EVENT_TX_DMADONE      0x04    /* Tx DMA transfer done */
#define RT_SERIAL_EVENT_RX_TIMEOUT      0x05    /* Rx timeout    */

#define RT_SERIAL_DMA_RX                0x01
#define RT_SERIAL_DMA_TX                0x02

#define RT_SERIAL_RX_INT                0x01
#define RT_SERIAL_TX_INT                0x02

#define RT_SERIAL_ERR_OVERRUN           0x01
#define RT_SERIAL_ERR_FRAMING           0x02
#define RT_SERIAL_ERR_PARITY            0x03

#define RT_SERIAL_TX_DATAQUEUE_SIZE     2048
#define RT_SERIAL_TX_DATAQUEUE_LWM      30

#define CHAR_CARRIAGE_RETURN     '\r'
#define CHAR_NEWLINE             '\n'

/* Default config for serial_configure structure */
#define RT_SERIAL_CONFIG_DEFAULT           \
{                                          \
    BAUD_RATE_115200, /* 115200 bits/s */  \
    DATA_BITS_8,      /* 8 databits */     \
    STOP_BITS_1,      /* 1 stopbit */      \
    PARITY_NONE,      /* No parity  */     \
    BIT_ORDER_LSB,    /* LSB first sent */ \
    NRZ_NORMAL,       /* Normal mode */    \
    RT_SERIAL_RB_BUFSZ, /* Buffer size */  \
    0                                      \
}

struct serial_configure
{
    rt_uint32_t baud_rate;

    rt_uint32_t data_bits               :4;
    rt_uint32_t stop_bits               :2;
    rt_uint32_t parity                  :2;
    rt_uint32_t bit_order               :1;
    rt_uint32_t invert                  :1;
	rt_uint32_t bufsz					:16;
    rt_uint32_t reserved                :4;
};

/*
 * Serial FIFO mode 
 */
struct rt_serial_rx_fifo
{
	/* software fifo */
	rt_uint8_t *buffer;

	rt_uint16_t put_index, get_index;
	rt_uint16_t counter, counter_ind;
};

struct rt_serial_tx_fifo
{
    char  *ptr;
    int   len;
    int   bytes;
	struct rt_completion completion;
};

/* 
 * Serial DMA mode
 */
struct rt_serial_rx_dma
{
	rt_bool_t activated;
};

struct rt_serial_tx_dma
{
	rt_bool_t activated;
	struct rt_data_queue data_queue;
};

struct rt_serial_device
{
    struct rt_device          parent;

    const struct rt_uart_ops *ops;
    struct serial_configure   config;

	void *serial_rx;
	void *serial_tx;
};
typedef struct rt_serial_device rt_serial_t;

/**
 * uart operators
 */
struct rt_uart_ops
{
    rt_err_t (*configure)(struct rt_serial_device *serial, struct serial_configure *cfg);
    rt_err_t (*control)(struct rt_serial_device *serial, int cmd, void *arg);
    rt_size_t (*dma_transmit)(struct rt_serial_device *serial, const rt_uint8_t *buf, rt_size_t size, int direction);
};

void rt_hw_serial_isr(struct rt_serial_device *serial);

rt_err_t rt_hw_serial_register(struct rt_serial_device *serial,
                               const char              *name,
                               rt_uint32_t              flag,
                               void                    *data);

#endif /*RT_USING_UARTX*/

#endif /*__UART_X_H__*/
