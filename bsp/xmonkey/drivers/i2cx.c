/**
* Copyright (C), 2015-2015, Chengdu QiPeng technology. Co., Ltd.
*   File name: i2cx.h
*      Author: XuJun
*     Version: 1.0.0
*        Date: 10-05-2016
* Description: 
*              implements i2c driver with hardware i2c of ST
*              only implements master transmitter/receiver(not included slave func)
* Missed:
*              1. i2c slave
*              2. rt_device_control
*            
* Change Logs:
* Date            Author           Notes
* 10-05-2015      XuJun            the first version
*/
#include <rtconfig.h>
#ifdef RT_USING_I2CX

#include "i2cx.h"
#include <rtconfig.h>
#include <rtthread.h>
#include <stm32f4xx.h>
#include <string.h>
#include <stm32x.h>

#if defined(RT_USING_I2C1)

/* I2C1 */
#define I2C1_GPIO_SCL        GPIO_Pin_8
#define I2C1_GPIO_SDA        GPIO_Pin_9
#define I2C1_GPIO            GPIOB
#define I2C1_GPIO_SCL_SRC    GPIO_PinSource8
#define I2C1_GPIO_SDA_SRC    GPIO_PinSource9
#define I2C1_TIM_NAME        "i2c1t"

/* I2C1 HW define */
#define I2C1_TX_DMA_STREAM            DMA1_Stream7
#define I2C1_TX_DMA_CHANNEL           DMA_Channel_1
#define I2C1_TX_DMA_FLAG_FEIF         DMA_FLAG_FEIF7
#define I2C1_TX_DMA_FLAG_DMEIF        DMA_FLAG_DMEIF7
#define I2C1_TX_DMA_FLAG_TEIF         DMA_FLAG_TEIF7
#define I2C1_TX_DMA_FLAG_HTIF         DMA_FLAG_HTIF7
#define I2C1_TX_DMA_FLAG_TCIF         DMA_FLAG_TCIF7
#define I2C1_TX_DMA_IRQn              DMA1_Stream7_IRQn
#define I2C1_TX_DMA_IRQHANDLER        DMA1_Stream7_IRQHandler

#define I2C1_RX_DMA_STREAM            DMA1_Stream0
#define I2C1_RX_DMA_CHANNEL           DMA_Channel_1
#define I2C1_RX_DMA_FLAG_FEIF         DMA_FLAG_FEIF0
#define I2C1_RX_DMA_FLAG_DMEIF        DMA_FLAG_DMEIF0
#define I2C1_RX_DMA_FLAG_TEIF         DMA_FLAG_TEIF0
#define I2C1_RX_DMA_FLAG_HTIF         DMA_FLAG_HTIF0
#define I2C1_RX_DMA_FLAG_TCIF         DMA_FLAG_TCIF0
#define I2C1_RX_DMA_IRQn              DMA1_Stream0_IRQn
#define I2C1_RX_DMA_IRQHANDLER        DMA1_Stream0_IRQHandler

struct rt_i2c_bus_device i2c_bus1;

#endif

#if defined(RT_USING_I2C2)
/* I2C2 */
#define I2C2_GPIO_SCL        GPIO_Pin_10
#define I2C2_GPIO_SDA        GPIO_Pin_11
#define I2C2_GPIO            GPIOB
#define I2C2_GPIO_SCL_SRC    GPIO_PinSource10
#define I2C2_GPIO_SDA_SRC    GPIO_PinSource11
#define I2C2_TIM_NAME        "i2c2t"

/* I2C2 HW define */
#define I2C2_TX_DMA_STREAM            DMA1_Stream7
#define I2C2_TX_DMA_CHANNEL           DMA_Channel_7
#define I2C2_TX_DMA_FLAG_FEIF         DMA_FLAG_FEIF7
#define I2C2_TX_DMA_FLAG_DMEIF        DMA_FLAG_DMEIF7
#define I2C2_TX_DMA_FLAG_TEIF         DMA_FLAG_TEIF7
#define I2C2_TX_DMA_FLAG_HTIF         DMA_FLAG_HTIF7
#define I2C2_TX_DMA_FLAG_TCIF         DMA_FLAG_TCIF7
#define I2C2_TX_DMA_IRQn              DMA1_Stream7_IRQn
#define I2C2_TX_DMA_IRQHANDLER        DMA1_Stream7_IRQHandler

#define I2C2_RX_DMA_STREAM            DMA1_Stream2
#define I2C2_RX_DMA_CHANNEL           DMA_Channel_7
#define I2C2_RX_DMA_FLAG_FEIF         DMA_FLAG_FEIF2
#define I2C2_RX_DMA_FLAG_DMEIF        DMA_FLAG_DMEIF2
#define I2C2_RX_DMA_FLAG_TEIF         DMA_FLAG_TEIF2
#define I2C2_RX_DMA_FLAG_HTIF         DMA_FLAG_HTIF2
#define I2C2_RX_DMA_FLAG_TCIF         DMA_FLAG_TCIF2
#define I2C2_RX_DMA_IRQn              DMA1_Stream2_IRQn
#define I2C2_RX_DMA_IRQHANDLER        DMA1_Stream2_IRQHandler

struct rt_i2c_bus_device i2c_bus2;

#endif

static void transfer_complete(struct rt_i2c_bus_device* bus)
{
    if (bus->tim.parent.flag & RT_TIMER_FLAG_ACTIVATED)
    {
        rt_timer_stop(&bus->tim);
    }
#ifdef I2CX_SYNC_SEMAPHORE
    rt_sem_release(&bus->sem);
#else
    rt_completion_done(&bus->completion);
#endif
}

static rt_err_t i2c_bus_device_init(rt_device_t dev)
{
    I2C_InitTypeDef  I2C_InitStructure;
    struct rt_i2c_bus_device *bus = (struct rt_i2c_bus_device *)dev;

    RT_ASSERT(bus->config.master_addr);
    RT_ASSERT(bus->config.speed);
    RT_ASSERT(bus->config.timeout);
    
    /* I2C configuration */
    I2C_InitStructure.I2C_Mode                = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle           = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1         = bus->config.master_addr;
    I2C_InitStructure.I2C_Ack                 = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed          = bus->config.speed;

    I2C_DeInit(bus->i2c);
    I2C_AcknowledgeConfig(bus->i2c, ENABLE);
    
    /* Disable clock stretch */
    //I2C_StretchClockCmd(bus->i2c, DISABLE);
    
    /* Apply I2C configuration after enabling it */
    I2C_Init(bus->i2c, &I2C_InitStructure);

    return RT_EOK;
}

static rt_err_t i2c_bus_device_open(rt_device_t dev, rt_uint16_t oflags)
{
    struct rt_i2c_bus_device *bus = (struct rt_i2c_bus_device *)dev;

    bus->i2c->CR1 |= I2C_CR1_BIT_PE;

    return RT_EOK;
}

static rt_err_t i2c_bus_device_close(rt_device_t dev)
{
    struct rt_i2c_bus_device *bus = (struct rt_i2c_bus_device *)dev;

    bus->i2c->CR1 &= ~I2C_CR1_BIT_PE;

    return RT_EOK;
}

#if defined(RT_USING_I2C1)
void I2C1_LowLevel_DMA_TxConfig(rt_device_t dev, rt_uint8_t *BufferSRC, rt_uint16_t BufferSize)
{
  DMA_InitTypeDef SDDMA_InitStructure;
  struct rt_i2c_bus_device *bus = (struct rt_i2c_bus_device *)dev;

  DMA_ClearFlag(I2C1_TX_DMA_STREAM, I2C1_TX_DMA_FLAG_FEIF | I2C1_TX_DMA_FLAG_DMEIF | I2C1_TX_DMA_FLAG_TEIF | I2C1_TX_DMA_FLAG_HTIF | I2C1_TX_DMA_FLAG_TCIF);

  /* DMA Stream disable */
  DMA_Cmd(I2C1_TX_DMA_STREAM, DISABLE);

  /* DMA Stream config */
  DMA_DeInit(I2C1_TX_DMA_STREAM);

  SDDMA_InitStructure.DMA_Channel = I2C1_TX_DMA_CHANNEL;
  SDDMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&(bus->i2c->DR);
  SDDMA_InitStructure.DMA_Memory0BaseAddr = (u32)BufferSRC;
  SDDMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  SDDMA_InitStructure.DMA_BufferSize = BufferSize;
  SDDMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  SDDMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  SDDMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  SDDMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  SDDMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  SDDMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
  SDDMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
  SDDMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  SDDMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_INC4;
  SDDMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_INC4;
  DMA_Init(I2C1_TX_DMA_STREAM, &SDDMA_InitStructure);

  DMA_FlowControllerConfig(I2C1_TX_DMA_STREAM, DMA_FlowCtrl_Memory);
  DMA_ITConfig(I2C1_TX_DMA_STREAM, DMA_IT_TC|DMA_IT_TE|DMA_IT_HT, ENABLE);

  /* DMA Stream enable */
  DMA_Cmd(I2C1_TX_DMA_STREAM, ENABLE);
}
void I2C1_LowLevel_DMA_RxConfig(rt_device_t dev, rt_uint8_t *BufferDST, rt_uint16_t BufferSize)
{
  DMA_InitTypeDef SDDMA_InitStructure;
  struct rt_i2c_bus_device *bus = (struct rt_i2c_bus_device *)dev;

  DMA_ClearFlag(I2C1_RX_DMA_STREAM, I2C1_RX_DMA_FLAG_FEIF | I2C1_RX_DMA_FLAG_DMEIF | I2C1_RX_DMA_FLAG_TEIF | I2C1_RX_DMA_FLAG_HTIF | I2C1_RX_DMA_FLAG_TCIF);

  /* DMA Stream disable */
  DMA_Cmd(I2C1_RX_DMA_STREAM, DISABLE);

  /* DMA Stream config */
  DMA_DeInit(I2C1_RX_DMA_STREAM);

  SDDMA_InitStructure.DMA_Channel = I2C1_RX_DMA_CHANNEL;
  SDDMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&(bus->i2c->DR);
  SDDMA_InitStructure.DMA_Memory0BaseAddr = (u32)BufferDST;
  SDDMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  SDDMA_InitStructure.DMA_BufferSize = BufferSize;
  SDDMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  SDDMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  SDDMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  SDDMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  SDDMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  SDDMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
  SDDMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
  SDDMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  SDDMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_INC4;
  SDDMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_INC4;
  DMA_Init(I2C1_RX_DMA_STREAM, &SDDMA_InitStructure);

  DMA_FlowControllerConfig(I2C1_RX_DMA_STREAM, DMA_FlowCtrl_Memory);

  DMA_ITConfig(I2C1_RX_DMA_STREAM, DMA_IT_TC|DMA_IT_TE|DMA_IT_HT, ENABLE);

  /* DMA Stream enable */
  DMA_Cmd(I2C1_RX_DMA_STREAM, ENABLE);
}
#endif

#if defined(RT_USING_I2C2)
void I2C2_LowLevel_DMA_TxConfig(rt_device_t dev, rt_uint8_t *BufferSRC, rt_uint16_t BufferSize)
{
  DMA_InitTypeDef SDDMA_InitStructure;
  struct rt_i2c_bus_device *bus = (struct rt_i2c_bus_device *)dev;

  DMA_ClearFlag(I2C2_TX_DMA_STREAM, I2C2_TX_DMA_FLAG_FEIF | I2C2_TX_DMA_FLAG_DMEIF | I2C2_TX_DMA_FLAG_TEIF | I2C2_TX_DMA_FLAG_HTIF | I2C2_TX_DMA_FLAG_TCIF);

  /* DMA Stream disable */
  DMA_Cmd(I2C2_TX_DMA_STREAM, DISABLE);

  /* DMA Stream config */
  DMA_DeInit(I2C2_TX_DMA_STREAM);

  SDDMA_InitStructure.DMA_Channel = I2C2_TX_DMA_CHANNEL;
  SDDMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&(bus->i2c->DR);
  SDDMA_InitStructure.DMA_Memory0BaseAddr = (u32)BufferSRC;
  SDDMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  SDDMA_InitStructure.DMA_BufferSize = BufferSize;
  SDDMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  SDDMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  SDDMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  SDDMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  SDDMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  SDDMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
  SDDMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
  SDDMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  SDDMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_INC4;
  SDDMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_INC4;
  DMA_Init(I2C2_TX_DMA_STREAM, &SDDMA_InitStructure);

  DMA_FlowControllerConfig(I2C2_TX_DMA_STREAM, DMA_FlowCtrl_Memory);

  /* DMA Stream enable */
  DMA_Cmd(I2C2_TX_DMA_STREAM, ENABLE);
}
void I2C2_LowLevel_DMA_RxConfig(rt_device_t dev, rt_uint8_t *BufferDST, rt_uint16_t BufferSize)
{
  DMA_InitTypeDef SDDMA_InitStructure;
  struct rt_i2c_bus_device *bus = (struct rt_i2c_bus_device *)dev;

  DMA_ClearFlag(I2C2_RX_DMA_STREAM, I2C2_RX_DMA_FLAG_FEIF | I2C2_RX_DMA_FLAG_DMEIF | I2C2_RX_DMA_FLAG_TEIF | I2C2_RX_DMA_FLAG_HTIF | I2C2_RX_DMA_FLAG_TCIF);

  /* DMA Stream disable */
  DMA_Cmd(I2C2_RX_DMA_STREAM, DISABLE);

  /* DMA Stream config */
  DMA_DeInit(I2C2_RX_DMA_STREAM);

  SDDMA_InitStructure.DMA_Channel = I2C2_RX_DMA_CHANNEL;
  SDDMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&(bus->i2c->DR);
  SDDMA_InitStructure.DMA_Memory0BaseAddr = (u32)BufferDST;
  SDDMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  SDDMA_InitStructure.DMA_BufferSize = BufferSize;
  SDDMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  SDDMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  SDDMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  SDDMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  SDDMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  SDDMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
  SDDMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
  SDDMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  SDDMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_INC4;
  SDDMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_INC4;
  DMA_Init(I2C2_RX_DMA_STREAM, &SDDMA_InitStructure);

  DMA_FlowControllerConfig(I2C2_RX_DMA_STREAM, DMA_FlowCtrl_Memory);

  DMA_ITConfig(I2C2_RX_DMA_STREAM, DMA_IT_TC|DMA_IT_TE|DMA_IT_HT, ENABLE);

  /* DMA Stream enable */
  DMA_Cmd(I2C2_RX_DMA_STREAM, ENABLE);
}
#endif

/*
 * Function: i2c_bus_device_read
 * ----------------------------
 *   Read any bytes from device
 *
 *   dev    : Source device
 *   pos    : The I2C address of dev, This is the 8 bits address with LSB bit 0
 *   buffer : The buffer which holder the bytes to be read
 *   count  : The bytes count in buffer
 *
 *   returns: RT_EOK when read successfully, otherwise indicate some error occurs
 */
static rt_size_t i2c_bus_device_read(rt_device_t dev,
                                     rt_off_t    pos,
                                     void       *buffer,
                                     rt_size_t   count)
{
    struct rt_i2c_bus_device *bus = (struct rt_i2c_bus_device *)dev;
    struct rt_i2c_trans *trans = &bus->trans;

    RT_ASSERT( bus        != RT_NULL);
    RT_ASSERT( buffer     != RT_NULL);
    RT_ASSERT( count       > 0      );
    RT_ASSERT( (pos&0xFF) != 0      );

    if (pos&0xFF == 0)
    {
        return -RT_ERROR;
    }
    if (count < 1)
    {
        return -RT_ERROR;
    }
    if (bus->i2c->SR2 & I2C_SR2_BIT_BUSY)
    {
        RT_ASSERT(RT_FALSE);
        return -RT_EBUSY;
    }

    trans->slave_addr  = (rt_uint8_t)pos;
    trans->buf         = (rt_uint8_t*)buffer;
    trans->len         = (rt_uint16_t)count;
    trans->mode       &= ~I2C_TRANS_FLAG_TX;
    trans->transferred = 0;
    
    bus->i2c->CR2 |= (I2C_CR2_BIT_ITERREN|I2C_CR2_BIT_ITEVTEN);
    bus->i2c->CR2 &= ~I2C_CR2_BIT_ITBUFEN;
    bus->i2c->CR1 |= I2C_CR1_BIT_ACK;

    if (trans->len > 1)
    {
        bus->dma_rx_conf(dev, trans->buf, trans->len);
        bus->dma_stream = bus->dma_rx_stream;
    }
    else
    {
        bus->i2c->CR2 |= I2C_CR2_BIT_ITBUFEN;
    }
    
    /* Start I2C operation */
    //bus->error     = -I2C_E_PENDING;
    bus->error = RT_EOK;
    bus->i2c->CR1 |= I2C_CR1_BIT_START;

    /* Start timer */
    rt_timer_control(&bus->tim, RT_TIMER_CTRL_SET_TIME, (void*)(&bus->config.timeout));
    rt_timer_start(&bus->tim);

    return -I2C_E_PENDING;
}

static rt_size_t i2c_bus_device_read_sync(rt_device_t dev,
                                          rt_off_t    pos,
                                          void       *buffer,
                                          rt_size_t   count)
{
    struct rt_i2c_bus_device *bus = (struct rt_i2c_bus_device *)dev;
    rt_size_t size;

#ifdef I2CX_SYNC_SEMAPHORE
#else
    rt_err_t err;
    rt_completion_reset(&bus->completion);
#endif

    size = i2c_bus_device_read(dev, pos, buffer, count);
    if (size != -I2C_E_PENDING)
    {
        return size;
    }

    /* wait for complete */
#ifdef I2CX_SYNC_SEMAPHORE
    rt_sem_take(&bus->sem, RT_WAITING_FOREVER);
#else
    err = rt_completion_wait(&bus->completion, RT_WAITING_FOREVER);
    //bus->i2c->CR2 &= ~(I2C_CR2_BIT_ITERREN|I2C_CR2_BIT_ITEVTEN|I2C_CR2_BIT_ITBUFEN);
    while(bus->i2c->SR2 & I2C_SR2_BIT_BUSY);
    if (err != RT_EOK)
        return err;
#endif

    if (bus->error == RT_EOK)
        return bus->trans.transferred;
    RT_ASSERT(bus->error < 0);
    return bus->error;
}

/*
 * Function: i2c_bus_device_write
 * ----------------------------
 *   Write any bytes to device
 *
 *   dev    : Target device
 *   pos    : The I2C address of dev, This is the 8 bits address with LSB bit 0
 *   buffer : The buffer which holder the bytes to be written
 *   count  : The bytes count in buffer
 *
 *   returns: RT_EOK when write successfully, otherwise indicate some error occurs
 */
static rt_size_t i2c_bus_device_write(rt_device_t dev,
                                      rt_off_t    pos,
                                      const void *buffer,
                                      rt_size_t   count)
{
    rt_err_t err = RT_EOK;
    struct rt_i2c_bus_device *bus = (struct rt_i2c_bus_device *)dev;
    struct rt_i2c_trans *trans = &bus->trans;

    RT_ASSERT( bus       != RT_NULL);
    RT_ASSERT( buffer    != RT_NULL);
    RT_ASSERT( count      > 0);
    RT_ASSERT((pos&0xFF) != 0);

    if (pos&0xFF == 0)
    {
        return -RT_ERROR;
    }
    if (count < 1)
    {
        return -RT_ERROR;
    }
    if (bus->i2c->SR2 & I2C_SR2_BIT_BUSY)
    {
        return -RT_EBUSY;
    }

    trans->slave_addr  = (rt_uint8_t)pos;
    trans->buf         = (rt_uint8_t*)buffer;
    trans->len         = (rt_uint16_t)count;
    trans->mode       |= I2C_TRANS_FLAG_TX;
    trans->transferred = 0;

    bus->i2c->CR2 |= (I2C_CR2_BIT_ITERREN|I2C_CR2_BIT_ITEVTEN);
    bus->i2c->CR2 &= ~I2C_CR2_BIT_ITBUFEN;
    bus->i2c->CR1 |= I2C_CR1_BIT_ACK;

    if (trans->len > 1)
    {
        bus->dma_tx_conf(dev, trans->buf, trans->len);
        bus->dma_stream = bus->dma_tx_stream;
    }
    else
    {
        bus->i2c->CR2 |= I2C_CR2_BIT_ITBUFEN;
    }

    /* Start I2C operation */
    //bus->error = -I2C_E_PENDING;
    bus->error = RT_EOK;
    bus->i2c->CR1 |= I2C_CR1_BIT_START;
    
    /* Start timer */
    rt_timer_control(&bus->tim, RT_TIMER_CTRL_SET_TIME, (void*)(&bus->config.timeout));
    err = rt_timer_start(&bus->tim);
    RT_ASSERT(err == RT_EOK);

    return -I2C_E_PENDING;
}

static rt_size_t i2c_bus_device_write_sync(rt_device_t dev,
                                           rt_off_t    pos,
                                           const void *buffer,
                                           rt_size_t   count)
{
    struct rt_i2c_bus_device *bus = (struct rt_i2c_bus_device *)dev;
    rt_size_t size;

#ifdef I2CX_SYNC_SEMAPHORE
#else
    rt_err_t err;
    rt_completion_reset(&bus->completion);
#endif

    size = i2c_bus_device_write(dev, pos, buffer, count);
    if (size != -I2C_E_PENDING)
    {
        return size;
    }

    /* wait for complete */
#ifdef I2CX_SYNC_SEMAPHORE
    rt_sem_take(&bus->sem, RT_WAITING_FOREVER);
#else
    err = rt_completion_wait(&bus->completion, RT_WAITING_FOREVER);
    //bus->i2c->CR2 &= ~(I2C_CR2_BIT_ITERREN|I2C_CR2_BIT_ITEVTEN|I2C_CR2_BIT_ITBUFEN);
    while(bus->i2c->SR2 & I2C_SR2_BIT_BUSY);
    if (err != RT_EOK)
        return err;
#endif

    if (bus->error == RT_EOK)
        return bus->trans.transferred;
    RT_ASSERT(bus->error < 0);
    return bus->error;
}

static rt_err_t i2c_bus_device_control(rt_device_t dev,
                                       rt_uint8_t  cmd,
                                       void       *args)
{
    struct rt_i2c_bus_device *bus = (struct rt_i2c_bus_device *)dev;
    //struct i2c_configuration* conf = RT_NULL;
    rt_err_t *err;

    RT_ASSERT(bus != RT_NULL);

    switch (cmd)
    {
    case RT_I2C_DEV_CTRL_ADDR:
        //bus->addr = *(rt_uint16_t *)args;
        break;
    case RT_I2C_DEV_CTRL_TIMEOUT:
        //bus->timeout = *(rt_uint32_t *)args;
        break;
    case RT_I2C_DEV_CTRL_GETERR:
        RT_ASSERT(args);
        err = (rt_err_t*)args;
        *err = bus->error;
        break;
    case RT_I2C_DEV_CTRL_CONFIG:
        RT_ASSERT(RT_FALSE);
        break;
    default:
        break;
    }

    return RT_EOK;
}

static void i2c_transfer_timeout(void *parameter)
{
    struct rt_i2c_bus_device *bus = (struct rt_i2c_bus_device*)parameter;

    bus->error = -RT_ETIMEOUT;

    /* make sure the bus is released */
    bus->i2c->CR1 &= ~I2C_CR1_BIT_PE;
    bus->i2c->CR1 |= I2C_CR1_BIT_PE;

    transfer_complete(bus);
}

void I2C_EV_DMA_Handler(struct rt_i2c_bus_device* bus)
{
    rt_uint16_t sr1=0;
    I2C_TypeDef* i2c = bus->i2c;
    struct rt_i2c_trans *trans = &bus->trans;
    rt_uint8_t addr = bus->trans.slave_addr;
    
    sr1 = i2c->SR1;
    if (trans->mode & I2C_TRANS_FLAG_TX)
    {
        if ( (sr1&I2C_TRANSFER_EV5) == I2C_TRANSFER_EV5)
        {
            addr &= ~I2C_ADDR_BIT_RX;
            i2c->DR = addr;
        }
        else if ( (sr1&I2C_TRANSFER_EV6) == I2C_TRANSFER_EV6)
        {
            rt_uint16_t sr2;
            sr2 = i2c->SR2;
            i2c->CR2 |= I2C_CR2_BIT_DMAEN;
        }
        else if ( (sr1&I2C_TRANSFER_EV8_2) == I2C_TRANSFER_EV8_2)
        {
            i2c->CR1 |= I2C_CR1_BIT_STOP;
            bus->trans.transferred = bus->trans.len;
            transfer_complete(bus);
        }
    }
    else
    {
        if ( (sr1&I2C_TRANSFER_EV5) == I2C_TRANSFER_EV5)
        {
            addr |= I2C_ADDR_BIT_RX;
            i2c->DR = addr;
        }
        else if ( (sr1&I2C_TRANSFER_EV6) == I2C_TRANSFER_EV6)
        {
            rt_uint16_t sr2;
            i2c->CR2 |= I2C_CR2_BIT_LAST;
            i2c->CR2 |= I2C_CR2_BIT_DMAEN;
            sr1 = i2c->SR1;
            sr2 = i2c->SR2;
        }
    }
}

void I2C_EV_RxInt_Handler(struct rt_i2c_bus_device* bus)
{
    u16 sr1;
    rt_uint8_t addr = bus->trans.slave_addr|I2C_ADDR_BIT_RX;
    I2C_TypeDef* i2c = bus->i2c;

    sr1 = i2c->SR1;
    if ( (sr1&I2C_TRANSFER_EV5) == I2C_TRANSFER_EV5)
    {
        //i2c->CR1 &= ~I2C_CR1_BIT_ACK;
        i2c->DR = addr;
    }
    if ( (sr1&I2C_TRANSFER_EV6) == I2C_TRANSFER_EV6)
    {
        u16 sr2;
        sr2 = i2c->SR2;
        /* EV6_1, we will just recieved one byte */
        i2c->CR1 &= ~I2C_CR1_BIT_ACK;
        i2c->CR1 |= I2C_CR1_BIT_STOP;
    }
    if ( (sr1&I2C_TRANSFER_EV7) == I2C_TRANSFER_EV7)
    {
        bus->trans.buf[0] = i2c->DR;
        bus->trans.transferred++;
        transfer_complete(bus);
    }
}

#define I2C_MASTER_TRANSMITTER_EV     (I2C_TRANSFER_EV6|I2C_TRANSFER_EV8_1)
void I2C_EV_TxInt_Handler(struct rt_i2c_bus_device* bus)
{
    u16 sr1;
    rt_uint8_t addr = bus->trans.slave_addr;
    I2C_TypeDef* i2c = bus->i2c;
    addr &= ~I2C_ADDR_BIT_RX;

    sr1 = i2c->SR1;
    if ( (sr1&I2C_TRANSFER_EV5) == I2C_TRANSFER_EV5)
    {
        i2c->DR = addr;
    }
    else if ( (sr1&I2C_TRANSFER_EV6) == I2C_TRANSFER_EV6)
    {
        u16 sr2;
        sr2 = i2c->SR2;
    }
    else if ( (sr1&I2C_TRANSFER_EV8_2) == I2C_TRANSFER_EV8_2)
    {
        i2c->CR1 |= I2C_CR1_BIT_STOP;
        transfer_complete(bus);
    }
    else if ( (sr1&I2C_TRANSFER_EV8_1) == I2C_TRANSFER_EV8_1)
    {
        if (bus->trans.transferred == 0)
        {
            i2c->DR = bus->trans.buf[0];
            bus->trans.transferred++;
        }
    }
}

#if defined(RT_USING_I2C1)
void I2C1_EV_IRQHandler(void)
{
    struct rt_i2c_bus_device *bus = (struct rt_i2c_bus_device *)&i2c_bus1;
    
    if (bus->trans.len > 1)
    {
        I2C_EV_DMA_Handler(bus);
    }
    else if (bus->trans.mode& I2C_TRANS_FLAG_TX)
    {
        //I2C_EV_DMA_Handler(bus);
        I2C_EV_TxInt_Handler(bus);
    }
    else
    {
        I2C_EV_RxInt_Handler(bus);
    }
}

void I2C1_ER_IRQHandler(void)
{
    struct rt_i2c_bus_device *bus = (struct rt_i2c_bus_device *)&i2c_bus1;
   
    if(bus->i2c->SR1 & I2C_SR1_BIT_AF)
    {
        bus->i2c->SR1 &= ~I2C_SR1_BIT_AF;
        bus->error = -RT_EI2C;
    }
    if(bus->i2c->SR1 & I2C_SR1_BIT_TIMEOUT)
    {
        bus->i2c->SR1 &= ~I2C_SR1_BIT_TIMEOUT;
        bus->error = -RT_EI2C;
    }
    if(bus->i2c->SR1 & I2C_SR1_BIT_OVR)
    {
        bus->i2c->SR1 &= ~I2C_SR1_BIT_OVR;
        bus->error = -RT_EI2C;
    }
    if(bus->i2c->SR1 & I2C_SR1_BIT_ARLO)
    {
        bus->i2c->SR1 &= ~I2C_SR1_BIT_ARLO;
        bus->error = -RT_EI2C;
    }
    if(bus->i2c->SR1 & I2C_SR1_BIT_BERR)
    {
        bus->i2c->SR1 &= ~I2C_SR1_BIT_BERR;
        bus->error = -RT_EI2C;
    }
    if(bus->i2c->SR1 & I2C_SR1_BIT_PECERR)
    {
        bus->i2c->SR1 &= ~I2C_SR1_BIT_PECERR;
        bus->error = -RT_EI2C;
    }
    if (bus->error != RT_EOK)
    {
        if (bus->dma_stream)
            DMA_Cmd(bus->dma_stream, DISABLE);
        bus->i2c->CR1 |= I2C_CR1_BIT_STOP;
        bus->i2c->CR2 &= ~I2C_CR2_BIT_DMAEN;

        transfer_complete(bus);
    }
}

/* I2C1 TX */
void I2C1_TX_DMA_IRQHANDLER()
{
    struct rt_i2c_bus_device *bus = (struct rt_i2c_bus_device *)&i2c_bus1;

    if(DMA_GetFlagStatus(I2C1_TX_DMA_STREAM, I2C1_TX_DMA_FLAG_TCIF))
    {
        DMA_Cmd(I2C1_TX_DMA_STREAM, DISABLE);
        DMA_ClearFlag(I2C1_TX_DMA_STREAM, I2C1_TX_DMA_FLAG_TCIF);
    }
    if(DMA_GetFlagStatus(I2C1_TX_DMA_STREAM, I2C1_TX_DMA_FLAG_HTIF))
    {
        DMA_ClearFlag(I2C1_TX_DMA_STREAM, I2C1_TX_DMA_FLAG_HTIF);
    }
    if (DMA_GetFlagStatus(I2C1_TX_DMA_STREAM, I2C1_TX_DMA_FLAG_TEIF))
    {
        DMA_ClearFlag(I2C1_TX_DMA_STREAM, I2C1_TX_DMA_FLAG_TEIF);
        DMA_Cmd(bus->dma_stream, DISABLE);
        bus->i2c->CR1 |= I2C_CR1_BIT_STOP;
        bus->i2c->CR2 &= ~I2C_CR2_BIT_DMAEN;
    }
    if(DMA_GetFlagStatus(I2C1_TX_DMA_STREAM, I2C1_TX_DMA_FLAG_FEIF))
    {
        DMA_ClearFlag(I2C1_TX_DMA_STREAM, I2C1_TX_DMA_FLAG_FEIF);
        DMA_Cmd(bus->dma_stream, DISABLE);
        bus->i2c->CR1 |= I2C_CR1_BIT_STOP;
        bus->i2c->CR2 &= ~I2C_CR2_BIT_DMAEN;
    }
    if(DMA_GetFlagStatus(I2C1_TX_DMA_STREAM, I2C1_TX_DMA_FLAG_DMEIF))
    {
        DMA_ClearFlag(I2C1_TX_DMA_STREAM, I2C1_TX_DMA_FLAG_DMEIF);
        DMA_Cmd(bus->dma_stream, DISABLE);
        bus->i2c->CR1 |= I2C_CR1_BIT_STOP;
        bus->i2c->CR2 &= ~I2C_CR2_BIT_DMAEN;
    }
}

/* I2C1 RX */
void I2C1_RX_DMA_IRQHANDLER()
{
    struct rt_i2c_bus_device *bus = (struct rt_i2c_bus_device *)&i2c_bus1;

    if(DMA_GetFlagStatus(I2C1_RX_DMA_STREAM, I2C1_RX_DMA_FLAG_TCIF))
    {
        bus->trans.transferred = bus->trans.len;
        DMA_Cmd(I2C1_RX_DMA_STREAM, DISABLE);
        DMA_ClearFlag(I2C1_RX_DMA_STREAM, I2C1_RX_DMA_FLAG_TCIF);
        bus->i2c->CR1 |= I2C_CR1_BIT_STOP;
        bus->i2c->CR2 &= ~I2C_CR2_BIT_DMAEN;

        transfer_complete(bus);
    }
    if(DMA_GetFlagStatus(I2C1_RX_DMA_STREAM, I2C1_RX_DMA_FLAG_HTIF))
    {
        DMA_ClearFlag(I2C1_RX_DMA_STREAM, I2C1_RX_DMA_FLAG_HTIF);
    }
    if (DMA_GetFlagStatus(I2C1_RX_DMA_STREAM, I2C1_RX_DMA_FLAG_TEIF))
    {
        DMA_ClearFlag(I2C1_RX_DMA_STREAM, I2C1_RX_DMA_FLAG_TEIF);
        DMA_Cmd(bus->dma_stream, DISABLE);
        bus->i2c->CR1 |= I2C_CR1_BIT_STOP;
        bus->i2c->CR2 &= ~I2C_CR2_BIT_DMAEN;
    }
    if(DMA_GetFlagStatus(I2C1_RX_DMA_STREAM, I2C1_RX_DMA_FLAG_FEIF))
    {
        DMA_ClearFlag(I2C1_RX_DMA_STREAM, I2C1_RX_DMA_FLAG_FEIF);
        DMA_Cmd(bus->dma_stream, DISABLE);
        bus->i2c->CR1 |= I2C_CR1_BIT_STOP;
        bus->i2c->CR2 &= ~I2C_CR2_BIT_DMAEN;
    }
    if(DMA_GetFlagStatus(I2C1_RX_DMA_STREAM, I2C1_RX_DMA_FLAG_DMEIF))
    {
        DMA_ClearFlag(I2C1_RX_DMA_STREAM, I2C1_RX_DMA_FLAG_DMEIF);
        DMA_Cmd(bus->dma_stream, DISABLE);
        bus->i2c->CR1 |= I2C_CR1_BIT_STOP;
        bus->i2c->CR2 &= ~I2C_CR2_BIT_DMAEN;
    }
}
#endif

#if defined(RT_USING_I2C2)
void I2C2_EV_IRQHandler(void)
{
    struct rt_i2c_bus_device *bus = (struct rt_i2c_bus_device *)&i2c_bus2;
    
    if (bus->trans.len > 1)
    {
        I2C_EV_DMA_Handler(bus);
    }
    else if (bus->trans.mode& I2C_TRANS_FLAG_TX)
    {
        I2C_EV_TxInt_Handler(bus);
    }
    else
    {
        I2C_EV_RxInt_Handler(bus);
    }
}
void I2C2_ER_IRQHandler(void)
{
    struct rt_i2c_bus_device *bus = (struct rt_i2c_bus_device *)&i2c_bus2;
    
    if(bus->i2c->SR1 & I2C_SR1_BIT_AF)
    {
        bus->i2c->SR1 &= ~I2C_SR1_BIT_AF;
        bus->error = -RT_EI2C;
    }
    if(bus->i2c->SR1 & I2C_SR1_BIT_TIMEOUT)
    {
        bus->i2c->SR1 &= ~I2C_SR1_BIT_TIMEOUT;
        bus->error = -RT_EI2C;
    }
    if(bus->i2c->SR1 & I2C_SR1_BIT_OVR)
    {
        bus->i2c->SR1 &= ~I2C_SR1_BIT_OVR;
        bus->error = -RT_EI2C;
    }
    if(bus->i2c->SR1 & I2C_SR1_BIT_ARLO)
    {
        bus->i2c->SR1 &= ~I2C_SR1_BIT_ARLO;
        bus->error = -RT_EI2C;
    }
    if(bus->i2c->SR1 & I2C_SR1_BIT_BERR)
    {
        bus->i2c->SR1 &= ~I2C_SR1_BIT_BERR;
        bus->error = -RT_EI2C;
    }
    if(bus->i2c->SR1 & I2C_SR1_BIT_PECERR)
    {
        bus->i2c->SR1 &= ~I2C_SR1_BIT_PECERR;
        bus->error = -RT_EI2C;
    }
    if (bus->error != RT_EOK)
    {
        if (bus->dma_stream)
            DMA_Cmd(bus->dma_stream, DISABLE);
        bus->i2c->CR1 |= I2C_CR1_BIT_STOP;
        bus->i2c->CR2 &= ~I2C_CR2_BIT_DMAEN;

        transfer_complete(bus);
    }

}
void I2C2_TX_DMA_IRQHANDLER()
{
    struct rt_i2c_bus_device *bus = (struct rt_i2c_bus_device *)&i2c_bus2;
    
    if(DMA_GetFlagStatus(I2C2_TX_DMA_STREAM, I2C2_TX_DMA_FLAG_TCIF))
    {
        DMA_Cmd(I2C2_TX_DMA_STREAM, DISABLE);
        DMA_ClearFlag(I2C2_TX_DMA_STREAM, I2C2_TX_DMA_FLAG_TCIF);
    }
    if(DMA_GetFlagStatus(I2C2_TX_DMA_STREAM, I2C2_TX_DMA_FLAG_HTIF))
    {
        DMA_ClearFlag(I2C2_TX_DMA_STREAM, I2C2_TX_DMA_FLAG_HTIF);
    }
    if (DMA_GetFlagStatus(I2C2_TX_DMA_STREAM, I2C2_TX_DMA_FLAG_TEIF))
    {
        DMA_ClearFlag(I2C2_TX_DMA_STREAM, I2C2_TX_DMA_FLAG_TEIF);
    }
    if(DMA_GetFlagStatus(I2C2_TX_DMA_STREAM, I2C2_TX_DMA_FLAG_FEIF))
    {
        DMA_ClearFlag(I2C2_TX_DMA_STREAM, I2C2_TX_DMA_FLAG_FEIF);
    }
    if(DMA_GetFlagStatus(I2C2_TX_DMA_STREAM, I2C2_TX_DMA_FLAG_DMEIF))
    {
        DMA_ClearFlag(I2C2_TX_DMA_STREAM, I2C2_TX_DMA_FLAG_DMEIF);
    }
}
void I2C2_RX_DMA_IRQHANDLER()
{
    struct rt_i2c_bus_device *bus = (struct rt_i2c_bus_device *)&i2c_bus2;

    if(DMA_GetFlagStatus(I2C2_RX_DMA_STREAM, I2C2_RX_DMA_FLAG_TCIF))
    {
        bus->trans.transferred = bus->trans.len;
        DMA_Cmd(I2C2_RX_DMA_STREAM, DISABLE);
        DMA_ClearFlag(I2C2_RX_DMA_STREAM, I2C2_RX_DMA_FLAG_TCIF);
        bus->i2c->CR1 |= I2C_CR1_BIT_STOP;
        bus->i2c->CR2 &= ~I2C_CR2_BIT_DMAEN;

        transfer_complete(bus);
    }
    if(DMA_GetFlagStatus(I2C2_RX_DMA_STREAM, I2C2_RX_DMA_FLAG_HTIF))
    {
        DMA_ClearFlag(I2C2_RX_DMA_STREAM, I2C2_RX_DMA_FLAG_HTIF);
    }
    if (DMA_GetFlagStatus(I2C2_RX_DMA_STREAM, I2C2_RX_DMA_FLAG_TEIF))
    {
        DMA_ClearFlag(I2C2_RX_DMA_STREAM, I2C2_RX_DMA_FLAG_TEIF);
    }
    if(DMA_GetFlagStatus(I2C2_RX_DMA_STREAM, I2C2_RX_DMA_FLAG_FEIF))
    {
        DMA_ClearFlag(I2C2_RX_DMA_STREAM, I2C2_RX_DMA_FLAG_FEIF);
    }
    if(DMA_GetFlagStatus(I2C2_RX_DMA_STREAM, I2C2_RX_DMA_FLAG_DMEIF))
    {
        DMA_ClearFlag(I2C2_RX_DMA_STREAM, I2C2_RX_DMA_FLAG_DMEIF);
    }
}
#endif

static void RCC_Configuration()
{
#if defined(RT_USING_I2C1)
    RCC->AHB1ENR |= RCC_AHB1Periph_GPIOB;
    RCC->APB1ENR |= RCC_APB1Periph_I2C1;
#endif

#if defined(RT_USING_I2C2)
    RCC->AHB1ENR |= RCC_AHB1Periph_GPIOB;
    RCC->APB1ENR |= RCC_APB1Periph_I2C2;
#endif

    RCC->AHB1ENR |= RCC_AHB1Periph_DMA1;
}

static void GPIO_Configuration()
{
    GPIO_InitTypeDef GPIO_InitStructure;

#if defined(RT_USING_I2C1)
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Pin   = I2C1_GPIO_SCL|I2C1_GPIO_SDA;
    GPIO_Init(I2C1_GPIO, &GPIO_InitStructure);

    GPIO_PinAFConfig(I2C1_GPIO, I2C1_GPIO_SCL_SRC, GPIO_AF_I2C1);
    GPIO_PinAFConfig(I2C1_GPIO, I2C1_GPIO_SDA_SRC, GPIO_AF_I2C1);
#endif

#if defined(RT_USING_I2C2)
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Pin   = I2C2_GPIO_SCL|I2C2_GPIO_SDA;
    GPIO_Init(I2C2_GPIO, &GPIO_InitStructure);

    GPIO_PinAFConfig(I2C2_GPIO, I2C2_GPIO_SCL_SRC, GPIO_AF_I2C2);
    GPIO_PinAFConfig(I2C2_GPIO, I2C2_GPIO_SDA_SRC, GPIO_AF_I2C2);
#endif
}

static void NVIC_Configuration()
{
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

#if defined(RT_USING_I2C1)
    NVIC_InitStructure.NVIC_IRQChannel = I2C1_EV_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = I2C1_ER_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = I2C1_TX_DMA_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = I2C1_RX_DMA_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#endif

#if defined(RT_USING_I2C2)
    NVIC_InitStructure.NVIC_IRQChannel = I2C2_EV_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = I2C2_ER_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = I2C2_TX_DMA_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = I2C2_RX_DMA_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#endif
}

/*
 * i2c register
 */

rt_err_t rt_hw_i2c_init(rt_uint32_t mode, 
                        struct i2c_configuration* config)
{
    rt_device_t device;
    rt_err_t err;
    struct rt_i2c_bus_device *bus = RT_NULL;

    RCC_Configuration();
    GPIO_Configuration();
    NVIC_Configuration();

#if defined(RT_USING_I2C1)
    bus       = &i2c_bus1;
    bus->i2c  = I2C1;
    bus->mode = mode;

    /* initialize semaphore */
#ifdef I2CX_SYNC_SEMAPHORE
    err = rt_sem_init(&bus->sem, "i2c1_sem", 0, RT_IPC_FLAG_FIFO);
    if (err != RT_EOK)
    {
        return err;
    }
#else
    rt_completion_init(&bus->completion);
#endif

    /* receive configuration */
    if (config)
    {
        bus->config.master_addr = config->master_addr;
        bus->config.speed       = config->speed;
        bus->config.timeout     = config->timeout;
    }
    else
    {
        return -RT_ERROR;
    }

    bus->dma_tx_conf = I2C1_LowLevel_DMA_TxConfig;
    bus->dma_rx_conf = I2C1_LowLevel_DMA_RxConfig;
    bus->dma_tx_stream = I2C1_TX_DMA_STREAM;
    bus->dma_rx_stream = I2C1_RX_DMA_STREAM;
    bus->dma_stream = RT_NULL;

    /* reset transaction */
    memset(&bus->trans, 0, sizeof(struct rt_i2c_trans));

    /* initialize device interface */
    device = (rt_device_t)&i2c_bus1;

    device->type        = RT_Device_Class_I2CBUS;
    device->rx_indicate = RT_NULL;
    device->tx_complete = RT_NULL;
    device->user_data   = RT_NULL;
    
    device->init    = i2c_bus_device_init;
    device->open    = i2c_bus_device_open;
    device->close   = i2c_bus_device_close;
    device->read    = i2c_bus_device_read_sync;
    device->write   = i2c_bus_device_write_sync;
    device->control = i2c_bus_device_control;

    /* create timer */
    rt_timer_init(&bus->tim, I2C1_TIM_NAME, i2c_transfer_timeout, bus, bus->config.timeout, RT_TIMER_FLAG_ONE_SHOT);

    /* register a character device */
    err = rt_device_register((rt_device_t)bus, I2C1_BUS_NAME, RT_DEVICE_FLAG_RDWR);
    if (err != RT_EOK)
    {
        return err;
    }
#endif

#if defined(RT_USING_I2C2)
    bus       = &i2c_bus2;
    bus->i2c  = I2C2;
    bus->mode = mode;

    /* initialize semaphore */
#ifdef I2CX_SYNC_SEMAPHORE
    err = rt_sem_init(&bus->sem, "i2c2_sem", 0, RT_IPC_FLAG_FIFO);
    if (err != RT_EOK)
    {
        return err;
    }
#else
    rt_completion_init(&bus->completion);
#endif

    /* receive configuration */
    if (config)
    {
        bus->config.master_addr = config->master_addr;
        bus->config.speed       = config->speed;
        bus->config.timeout     = config->timeout;
    }
    else
    {
        return -RT_ERROR;
    }

    bus->dma_tx_conf = I2C2_LowLevel_DMA_TxConfig;
    bus->dma_rx_conf = I2C2_LowLevel_DMA_RxConfig;

    /* reset transaction */
    memset(&bus->trans, 0, sizeof(struct rt_i2c_trans));

    /* initialize device interface */
    device = (rt_device_t)&i2c_bus2;

    device->type        = RT_Device_Class_I2CBUS;
    device->rx_indicate = RT_NULL;
    device->tx_complete = RT_NULL;
    device->user_data   = RT_NULL;
    
    device->init    = i2c_bus_device_init;
    device->open    = i2c_bus_device_open;
    device->close   = i2c_bus_device_close;
    device->read    = i2c_bus_device_read_sync;
    device->write   = i2c_bus_device_write_sync;
    device->control = i2c_bus_device_control;

    /* create timer */
    rt_timer_init(&bus->tim, I2C2_TIM_NAME, i2c_transfer_timeout, bus, bus->timeout, RT_TIMER_FLAG_ONE_SHOT);

    /* register a character device */
    err = rt_device_register((rt_device_t)bus, I2C2_BUS_NAME, RT_DEVICE_FLAG_RDWR);
    if (err != RT_EOK)
    {
        return err;
    }
#endif
    return RT_EOK;
}

#endif /*RT_USING_I2CX*/

