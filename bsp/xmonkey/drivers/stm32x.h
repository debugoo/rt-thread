/*
 * File      : stm32x.h
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2009, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 05-05-2016     XuJun        created
 */

#ifndef __STM32X_H__
#define __STM32X_H__

#include <rtthread.h>

/* RCC AHB */
#define RCC_AHB1Periph_EN_GPIOA              ((rt_uint32_t)(1<<0))
#define RCC_AHB1Periph_EN_GPIOB              ((rt_uint32_t)(1<<1))
#define RCC_AHB1Periph_EN_GPIOC              ((rt_uint32_t)(1<<2))
#define RCC_AHB1Periph_EN_GPIOD              ((rt_uint32_t)(1<<3))
#define RCC_AHB1Periph_EN_GPIOE              ((rt_uint32_t)(1<<4))
#define RCC_AHB1Periph_EN_GPIOF              ((rt_uint32_t)(1<<5))
#define RCC_AHB1Periph_EN_GPIOG              ((rt_uint32_t)(1<<6))
#define RCC_AHB1Periph_EN_GPIOH              ((rt_uint32_t)(1<<7))
#define RCC_AHB1Periph_EN_GPIOIE             ((rt_uint32_t)(1<<8))
#define RCC_AHB1Periph_EN_CRC                ((rt_uint32_t)(1<<12))
#define RCC_AHB1Periph_EN_BKP_SRAM           ((rt_uint32_t)(1<<18))
#define RCC_AHB1Periph_EN_CCM_DATA_RAM       ((rt_uint32_t)(1<<20))
#define RCC_AHB1Periph_EN_DMA1               ((rt_uint32_t)(1<<21))
#define RCC_AHB1Periph_EN_DMA2               ((rt_uint32_t)(1<<22))
#define RCC_AHB1Periph_EN_ETH_MAC            ((rt_uint32_t)(1<<25))
#define RCC_AHB1Periph_EN_ETH_MAC_TX         ((rt_uint32_t)(1<<26))
#define RCC_AHB1Periph_EN_ETH_MAC_RX         ((rt_uint32_t)(1<<27))
#define RCC_AHB1Periph_EN_ETH_MAC_PTP        ((rt_uint32_t)(1<<28))
#define RCC_AHB1Periph_EN_ETH_OTH_HS         ((rt_uint32_t)(1<<29))
#define RCC_AHB1Periph_EN_ETH_OTH_HS_ULPIE   ((rt_uint32_t)(1<<30))

#define RCC_AHB1Periph_BIT_SRAM               ((rt_uint32_t)(1<<2))
#define RCC_AHB1Periph_BIT_FLITF              ((rt_uint32_t)(1<<4))
#define RCC_AHB1Periph_BIT_CRC                ((rt_uint32_t)(1<<6))
#define RCC_AHB1Periph_BIT_FSMC               ((rt_uint32_t)(1<<8))
#define RCC_AHB1Periph_BIT_SDIO               ((rt_uint32_t)(1<<10))

/* RCC APB1 */
#define RCC_APB1Periph_BIT_TIM2              ((rt_uint32_t)(1<<0))
#define RCC_APB1Periph_BIT_TIM3              ((rt_uint32_t)(1<<1))
#define RCC_APB1Periph_BIT_TIM4              ((rt_uint32_t)(1<<2))
#define RCC_APB1Periph_BIT_TIM5              ((rt_uint32_t)(1<<3))
#define RCC_APB1Periph_BIT_TIM6              ((rt_uint32_t)(1<<4))
#define RCC_APB1Periph_BIT_TIM7              ((rt_uint32_t)(1<<5))
#define RCC_APB1Periph_BIT_TIM12             ((rt_uint32_t)(1<<6))
#define RCC_APB1Periph_BIT_TIM13             ((rt_uint32_t)(1<<7))
#define RCC_APB1Periph_BIT_TIM14             ((rt_uint32_t)(1<<8))
#define RCC_APB1Periph_BIT_WWDG              ((rt_uint32_t)(1<<11))
#define RCC_APB1Periph_BIT_SPI2              ((rt_uint32_t)(1<<14))
#define RCC_APB1Periph_BIT_SPI3              ((rt_uint32_t)(1<<15))
#define RCC_APB1Periph_BIT_USART2            ((rt_uint32_t)(1<<17))
#define RCC_APB1Periph_BIT_USART3            ((rt_uint32_t)(1<<18))
#define RCC_APB1Periph_BIT_UART4             ((rt_uint32_t)(1<<19))
#define RCC_APB1Periph_BIT_UART5             ((rt_uint32_t)(1<<20))
#define RCC_APB1Periph_BIT_I2C1              ((rt_uint32_t)(1<<21))
#define RCC_APB1Periph_BIT_I2C2              ((rt_uint32_t)(1<<22))
#define RCC_APB1Periph_BIT_USB               ((rt_uint32_t)(1<<23))
#define RCC_APB1Periph_BIT_CAN1              ((rt_uint32_t)(1<<25))
#define RCC_APB1Periph_BIT_BKP               ((rt_uint32_t)(1<<27))
#define RCC_APB1Periph_BIT_PWR               ((rt_uint32_t)(1<<28))
#define RCC_APB1Periph_BIT_DAC               ((rt_uint32_t)(1<<29))

/* RCC APB2 */
#define RCC_APB2Periph_BIT_AFIO              ((rt_uint32_t)(1<<0))
#define RCC_APB2Periph_BIT_IOPA              ((rt_uint32_t)(1<<2))
#define RCC_APB2Periph_BIT_IOPB              ((rt_uint32_t)(1<<3))
#define RCC_APB2Periph_BIT_IOPC              ((rt_uint32_t)(1<<4))
#define RCC_APB2Periph_BIT_IOPD              ((rt_uint32_t)(1<<5))
#define RCC_APB2Periph_BIT_IOPE              ((rt_uint32_t)(1<<6))
#define RCC_APB2Periph_BIT_IOPF              ((rt_uint32_t)(1<<7))
#define RCC_APB2Periph_BIT_IOPG              ((rt_uint32_t)(1<<8))
#define RCC_APB2Periph_BIT_ADC1              ((rt_uint32_t)(1<<9))
#define RCC_APB2Periph_BIT_ADC2              ((rt_uint32_t)(1<<10))
#define RCC_APB2Periph_BIT_TIM1              ((rt_uint32_t)(1<<11))
#define RCC_APB2Periph_BIT_SPI1              ((rt_uint32_t)(1<<12))
#define RCC_APB2Periph_BIT_TIM8              ((rt_uint32_t)(1<<13))
#define RCC_APB2Periph_BIT_USART1            ((rt_uint32_t)(1<<14))
#define RCC_APB2Periph_BIT_ADC3              ((rt_uint32_t)(1<<15))
#define RCC_APB2Periph_BIT_TIM9              ((rt_uint32_t)(1<<19))
#define RCC_APB2Periph_BIT_TIM10             ((rt_uint32_t)(1<<20))
#define RCC_APB2Periph_BIT_TIM11             ((rt_uint32_t)(1<<21))


/* I2C SR1 register bits */
#define I2C_SR1_BIT_SB          (1<<0)
#define I2C_SR1_BIT_ADDR        (1<<1)
#define I2C_SR1_BIT_BTF         (1<<2)
#define I2C_SR1_BIT_ADD10       (1<<3)
#define I2C_SR1_BIT_STOPF       (1<<4)
#define I2C_SR1_BIT_RESERVED1   (1<<5)
#define I2C_SR1_BIT_RXNE        (1<<6)
#define I2C_SR1_BIT_TXE         (1<<7)
#define I2C_SR1_BIT_BERR        (1<<8)
#define I2C_SR1_BIT_ARLO        (1<<9)
#define I2C_SR1_BIT_AF          (1<<10)
#define I2C_SR1_BIT_OVR         (1<<11)
#define I2C_SR1_BIT_PECERR      (1<<12)
#define I2C_SR1_BIT_RESERVED2   (1<<13)
#define I2C_SR1_BIT_TIMEOUT     (1<<14)
#define I2C_SR1_BIT_SMBALERT    (1<<15)

/* I2C transfer events */
#define I2C_TRANSFER_EV5             (I2C_SR1_BIT_SB)
#define I2C_TRANSFER_EV6             (I2C_SR1_BIT_ADDR)
/* I2C recieve events */
#define I2C_TRANSFER_EV6_1           (0)
#define I2C_TRANSFER_EV7             (I2C_SR1_BIT_RXNE)
#define I2C_TRANSFER_EV7_1           (I2C_SR1_BIT_RXNE)
/* I2C transmit events */
#define I2C_TRANSFER_EV8_1           (I2C_SR1_BIT_TXE)
#define I2C_TRANSFER_EV8             (I2C_SR1_BIT_TXE)
#define I2C_TRANSFER_EV8_2           (I2C_SR1_BIT_TXE|I2C_SR1_BIT_BTF)

/* I2C SR2 register bits */
#define I2C_SR2_BIT_MSL         (1<<0)
#define I2C_SR2_BIT_BUSY        (1<<1)
#define I2C_SR2_BIT_TRA         (1<<2)
#define I2C_SR2_BIT_RES1        (1<<3)
#define I2C_SR2_BIT_GENCALL     (1<<4)
#define I2C_SR2_BIT_SMBDEFAULT  (1<<5)
#define I2C_SR2_BIT_SMBHOST     (1<<6)
#define I2C_SR2_BIT_DUALF       (1<<7)
#define I2C_SR2_BIT_PEC0        (1<<8)
#define I2C_SR2_BIT_PEC1        (1<<9)
#define I2C_SR2_BIT_PEC2        (1<<10)
#define I2C_SR2_BIT_PEC3        (1<<11)
#define I2C_SR2_BIT_PEC4        (1<<12)
#define I2C_SR2_BIT_PEC5        (1<<13)
#define I2C_SR2_BIT_PEC6        (1<<14)
#define I2C_SR2_BIT_PEC7        (1<<15)

/* I2C CR1 register bits */
#define I2C_CR1_BIT_PE          (1<<0)
#define I2C_CR1_BIT_SMBUS       (1<<1)
#define I2C_CR1_BIT_RES1        (1<<2)
#define I2C_CR1_BIT_SMBTYPE     (1<<3)
#define I2C_CR1_BIT_ENARP       (1<<4)
#define I2C_CR1_BIT_ENPEC       (1<<5)
#define I2C_CR1_BIT_ENGC        (1<<6)
#define I2C_CR1_BIT_NO_STRETCH  (1<<7)
#define I2C_CR1_BIT_START       (1<<8)
#define I2C_CR1_BIT_STOP        (1<<9)
#define I2C_CR1_BIT_ACK         (1<<10)
#define I2C_CR1_BIT_POS         (1<<11)
#define I2C_CR1_BIT_PEC         (1<<12)
#define I2C_CR1_BIT_ALERT       (1<<13)
#define I2C_CR1_BIT_RES2        (1<<14)
#define I2C_CR1_BIT_SWRST       (1<<15)

/* I2C CR2 register bits */
#define I2C_CR2_BIT_FREQ0       (1<<0)
#define I2C_CR2_BIT_FREQ1       (1<<1)
#define I2C_CR2_BIT_FREQ2       (1<<2)
#define I2C_CR2_BIT_FREQ3       (1<<3)
#define I2C_CR2_BIT_FREQ4       (1<<4)
#define I2C_CR2_BIT_FREQ5       (1<<5)
#define I2C_CR2_BIT_RES1        (1<<6)
#define I2C_CR2_BIT_RES2        (1<<7)
#define I2C_CR2_BIT_ITERREN     (1<<8)
#define I2C_CR2_BIT_ITEVTEN     (1<<9)
#define I2C_CR2_BIT_ITBUFEN     (1<<10)
#define I2C_CR2_BIT_DMAEN       (1<<11)
#define I2C_CR2_BIT_LAST        (1<<12)
#define I2C_CR2_BIT_RES3        (1<<13)
#define I2C_CR2_BIT_RES4        (1<<14)
#define I2C_CR2_BIT_RES5        (1<<15)


#endif
