/**
 ******************************************************************************
 * @file    platform_config.h
 * @author  Satish Nair, Zachary Crockett and Mohit Bhoite
 * @version V1.0.0
 * @date    13-March-2013
 * @brief   Board specific configuration file.
 ******************************************************************************
    Copyright (c) 2013 Spark Labs, Inc.  All rights reserved.

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation, either
    version 3 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, see <http://www.gnu.org/licenses/>.
    ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PLATFORM_CONFIG_H
#define __PLATFORM_CONFIG_H

/* Uncomment the line below to enable WLAN, WIRING, SFLASH and RTC functionality */
#define SPARK_WLAN_ENABLE
#define SPARK_WIRING_ENABLE
#define SPARK_SFLASH_ENABLE
#define SPARK_RTC_ENABLE

#define         ID1          (0x1FFFF7E8)
#define         ID2          (0x1FFFF7EC)
#define         ID3          (0x1FFFF7F0)

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

//LEDs (STM32F4 discovery is PD12-15)
#define LEDn                              4
#define LEDGREEN_GPIO_PIN                 GPIO_Pin_12
#define LEDGREEN_GPIO_PORT                GPIOD
#define LEDGREEN_GPIO_CLK                 RCC_AHB1Periph_GPIOD
#define LEDORANGE_GPIO_PIN                GPIO_Pin_13
#define LEDORANGE_GPIO_PORT               GPIOD
#define LEDORANGE_GPIO_CLK                RCC_AHB1Periph_GPIOD
#define LEDRED_GPIO_PIN                   GPIO_Pin_14
#define LEDRED_GPIO_PORT                  GPIOD
#define LEDRED_GPIO_CLK                   RCC_AHB1Periph_GPIOD
#define LEDBLUE_GPIO_PIN                  GPIO_Pin_15
#define LEDBLUE_GPIO_PORT                 GPIOD
#define LEDBLUE_GPIO_CLK                  RCC_AHB1Periph_GPIOD

//Push Buttons (User button is PA0 on STM32F4 discovery)
#define BUTTONn                           1
#define BUTTON1_GPIO_PIN                  GPIO_Pin_0
#define BUTTON1_GPIO_PORT                 GPIOA
#define BUTTON1_GPIO_CLK                  RCC_AHB1Periph_GPIOA
#define BUTTON1_GPIO_MODE                 GPIO_Mode_IN
#define BUTTON1_GPIO_PUPD                 GPIO_PuPd_DOWN
#define BUTTON1_PRESSED                   0x01
#define BUTTON1_EXTI_LINE                 EXTI_Line0
#define BUTTON1_EXTI_PORT_SOURCE          EXTI_PortSourceGPIOA
#define BUTTON1_EXTI_PIN_SOURCE           GPIO_PinSource0
#define BUTTON1_EXTI_IRQn                 EXTI0_IRQn
#define BUTTON1_EXTI_TRIGGER              EXTI_Trigger_Falling
#define BUTTON2_GPIO_PIN                  0
#define BUTTON2_GPIO_PORT                 0
#define BUTTON2_GPIO_CLK                  0
#define BUTTON2_GPIO_MODE                 0
#define BUTTON2_GPIO_PUPD                 0
#define BUTTON2_PRESSED                   0
#define BUTTON2_EXTI_LINE                 0
#define BUTTON2_EXTI_PORT_SOURCE          0
#define BUTTON2_EXTI_PIN_SOURCE           0
#define BUTTON2_EXTI_IRQn                 0
#define BUTTON2_EXTI_TRIGGER              0

//Header IOs
#define Dn                                8
#define D0_GPIO_PIN                       GPIO_Pin_7
#define D0_GPIO_PORT                      GPIOE
#define D0_GPIO_CLK                       RCC_AHB1Periph_GPIOE
#define D1_GPIO_PIN                       GPIO_Pin_8
#define D1_GPIO_PORT                      GPIOE
#define D1_GPIO_CLK                       RCC_AHB1Periph_GPIOE
#define D2_GPIO_PIN                       GPIO_Pin_9
#define D2_GPIO_PORT                      GPIOE
#define D2_GPIO_CLK                       RCC_AHB1Periph_GPIOE
#define D3_GPIO_PIN                       GPIO_Pin_10
#define D3_GPIO_PORT                      GPIOE
#define D3_GPIO_CLK                       RCC_AHB1Periph_GPIOE
#define D4_GPIO_PIN                       GPIO_Pin_11
#define D4_GPIO_PORT                      GPIOE
#define D4_GPIO_CLK                       RCC_AHB1Periph_GPIOE
#define D5_GPIO_PIN                       GPIO_Pin_12
#define D5_GPIO_PORT                      GPIOE
#define D5_GPIO_CLK                       RCC_AHB1Periph_GPIOE
#define D6_GPIO_PIN                       GPIO_Pin_13
#define D6_GPIO_PORT                      GPIOE
#define D6_GPIO_CLK                       RCC_AHB1Periph_GPIOE
#define D7_GPIO_PIN                       GPIO_Pin_14
#define D7_GPIO_PORT                      GPIOE
#define D7_GPIO_CLK                       RCC_AHB1Periph_GPIOE

//CC3000 Interface pins
#define CC3000_SPI                        SPI2
#define CC3000_SPI_CLK                    RCC_APB1Periph_SPI2
#define CC3000_SPI_CLK_CMD                RCC_APB1PeriphClockCmd
#define CC3000_SPI_SCK_GPIO_PIN           GPIO_Pin_13                    /* PB.13 */
#define CC3000_SPI_SCK_GPIO_PORT          GPIOB                        /* GPIOB */
#define CC3000_SPI_SCK_GPIO_CLK           RCC_AHB1Periph_GPIOB
#define CC3000_SPI_MISO_GPIO_PIN          GPIO_Pin_14                    /* PB.14 */
#define CC3000_SPI_MISO_GPIO_PORT         GPIOB                        /* GPIOB */
#define CC3000_SPI_MISO_GPIO_CLK          RCC_AHB1Periph_GPIOB
#define CC3000_SPI_MOSI_GPIO_PIN          GPIO_Pin_15                    /* PB.15 */
#define CC3000_SPI_MOSI_GPIO_PORT         GPIOB                        /* GPIOB */
#define CC3000_SPI_MOSI_GPIO_CLK          RCC_AHB1Periph_GPIOB
#define CC3000_WIFI_CS_GPIO_PIN           GPIO_Pin_12                    /* PB.12 */
#define CC3000_WIFI_CS_GPIO_PORT          GPIOB                        /* GPIOB */
#define CC3000_WIFI_CS_GPIO_CLK           RCC_AHB1Periph_GPIOB
#define CC3000_WIFI_EN_GPIO_PIN           GPIO_Pin_8                    /* PB.08 */
#define CC3000_WIFI_EN_GPIO_PORT          GPIOB                        /* GPIOB */
#define CC3000_WIFI_EN_GPIO_CLK           RCC_AHB1Periph_GPIOB
#define CC3000_WIFI_INT_GPIO_PIN          GPIO_Pin_11                    /* PB.11 */
#define CC3000_WIFI_INT_GPIO_PORT         GPIOB                        /* GPIOB */
#define CC3000_WIFI_INT_GPIO_CLK          RCC_AHB1Periph_GPIOB

#define CC3000_WIFI_INT_EXTI_LINE         EXTI_Line11
#define CC3000_WIFI_INT_EXTI_PORT_SOURCE  EXTI_PortSourceGPIOB
#define CC3000_WIFI_INT_EXTI_PIN_SOURCE   GPIO_PinSource11
#define CC3000_WIFI_INT_EXTI_IRQn         EXTI15_10_IRQn

#define CC3000_SPI_DMA_CLK                RCC_AHB1Periph_DMA1
#define CC3000_SPI_RX_DMA_CHANNEL         DMA_Channel_0
#define CC3000_SPI_RX_DMA_STREAM          DMA1_Stream3
#define CC3000_SPI_TX_DMA_CHANNEL         DMA_Channel_0
#define CC3000_SPI_TX_DMA_STREAM          DMA1_Stream4
#define CC3000_SPI_RX_DMA_TCFLAG          DMA_FLAG_TCIF3
#define CC3000_SPI_TX_DMA_TCFLAG          DMA_FLAG_TCIF4
#define CC3000_SPI_RX_DMA_IRQn            DMA1_Stream3_IRQn
#define CC3000_SPI_TX_DMA_IRQn            DMA1_Stream4_IRQn

#define CC3000_SPI_DR_BASE                ((uint32_t)0x40003800)    /* SPI2_BASE | 0x0C */

#define CC3000_SPI_BAUDRATE_PRESCALER     SPI_BaudRatePrescaler_4

//SST25 FLASH Interface pins
#define sFLASH_SPI                        SPI2
#define sFLASH_SPI_CLK                    RCC_APB1Periph_SPI2
#define sFLASH_SPI_CLK_CMD                RCC_APB1PeriphClockCmd
#define sFLASH_SPI_SCK_GPIO_PIN           GPIO_Pin_13                    /* PB.13 */
#define sFLASH_SPI_SCK_GPIO_PIN_SOURCE    GPIO_PinSource13
#define sFLASH_SPI_SCK_GPIO_PORT          GPIOB                        /* GPIOB */
#define sFLASH_SPI_SCK_GPIO_CLK           RCC_AHB1Periph_GPIOB
#define sFLASH_SPI_MISO_GPIO_PIN          GPIO_Pin_14                    /* PB.14 */
#define sFLASH_SPI_MISO_GPIO_PIN_SOURCE   GPIO_PinSource14
#define sFLASH_SPI_MISO_GPIO_PORT         GPIOB                        /* GPIOB */
#define sFLASH_SPI_MISO_GPIO_CLK          RCC_AHB1Periph_GPIOB
#define sFLASH_SPI_MOSI_GPIO_PIN          GPIO_Pin_15                    /* PB.15 */
#define sFLASH_SPI_MOSI_GPIO_PIN_SOURCE   GPIO_PinSource15
#define sFLASH_SPI_MOSI_GPIO_PORT         GPIOB                        /* GPIOB */
#define sFLASH_SPI_MOSI_GPIO_CLK          RCC_AHB1Periph_GPIOB
#define sFLASH_MEM_CS_GPIO_PIN            GPIO_Pin_9                    /* PB.09 */
#define sFLASH_MEM_CS_GPIO_PORT           GPIOB                        /* GPIOB */
#define sFLASH_MEM_CS_GPIO_CLK            RCC_AHB1Periph_GPIOB

#define sFLASH_SPI_BAUDRATE_PRESCALER     SPI_BaudRatePrescaler_4

#define USB_DISCONNECT_GPIO_PIN           GPIO_Pin_12
#define USB_DISCONNECT_GPIO_PORT          GPIOA
#define USB_DISCONNECT_GPIO_CLK           RCC_AHB1Periph_GPIOA

#define UI_TIMER_FREQUENCY                100                            /* 100Hz -> 10ms */
#define BUTTON_DEBOUNCE_INTERVAL          1000 / UI_TIMER_FREQUENCY

//NVIC Priorities based on NVIC_PriorityGroup_4
#define DMA1_STREAM4_IRQ_PRIORITY         0    //CC3000_SPI_TX_DMA Interrupt
#define EXTI15_10_IRQ_PRIORITY            1    //CC3000_WIFI_INT_EXTI & User Interrupt
#define USB_LP_IRQ_PRIORITY               2    //USB_LP_CAN1_RX0 Interrupt
#define RTCALARM_IRQ_PRIORITY             3    //RTC Alarm Interrupt
#define RTC_IRQ_PRIORITY                  4    //RTC Seconds Interrupt
#define TIM1_CC_IRQ_PRIORITY              5    //TIM1_CC4 Interrupt
#define EXTI2_IRQ_PRIORITY                6    //BUTTON1_EXTI Interrupt
#define USART2_IRQ_PRIORITY               7    //USART2 Interrupt
#define EXTI0_IRQ_PRIORITY                11    //User Interrupt
#define EXTI1_IRQ_PRIORITY                11    //User Interrupt
#define EXTI3_IRQ_PRIORITY                11    //User Interrupt
#define EXTI4_IRQ_PRIORITY                11    //User Interrupt
#define EXTI9_5_IRQ_PRIORITY              12    //User Interrupt
#define SYSTICK_IRQ_PRIORITY              13    //CORTEX_M3 Systick Interrupt
#define SVCALL_IRQ_PRIORITY               14    //CORTEX_M3 SVCall Interrupt
#define PENDSV_IRQ_PRIORITY               15    //CORTEX_M3 PendSV Interrupt

/* Exported functions ------------------------------------------------------- */

#endif /* __PLATFORM_CONFIG_H */
