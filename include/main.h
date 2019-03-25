/**
  ******************************************************************************
  * @file    main.h 
  * @author	Luigi Pannocchi 
  * @brief   Header for main.c module
  ******************************************************************************
*/

#ifndef __MAIN_H
#define __MAIN_H

#include "stm32f4xx_hal.h"

#define INT_MPU_PIN 			GPIO_PIN_4

#define USARTx                           USART2
#define USARTx_BAUD			 (115200)
#define USARTx_CLK_ENABLE()              __HAL_RCC_USART2_CLK_ENABLE();
#define USARTx_RX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
#define USARTx_TX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE() 

#define USARTx_FORCE_RESET()             __HAL_RCC_USART2_FORCE_RESET()
#define USARTx_RELEASE_RESET()           __HAL_RCC_USART2_RELEASE_RESET()

/* Definition for USARTx Pins */
#define USARTx_TX_PIN                    GPIO_PIN_2
#define USARTx_TX_GPIO_PORT              GPIOA  
#define USARTx_TX_AF                     GPIO_AF7_USART2
#define USARTx_RX_PIN                    GPIO_PIN_3
#define USARTx_RX_GPIO_PORT              GPIOA 
#define USARTx_RX_AF                     GPIO_AF7_USART2

/* Definition for USARTx's NVIC */
#define USARTx_IRQn                      USART2_IRQn
#define USARTx_IRQHandler                USART2_IRQHandler

#define I2C_ADDRESS	0x30F

/* Size of Transmission buffer */
#define TXBUFFERSIZE                      (255)
/* Size of Reception buffer */
//#define RXBUFFERSIZE                      TXBUFFERSIZE

#endif
