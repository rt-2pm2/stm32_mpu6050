/**
 ******************************************************************************
 * @file    stm32f4xx_hal_msp.c
 * @authors MCD Application Team, Luigi Pannocchi
 * @brief   This file contains the HAL System and Peripheral (PPP) MSP initialization
 *          and de-initialization functions.
 *          It should be copied to the application folder and renamed into 'stm32f4xx_hal_msp.c'.           
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "main.h"

/** @addtogroup STM32F4xx_HAL_Driver
 * @{
 */

/** @defgroup HAL_MSP HAL MSP
 * @brief HAL MSP module.
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @defgroup HAL_MSP_Private_Functions HAL MSP Private Functions
 * @{
 */

/**
 * @brief  Initializes the I2C MSP.
 * @param I2C handle pointer
 * @retval None
 */
void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c) {
	/*
	 * (#)
	 * 	(##) Enable the i2c peripheral clock
	 */
	__HAL_RCC_I2C3_CLK_ENABLE();
	__HAL_RCC_I2C3_FORCE_RESET();
	__HAL_RCC_I2C3_RELEASE_RESET();

	/*
	 *	(##) I2C pins configuration
	 */ 
	GPIO_InitTypeDef  GPIO_InitStruct;

	/*	
	 * 		(+++) Enable the clock for the I2C GPIOs 
	 * 			(I2C3 is on port A and C)
	 */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	/*
	 * 		(+++) Configure I2C pins as alternate function open-drain
	 * 			I2C TX (SCL) GPIO pin configuration  
	 */
	GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD; // AF Open-Drain
	GPIO_InitStruct.Pull      = GPIO_PULLUP;
	GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;

	GPIO_InitStruct.Pin       = GPIO_PIN_8;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* 			I2C RX (SDA) GPIO pin configuration  */
	GPIO_InitStruct.Pin = GPIO_PIN_9;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);


	/*
	 *	(##) DMA Configuration
	 * 		(+++) Declare a DMA_HandleTypeDef handle structure for the 
	 * 			transmit or receive stream 
	 */ 
	static DMA_HandleTypeDef hdma_rx;
	/* 		(+++) Enable DMA1 clock */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* Configure the DMA handler for Receiving process */
	hdma_rx.Instance                 = DMA1_Stream2;
	hdma_rx.Init.Channel             = DMA_CHANNEL_3;
	hdma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
	hdma_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
	hdma_rx.Init.MemInc              = DMA_MINC_ENABLE;
	hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
	hdma_rx.Init.Mode                = DMA_NORMAL;
	hdma_rx.Init.Priority            = DMA_PRIORITY_HIGH;
	hdma_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
	hdma_rx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
	hdma_rx.Init.MemBurst            = DMA_MBURST_INC4;
	hdma_rx.Init.PeriphBurst         = DMA_PBURST_INC4;

	HAL_DMA_Init(&hdma_rx);

	/* Associate the initialized DMA handle to the the I2C handle */
	__HAL_LINKDMA(hi2c, hdmarx, hdma_rx);


	/*		(+++) Configure the priority and enable the NVIC for the 
	 *		transfer complete interrupt on the DMA Tx or Rx Stream
	 */

	/* NVIC configuration for DMA transfer complete interrupt (I2C3_RX) */
	HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);

	/*##-7- Configure the NVIC for I2C #########################################*/
	/* NVIC for I2C3 */
	HAL_NVIC_SetPriority(I2C3_ER_IRQn, 0, 1);
	HAL_NVIC_EnableIRQ(I2C3_ER_IRQn);
	HAL_NVIC_SetPriority(I2C3_EV_IRQn, 0, 2);
	HAL_NVIC_EnableIRQ(I2C3_EV_IRQn);
}

/**
 * @brief  DeInitializes the I2C MSP.
 * @note   This functiona is called from HAL_DeInit() function to perform system
 *         level de-initialization (GPIOs, clock, DMA, interrupt).
 * @retval None
 */
void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c) {
	/*##-1- Reset peripherals ##################################################*/
	__HAL_RCC_I2C3_FORCE_RESET();
	__HAL_RCC_I2C3_RELEASE_RESET();

	/*##-2- Disable peripherals and GPIO Clocks ################################*/
	HAL_GPIO_DeInit(GPIOA, GPIO_PIN_8);
	HAL_GPIO_DeInit(GPIOC, GPIO_PIN_9);

	/*##-3- Disable the DMA Streams ############################################*/
	/* De-Initialize the DMA Stream associate to reception process */
	//HAL_DMA_DeInit(&hdma_rx);
	HAL_DMA_DeInit(hi2c->hdmarx);

	/*##-4- Disable the NVIC for DMA ###########################################*/
	HAL_NVIC_DisableIRQ(DMA1_Stream2_IRQn);

	/*##-5- Disable the NVIC for I2C ###########################################*/
	HAL_NVIC_DisableIRQ(I2C3_ER_IRQn);
	HAL_NVIC_DisableIRQ(I2C3_EV_IRQn);
}


/**
 * @brief UART MSP Initialization 
 *        This function configures the hardware resources used in this example: 
 *           - Peripheral's clock enable
 *           - Peripheral's GPIO Configuration  
 *           - NVIC configuration for UART interrupt request enable
 * @param huart: UART handle pointer
 * @retval None
 */
void HAL_UART_MspInit(UART_HandleTypeDef *huart) {  
	GPIO_InitTypeDef  GPIO_InitStruct;

	/*##-1- Enable peripherals and GPIO Clocks #################################*/
	/* Enable GPIO TX/RX clock */
	USARTx_TX_GPIO_CLK_ENABLE();
	USARTx_RX_GPIO_CLK_ENABLE();
	/* Enable USART2 clock */
	USARTx_CLK_ENABLE(); 

	/*##-2- Configure peripheral GPIO ##########################################*/  
	/* UART TX GPIO pin configuration  */
	GPIO_InitStruct.Pin       = USARTx_TX_PIN;
	GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull      = GPIO_NOPULL;
	GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
	GPIO_InitStruct.Alternate = USARTx_TX_AF;

	HAL_GPIO_Init(USARTx_TX_GPIO_PORT, &GPIO_InitStruct);

	/* UART RX GPIO pin configuration  */
	GPIO_InitStruct.Pin = USARTx_RX_PIN;
	GPIO_InitStruct.Alternate = USARTx_RX_AF;

	HAL_GPIO_Init(USARTx_RX_GPIO_PORT, &GPIO_InitStruct);

	/*##-3- Configure the NVIC for UART ########################################*/
	/* NVIC for USART1 */
	HAL_NVIC_SetPriority(USARTx_IRQn, 0, 1);
	HAL_NVIC_EnableIRQ(USARTx_IRQn);
}

/**
 * @brief UART MSP De-Initialization 
 *        This function frees the hardware resources used in this example:
 *          - Disable the Peripheral's clock
 *          - Revert GPIO and NVIC configuration to their default state
 * @param huart: UART handle pointer
 * @retval None
 */
void HAL_UART_MspDeInit(UART_HandleTypeDef *huart) {
	/*##-1- Reset peripherals ##################################################*/
	USARTx_FORCE_RESET();
	USARTx_RELEASE_RESET();

	/*##-2- Disable peripherals and GPIO Clocks #################################*/
	/* Configure UART Tx as alternate function  */
	HAL_GPIO_DeInit(USARTx_TX_GPIO_PORT, USARTx_TX_PIN);
	/* Configure UART Rx as alternate function  */
	HAL_GPIO_DeInit(USARTx_RX_GPIO_PORT, USARTx_RX_PIN);

	/*##-3- Disable the NVIC for UART ##########################################*/
	HAL_NVIC_DisableIRQ(USARTx_IRQn);
}


/**
 * @brief TIM MSP Initialization 
 *        This function configures the hardware resources used in this example: 
 *           - Peripheral's clock enable
 *           - Peripheral's GPIO Configuration  
 * @param htim: TIM handle pointer
 * @retval None
 */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
	/*##-1- Enable peripherals and GPIO Clocks #################################*/
	/* TIMx Peripheral clock enable */
	__HAL_RCC_TIM2_CLK_ENABLE();

	/*##-2- Configure the NVIC for TIMx ########################################*/
	/* Set Interrupt Group Priority */ 
	HAL_NVIC_SetPriority(TIM2_IRQn, 4, 0);

	/* Enable the TIMx global Interrupt */
	HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

/**
 * @brief  Initializes the PPP MSP.
 * @note   This functiona is called from HAL_PPP_Init() function to perform 
 *         peripheral(PPP) system level initialization (GPIOs, clock, DMA, interrupt)
 * @retval None
 */
void HAL_PPP_MspInit(void)
{

}

/**
 * @brief  DeInitializes the PPP MSP.
 * @note   This functiona is called from HAL_PPP_DeInit() function to perform 
 *         peripheral(PPP) system level de-initialization (GPIOs, clock, DMA, interrupt)
 * @retval None
 */
void HAL_PPP_MspDeInit(void)
{

}

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
