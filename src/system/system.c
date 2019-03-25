/**
 * @file system.c
 *
 * This file provides the functions to setup the clock of the system
 */

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_conf.h"

#include "main.h"

/*!< Uncomment the following line if you need to relocate your vector Table in
     Internal SRAM. */
/* #define VECT_TAB_SRAM */
#define VECT_TAB_OFFSET  0x00 /*!< Vector Table base offset field. 
                                   This value must be a multiple of 0x200. */

uint32_t SystemCoreClock = 16000000;

const uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
const uint8_t APBPrescTable[8]  = {0, 0, 0, 0, 1, 2, 3, 4};

void SystemInit(void) {
	/* FPU settings ------------------------------------------------------------*/
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
	SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));  /* set CP10 and CP11 Full Access */
#endif
	/* Reset the RCC clock configuration to the default reset state ------------*/
	/* Set HSION bit */
	RCC->CR |= (uint32_t)0x00000001;

	/* Reset CFGR register */
	RCC->CFGR = 0x00000000;

	/* Reset HSEON, CSSON and PLLON bits */
	RCC->CR &= (uint32_t)(0xFFFFFFFF - (RCC_CR_HSEON + RCC_CR_CSSON + RCC_CR_PLLON));

	/* Reset PLLCFGR register */
	RCC->PLLCFGR = 0x24003010;

	/* Reset HSEBYP bit */
	RCC->CR &= (uint32_t)0xFFFBFFFF;

	/* Disable all interrupts */
	RCC->CIR = 0x00000000;

	/* Configure the Vector Table location add offset address ------------------*/
#ifdef VECT_TAB_SRAM
	SCB->VTOR = SRAM_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal SRAM */
#else
	SCB->VTOR = FLASH_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal FLASH */
#endif
}


/**
 * @brief  Update SystemCoreClock variable according to Clock Register Values.
 *         The SystemCoreClock variable contains the core clock (HCLK), it can
 *         be used by the user application to setup the SysTick timer or configure
 *         other parameters.
 *           
 * @note   Each time the core clock (HCLK) changes, this function must be called
 *         to update SystemCoreClock variable value. Otherwise, any configuration
 *         based on this variable will be incorrect.         
 *     
 * @note   - The system frequency computed by this function is not the real 
 *           frequency in the chip. It is calculated based on the predefined 
 *           constant and the selected clock source:
 *             
 *           - If SYSCLK source is HSI, SystemCoreClock will contain the HSI_VALUE(*)
 *                                              
 *           - If SYSCLK source is HSE, SystemCoreClock will contain the HSE_VALUE(**)
 *                          
 *           - If SYSCLK source is PLL, SystemCoreClock will contain the HSE_VALUE(**) 
 *             or HSI_VALUE(*) multiplied/divided by the PLL factors.
 *         
 *         (*) HSI_VALUE is a constant defined in stm32f4xx_hal_conf.h file (default value
 *             16 MHz) but the real value may vary depending on the variations
 *             in voltage and temperature.   
 *    
 *         (**) HSE_VALUE is a constant defined in stm32f4xx_hal_conf.h file (its value
 *              depends on the application requirements), user has to ensure that HSE_VALUE
 *              is same as the real frequency of the crystal used. Otherwise, this function
 *              may have wrong result.
 *                
 *         - The result of this function could be not correct when using fractional
 *           value for HSE crystal.
 *     
 * @param  None
 * @retval None
 */
void SystemCoreClockUpdate(void) {
	uint32_t tmp = 0, pllvco = 0, pllp = 2, pllsource = 0, pllm = 2;

	/* Get SYSCLK source -------------------------------------------------------*/
	tmp = RCC->CFGR & RCC_CFGR_SWS;

	switch (tmp)
	{
		case 0x00:  /* HSI used as system clock source */
			SystemCoreClock = HSI_VALUE;
			break;
		case 0x04:  /* HSE used as system clock source */
			SystemCoreClock = HSE_VALUE;
			break;
		case 0x08:  /* PLL used as system clock source */

			/* PLL_VCO = (HSE_VALUE or HSI_VALUE / PLL_M) * PLL_N
			   SYSCLK = PLL_VCO / PLL_P
			   */    
			pllsource = (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) >> 22;
			pllm = RCC->PLLCFGR & RCC_PLLCFGR_PLLM;

			if (pllsource != 0)
			{
				/* HSE used as PLL clock source */
				pllvco = (HSE_VALUE / pllm) * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6);
			}
			else
			{
				/* HSI used as PLL clock source */
				pllvco = (HSI_VALUE / pllm) * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6);
			}

			pllp = (((RCC->PLLCFGR & RCC_PLLCFGR_PLLP) >>16) + 1 ) *2;
			SystemCoreClock = pllvco/pllp;
			break;
		default:
			SystemCoreClock = HSI_VALUE;
			break;
	}
	/* Compute HCLK frequency --------------------------------------------------*/
	/* Get HCLK prescaler */
	tmp = AHBPrescTable[((RCC->CFGR & RCC_CFGR_HPRE) >> 4)];
	/* HCLK frequency */
	SystemCoreClock >>= tmp;
}


/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 168000000
  *            HCLK(Hz)                       = 168000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 8
  *            PLL_N                          = 336
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */
void SetSysClock(void) {
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  
  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | 
		  RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | 
		  RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

  /* STM32F405x/407x/415x/417x Revision Z devices: prefetch is supported  */
  if (HAL_GetREVID() == 0x1001)
  {
    /* Enable the Flash prefetch */
    __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
  }
}


/**
  * @brief  Configures EXTI Line0 (connected to PA4 pin) in interrupt mode
  * @param  None
  * @retval None
  */
void EXTILine4_Config(void) {
  GPIO_InitTypeDef   GPIO_InitStructure;

  /* Enable GPIOA clock */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  
  /* Configure PA4 pin as input floating */
  GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Pin = INT_MPU_PIN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Enable and set EXTI Line0 Interrupt to the lowest priority */
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);
}


/**
  * @brief Set up the UART interface
  * UART1 configured as follow:
  * - Word Length = 8 Bits
  * - Stop Bit = One Stop bit
  * - Parity = None 
  * - BaudRate = 9600 baud
  * - Hardware flow control disabled (RTS and CTS signals) 
  *
  * @param  UartHandle Handle to the UART interface
  * @retval Status of the HAL 
  *
  */
HAL_StatusTypeDef StartUART(UART_HandleTypeDef* UartHandle) { 

	HAL_StatusTypeDef ret;

	/* Put the USART peripheral in the Asynchronous mode (UART Mode) */
	UartHandle->Instance          = USARTx;

	UartHandle->Init.BaudRate     = USARTx_BAUD;
	UartHandle->Init.WordLength   = UART_WORDLENGTH_8B;
	UartHandle->Init.StopBits     = UART_STOPBITS_1;
	UartHandle->Init.Parity       = UART_PARITY_NONE;
	UartHandle->Init.HwFlowCtl    = UART_HWCONTROL_NONE;
	UartHandle->Init.Mode         = UART_MODE_TX_RX;
	UartHandle->Init.OverSampling = UART_OVERSAMPLING_16;

	ret = HAL_UART_Init(UartHandle);

	return ret;
}


/**
  * @brief Set up the I2C interface
  *
  * @param  I2cHandle Handle to the i2c structure
  * @retval Status of the HAL 
  *
  */

HAL_StatusTypeDef StartI2C(I2C_HandleTypeDef* I2cHandle) {
	HAL_StatusTypeDef ret;

	I2cHandle->Instance             = I2C3;

	I2cHandle->Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
	I2cHandle->Init.ClockSpeed      = 400000;
	I2cHandle->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	I2cHandle->Init.DutyCycle       = I2C_DUTYCYCLE_16_9;
	I2cHandle->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	I2cHandle->Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
	I2cHandle->Init.OwnAddress1     = I2C_ADDRESS;
	I2cHandle->Init.OwnAddress2     = 0xFE;

	ret = HAL_I2C_Init(I2cHandle);

	return ret;
}
