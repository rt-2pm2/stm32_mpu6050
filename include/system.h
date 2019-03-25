/**
 * @file system.h
 *
 */

/**
  * @brief  Configures the system clock to run at 168 Mhz 
  * @param  None
  * @retval None
  *
  */
void SetSysClock(void);


/**
  * @brief Set up the UART interface
  * @param UartHandle Handler to the hal UART structure 
  * @retval Status of the HAL 
  *
  */
HAL_StatusTypeDef StartUART(UART_HandleTypeDef* UartHandle);


/**
  * @brief Set up the I2C interface
  * @param I2cHandle Handler to the hal I2C structure 
  * @retval Status of the HAL 
  *
  */
HAL_StatusTypeDef StartI2C(I2C_HandleTypeDef* I2cHandle);

/**
  * @brief Configure the EXTI line for the MPU interupt 
  * @param None
  * @retval None 
  *
  */
void EXTILine4_Config(void);

