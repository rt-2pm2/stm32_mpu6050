/**
 * @file main.c
 */

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4_mpu6050.h"
#include "system.h"
#include "main.h"

static void Error_Handler(void);

/* I2C handler declaration */
I2C_HandleTypeDef I2cHandle;

/* UART handler declaration */
UART_HandleTypeDef UartHandle;

/* TIM handler declaration */
TIM_HandleTypeDef TimHandle;

/* MPU Data structure */
MPU6050_data_str mpu_data;

float acc[3];
float gyro[3]; 
uint8_t readflag = 1;


int float2buff(float x, char buffer[], int MaxBuffSize, int Nint, int Ndec) {
	int i;
	char int_buff[Nint];
	char digit;

	int dig_counter;
	float dec_p;
	int int_p;

	// Check the sign and separate the integer/decimal part of the number
	if (x < 0.0) {
		buffer[0] = '-';
		int_p = (int) (-x);
		dec_p = (-x) - int_p;
	} else {
		buffer[0] = '+';
		int_p = (int) (x);
		dec_p = x - int_p;
	}

	if (int_p == 0) {
		int_buff[0] = '0';
		dig_counter = 1;
	}
	else {
		dig_counter = 0;

		while (int_p > 0) {
			int_buff[dig_counter] = (int_p % 10) + '0';
			int_p /= 10;
			dig_counter++;
		}
	}

	for (i = 0; i < dig_counter; i++) {
		buffer[i + 1] = int_buff[dig_counter - 1 - i];
	}
	buffer[dig_counter + 1] = '.';

	for (i = 0; i < Ndec; i++) {
		dec_p *= 10;
		digit = (char)dec_p;
		dec_p -= digit;
		buffer[dig_counter + i + 2] = digit + '0';
	}
	
	return (Ndec + dig_counter + 2);
}

// =============================================================================

int main() {

	HAL_StatusTypeDef halStatus;
	MPU6050_Status_t mpu_status;
	/* STM32F4xx HAL library initialization:
	   - Configure the Flash prefetch, instruction and Data caches
	   - Configure the Systick to generate an interrupt each 1 msec
	   - Set NVIC Group Priority to 4
	   - Global MSP (MCU Support Package) initialization
	   */
	HAL_Init();		
	SetSysClock();

	// ## Configure the UART peripheral #####################################
	halStatus = StartUART(&UartHandle);
	if (halStatus != HAL_OK) {
		  Error_Handler();
	}


	// ## Configure the I2C peripheral ######################################
	halStatus = StartI2C(&I2cHandle);
	if (halStatus != HAL_OK) {
		Error_Handler();    
	}

	// ## Configure the TIM peripheral ######################################
	halStatus = StartTIM(&TimHandle);
	if (halStatus != HAL_OK) {
		Error_Handler();
	}


	// Initialize the mpu6050
	mpu_status = MPU6050_Init(&mpu_data, MPU6050_Accelerometer_2G, MPU6050_Gyroscope_250s);
	if (mpu_status != MPU6050_Ok) {
		Error_Handler();
	}

	// Configure the interrupt on the mpu line
	EXTILine4_Config();

	while (1) { 
	
		HAL_Delay(20);

		//int digs = 0;
		//tr_status = HAL_UART_Transmit_IT(&UartHandle, (uint8_t*)data_buff, digs);
			
	}
	
	return 0;
}


void Error_Handler(void) {
	while(1) {
	}
}


// ===================== Callbacks ========================
/**
  * @brief  Period elapsed callback in non blocking mode
  * @param  htim: TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{ 
	return;
}



void HAL_I2C_ErrorCallback(I2C_HandleTypeDef* hi2c) {
	Error_Handler();	
}


/**
  * @brief  Defines the routine at the end of DMA data reception 
  * @param  handle to i2c 
  * @retval None
  */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef* hi2c) {
	MPU6050_UpdateAll(&mpu_data);

	// Update the shared data only if the data is not
	// currently used
	if (readflag) {
		void MPU6050_Publish(&mpu_data);
	}

	return;	
}


/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == INT_MPU_PIN) {
		// Take the current time
		uint32_t time = __HAL_TIM_GetCounter(&TimHandle);
		MPU6050_ReqAll(&mpu_data, time);
	}
}
