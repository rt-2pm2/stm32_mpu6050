/**	
 * |----------------------------------------------------------------------
 * | Copyright (C) Luigi Pannocchi, 2018
 * | 
 * | This program is free software: you can redistribute it and/or modify
 * | it under the terms of the GNU General Public License as published by
 * | the Free Software Foundation, either version 3 of the License, or
 * | any later version.
 * |  
 * | This program is distributed in the hope that it will be useful,
 * | but WITHOUT ANY WARRANTY; without even the implied warranty of
 * | MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * | GNU General Public License for more details.
 * | 
 * | You should have received a copy of the GNU General Public License
 * | along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * |----------------------------------------------------------------------
 */

#include "stm32f4xx_hal.h"
#include "stm32f4_mpu6050.h"
#include <string.h>

static const float G = 9.81;

// Defined in the main.c
extern I2C_HandleTypeDef I2cHandle;

/* Convert a discrete 16 bit 2's complemenent value to the
 * associated real value.
 * The factor 2 in the computation is a fix for the fact
 * that the Accelerometer seems to return half of the 
 * acceleration.
 */
float toReal(const int16_t x, float scale) {
	if (x >= 0) 
		return ((x / 32767.0) * 2.0 * scale);
	else
		return ((x / 32768.0) * 2.0 * scale);
}

void Error_Handler(int errorcode) {
	switch(errorcode) {
		case HAL_I2C_ERROR_NONE: 
			break;
		case HAL_I2C_ERROR_BERR:
			while(1) {}
			break;
		case HAL_I2C_ERROR_ARLO:
			while(1) {}
			break;
		case HAL_I2C_ERROR_AF:
			while(1) {}
			break;
		case HAL_I2C_ERROR_OVR:
			while(1) {}
			break;
		case HAL_I2C_ERROR_DMA:
			while(1) {}
			break;
		case HAL_I2C_ERROR_TIMEOUT:
			while(1) {}
			break;
		default:
			while(1) {}
			break;
	}
}


HAL_StatusTypeDef MPU6050_Write(uint16_t addr, uint16_t regaddr, uint8_t data);

HAL_StatusTypeDef MPU6050_Read(uint16_t addr, uint16_t regaddr,
		uint8_t* data, uint16_t Size);



MPU6050_Status_t MPU6050_Init(MPU6050_data_str* mpu_str, 
		MPU6050_AccRange_t AccelerometerRange, 
		MPU6050_GyroRange_t GyroscopeRange) {

	HAL_StatusTypeDef halStatus;

	// Select the Accelerometer full scale
	switch (AccelerometerRange) {
		case MPU6050_Accelerometer_2G: 
			mpu_str->Acc_scale = G * 2.0; 
			break;
		case MPU6050_Accelerometer_4G:
			mpu_str->Acc_scale = G * 4.0;
			break;
		case MPU6050_Accelerometer_8G:
			mpu_str->Acc_scale = G * 8.0;
			break;
		case MPU6050_Accelerometer_16G:
			mpu_str->Acc_scale = G * 16.0;
			break;
		default:
			mpu_str->Acc_scale = G * 16.0;
	}

	// Select the Gyroscope full scale
	switch (GyroscopeRange) {
		case MPU6050_Gyroscope_250s:
			mpu_str->Gyro_scale = 250.0;
			break;
		case MPU6050_Gyroscope_500s:
			mpu_str->Gyro_scale = 500.0;
			break;
		case MPU6050_Gyroscope_1000s:
			mpu_str->Gyro_scale = 1000.0;
			break;
		case MPU6050_Gyroscope_2000s:
			mpu_str->Gyro_scale = 2000.0;
			break;
		default:
			mpu_str->Gyro_scale = 2000.0;
	}

	memset(&mpu_str->mem, 0, sizeof(MPU6050_mem));

	//reset the whole module first
	halStatus = MPU6050_Write(MPU6050_I2C_ADDR, MPU6050_PWR_MGMT_1, 1<<7);
	HAL_Delay(80);    //wait for 80ms for the gyro to stable

	if (halStatus != HAL_OK) 
		return MPU6050_ConnectionError; 
	
	//reset gyro and accel sensor
	halStatus = MPU6050_Write(MPU6050_I2C_ADDR, MPU6050_SIGNAL_PATH_RESET, 0x07); 	
	if (halStatus != HAL_OK) 
		return MPU6050_ConnectionError; 

	//PLL with Z axis gyroscope reference
	halStatus = MPU6050_Write(MPU6050_I2C_ADDR, MPU6050_PWR_MGMT_1, MPU6050_CLOCK_PLL_ZGYRO);
	if (halStatus != HAL_OK) 
		return MPU6050_ConnectionError; 

	//DLPF_CFG = 1: Fs=1khz; bandwidth=188hz 
	halStatus = MPU6050_Write(MPU6050_I2C_ADDR, MPU6050_CONFIG, 0x01);
	if (halStatus != HAL_OK) 
		return MPU6050_ConnectionError; 

	//(1Khz / 2 ) = 500Hz sample rate ~ 2ms
	halStatus = MPU6050_Write(MPU6050_I2C_ADDR, MPU6050_SMPLRT_DIV, 0x01);
	if (halStatus != HAL_OK) 
		return MPU6050_ConnectionError; 

	//Gyro scale setting
	halStatus = MPU6050_Write(MPU6050_I2C_ADDR, MPU6050_GYRO_CONFIG, GyroscopeRange << 3);
	if (halStatus != HAL_OK) 
		return MPU6050_ConnectionError; 

	//Accel full scale setting
	halStatus = MPU6050_Write(MPU6050_I2C_ADDR, MPU6050_ACCEL_CONFIG, AccelerometerRange << 3);
	if (halStatus != HAL_OK) 
		return MPU6050_ConnectionError; 

	//interrupt status bits are cleared on any read operation
	halStatus = MPU6050_Write(MPU6050_I2C_ADDR, MPU6050_INT_PIN_CFG, 1<<4);
	if (halStatus != HAL_OK) 
		return MPU6050_ConnectionError; 

	// interupt occurs when a write operation to all sensor
	// register has been completed.
	halStatus = MPU6050_Write(MPU6050_I2C_ADDR, MPU6050_INT_ENABLE, 1<<0);   
	if (halStatus != HAL_OK) 
		return MPU6050_ConnectionError; 

	return MPU6050_Ok;	
}

//Gyro scale setting
MPU6050_Status_t MPU6050_SetGyroScale(MPU6050_data_str* mpu_str, 
		MPU6050_GyroRange_t GyroscopeRange) {
	
	HAL_StatusTypeDef halStatus;

	// Select the Gyroscope full scale
	switch (GyroscopeRange) {
		case MPU6050_Gyroscope_250s:
			mpu_str->Gyro_scale = 250.0;
			break;
		case MPU6050_Gyroscope_500s:
			mpu_str->Gyro_scale = 500.0;
			break;
		case MPU6050_Gyroscope_1000s:
			mpu_str->Gyro_scale = 1000.0;
			break;
		case MPU6050_Gyroscope_2000s:
			mpu_str->Gyro_scale = 2000.0;
			break;
		default:
			mpu_str->Gyro_scale = 2000.0;
	}

	halStatus = MPU6050_Write(MPU6050_I2C_ADDR, MPU6050_GYRO_CONFIG, GyroscopeRange << 3);
	if (halStatus != HAL_OK)
		return MPU6050_Ok;
	else
		return MPU6050_ConnectionError;
}

//Accelerometer scale setting
MPU6050_Status_t MPU6050_SetAcceleroemeterScale(MPU6050_data_str* mpu_str, 
		MPU6050_AccRange_t AccelerometerRange) { 

	HAL_StatusTypeDef halStatus;

	// Select the Accelerometer full scale
	switch (AccelerometerRange) {
		case MPU6050_Accelerometer_2G: 
			mpu_str->Acc_scale = G * 2.0; ;
			break;
		case MPU6050_Accelerometer_4G:
			mpu_str->Acc_scale = G * 4.0;
			break;
		case MPU6050_Accelerometer_8G:
			mpu_str->Acc_scale = G * 8.0;
			break;
		case MPU6050_Accelerometer_16G:
			mpu_str->Acc_scale = G * 16.0;
			break;
		default:
			mpu_str->Acc_scale = G * 16.0;
	}
	
	//Accel full scale setting
	halStatus = MPU6050_Write(MPU6050_I2C_ADDR, MPU6050_ACCEL_CONFIG, AccelerometerRange << 3);
	if (halStatus != HAL_OK)
		return MPU6050_Ok;
	else
		return MPU6050_ConnectionError;

}

MPU6050_Status_t MPU6050_ReqAccelerometer(MPU6050_data_str* ds) {
	HAL_StatusTypeDef halStatus;	

	halStatus = MPU6050_Read(MPU6050_I2C_ADDR, MPU6050_ACCEL_XOUT_H,
		(uint8_t*)&ds->mem, 3 * sizeof(int16_t));
	
	if (halStatus != HAL_OK)	
		return MPU6050_ConnectionError;
	else 
		return MPU6050_Ok;
}

MPU6050_Status_t MPU6050_ReqGyroscope(MPU6050_data_str* ds) {
	HAL_StatusTypeDef halStatus;

	halStatus = MPU6050_Read(MPU6050_I2C_ADDR, MPU6050_GYRO_XOUT_H,
		(uint8_t*) &ds->mem + 4 * sizeof(int16_t), 3 * sizeof(int16_t));

	if (halStatus != HAL_OK)
		return MPU6050_ConnectionError;
	else 
		return MPU6050_Ok;
}

void MPU6050_UpdateAccelerometer(MPU6050_data_str* ds) {
	ds->priv_data.Acc_X = 
		((uint16_t)(ds->mem.Acc_X & 0x00FF) << 8) | (uint16_t)(ds->mem.Acc_X >> 8);
	ds->priv_data.Acc_Y = 
		((uint16_t)(ds->mem.Acc_Y & 0x00FF) << 8) | (uint16_t)(ds->mem.Acc_Y >> 8);
	ds->priv_data.Acc_Z = 
		((uint16_t)(ds->mem.Acc_Z & 0x00FF) << 8) | (uint16_t)(ds->mem.Acc_Z >> 8);
}
		
void MPU6050_UpdateGyroscope(MPU6050_data_str* ds) {
	ds->priv_data.Gyro_X = 
		((uint16_t)(ds->mem.Gyro_X & 0x00FF) << 8) | (uint16_t)(ds->mem.Gyro_X >> 8);
	ds->priv_data.Gyro_Y = 
		((uint16_t)(ds->mem.Gyro_Y & 0x00FF) << 8) | (uint16_t)(ds->mem.Gyro_Y >> 8);
	ds->priv_data.Gyro_Z = 
		((uint16_t)(ds->mem.Gyro_Z & 0x00FF) << 8) | (uint16_t)(ds->mem.Gyro_Z >> 8);
}


MPU6050_Status_t MPU6050_ReqTemperature(MPU6050_data_str* ds) {
	HAL_StatusTypeDef halStatus;

	halStatus = MPU6050_Read(MPU6050_I2C_ADDR, MPU6050_TEMP_OUT_H,
		(uint8_t*) &ds->mem + 3 * sizeof(int16_t), sizeof(int16_t));

	if (halStatus != HAL_OK)
		return MPU6050_ConnectionError;
	else
		return MPU6050_Ok;
}

void MPU6050_UpdateTemperature(MPU6050_data_str* ds) {

	ds->priv_data.Temp = ((uint16_t)(ds->mem.Temp & 0x00FF) << 8) | 
			(uint16_t)(ds->mem.Temp >> 8);
}


MPU6050_Status_t MPU6050_ReqAll(MPU6050_data_str* ds, uint32_t timestamp) {
	//MPU6050_Status_t ret;
	HAL_StatusTypeDef halStatus;

	ds->priv_data.timestamp = timestamp;

	halStatus = MPU6050_Read(MPU6050_I2C_ADDR, MPU6050_ACCEL_XOUT_H,
		(uint8_t*) &ds->mem, sizeof(MPU6050_mem));
	
	if (halStatus != HAL_OK)
		return MPU6050_Ok;
	else
		return MPU6050_ConnectionError;
}


void MPU6050_Publish(MPU6050_data_str* ds) {
	memcpy(&ds->sensor_data, &ds->priv_data, sizeof(IMU_Data));
}

void MPU6050_UpdateAll(MPU6050_data_str* ds) {

	// Fix the byte order of the IMU data
	//	      Move the lower part (H) up    |  Move the upper part (L) down
	ds->priv_data.Acc_X = 
		((uint16_t)(ds->mem.Acc_X & 0x00FF) << 8) | (uint16_t)(ds->mem.Acc_X >> 8);
	ds->priv_data.Acc_Y = 
		((uint16_t)(ds->mem.Acc_Y  & 0x00FF) << 8) | (uint16_t)(ds->mem.Acc_Y >> 8);
	ds->priv_data.Acc_Z = 
		((uint16_t)(ds->mem.Acc_Z  & 0x00FF) << 8) | (uint16_t)(ds->mem.Acc_Z >> 8);

	ds->priv_data.Gyro_X = 
		((uint16_t)(ds->mem.Gyro_X & 0x00FF) << 8) | (uint16_t)(ds->mem.Gyro_X >> 8);
	ds->priv_data.Gyro_Y = 
		((uint16_t)(ds->mem.Gyro_Y & 0x00FF) << 8) | (uint16_t)(ds->mem.Gyro_Y >> 8);
	ds->priv_data.Gyro_Z = 
		((uint16_t)(ds->mem.Gyro_Z & 0x00FF) << 8) | (uint16_t)(ds->mem.Gyro_Z >> 8);

	ds->priv_data.Temp = 
		((uint16_t)(ds->mem.Temp & 0x00FF) << 8) | (uint16_t)(ds->mem.Temp >> 8);

}


void MPU6050_GetAccelerometer(MPU6050_data_str* ds, float acc[3], uint32_t* timestamp) {
	acc[0] = toReal(ds->sensor_data.Acc_X, ds->Acc_scale);
	acc[1] = toReal(ds->sensor_data.Acc_Y, ds->Acc_scale);
	acc[2] = toReal(ds->sensor_data.Acc_Z, ds->Acc_scale);

	*timestamp = ds->sensor_data.timestamp;
	return;
}

void MPU6050_GetGyro(MPU6050_data_str* ds, float gyro[3], uint32_t* timestamp) {
	gyro[0] = toReal(ds->sensor_data.Gyro_X, ds->Gyro_scale);
	gyro[1] = toReal(ds->sensor_data.Gyro_Y, ds->Gyro_scale);
	gyro[2] = toReal(ds->sensor_data.Gyro_Z, ds->Gyro_scale);

	return;
}

HAL_StatusTypeDef MPU6050_Write(uint16_t addr, uint16_t regaddr, uint8_t data) {
	uint8_t locdata = data;
	size_t trials = 0;
	int errcode;
	HAL_StatusTypeDef ret = HAL_BUSY;
	while (ret != HAL_OK) { 
		ret = HAL_I2C_Mem_Write(&I2cHandle, addr<<1,
				regaddr, I2C_MEMADD_SIZE_8BIT, &locdata, 1, 4);
		// If the write is not succesful check the error
		if (ret != HAL_OK) {
			errcode = HAL_I2C_GetError(&I2cHandle); 
			if (errcode != HAL_I2C_ERROR_AF)
				Error_Handler(errcode);
			else
				trials++;
		}   
	}	
	return ret;
}

HAL_StatusTypeDef MPU6050_Read(uint16_t addr, uint16_t regaddr,
		uint8_t* data, uint16_t Size) {
	int errcode;
	size_t trials = 0;
	HAL_StatusTypeDef ret = HAL_BUSY;

	while (ret != HAL_OK) {
		ret = HAL_I2C_Mem_Read_DMA(&I2cHandle, addr<<1, regaddr,
				I2C_MEMADD_SIZE_8BIT, data, Size);
		// If the write is not succesful check the error
		if (ret != HAL_OK) {
			errcode = HAL_I2C_GetError(&I2cHandle); 
			if (errcode != HAL_I2C_ERROR_AF)
				Error_Handler(errcode);
			else
				trials++;
		}
	}	
	return ret;
}
