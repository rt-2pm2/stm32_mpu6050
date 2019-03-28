/**
 * @author	Luigi Pannocchi 
 * @email	l.pannocchi@gmail.com` 
 * @license GNU GPL v3
 *	
 ----------------------------------------------------------------------
 Copyright (C) Luigi Pannocchi, 2019

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 ----------------------------------------------------------------------
 */
#ifndef MPU6050_H
#define MPU6050_H

//#include "stm32f4xx.h"

/* Default I2C clock */
#ifndef MPU6050_I2C_CLOCK
#define MPU6050_I2C_CLOCK			400000
#endif

/* Default I2C address */
#define MPU6050_I2C_ADDR			0x68

/* Who I am register value */
#define MPU6050_I_AM				0x68

/* MPU6050 registers */
#define MPU6050_AUX_VDDIO			0x01
#define MPU6050_SMPLRT_DIV			0x19
#define MPU6050_CONFIG				0x1A
#define MPU6050_GYRO_CONFIG			0x1B
#define MPU6050_ACCEL_CONFIG		0x1C
#define MPU6050_MOTION_THRESH		0x1F
#define MPU6050_INT_PIN_CFG			0x37
#define MPU6050_INT_ENABLE			0x38
#define MPU6050_INT_STATUS			0x3A
#define MPU6050_ACCEL_XOUT_H		0x3B
#define MPU6050_ACCEL_XOUT_L		0x3C
#define MPU6050_ACCEL_YOUT_H		0x3D
#define MPU6050_ACCEL_YOUT_L		0x3E
#define MPU6050_ACCEL_ZOUT_H		0x3F
#define MPU6050_ACCEL_ZOUT_L		0x40
#define MPU6050_TEMP_OUT_H			0x41
#define MPU6050_TEMP_OUT_L			0x42
#define MPU6050_GYRO_XOUT_H			0x43
#define MPU6050_GYRO_XOUT_L			0x44
#define MPU6050_GYRO_YOUT_H			0x45
#define MPU6050_GYRO_YOUT_L			0x46
#define MPU6050_GYRO_ZOUT_H			0x47
#define MPU6050_GYRO_ZOUT_L			0x48
#define MPU6050_MOT_DETECT_STATUS	0x61
#define MPU6050_SIGNAL_PATH_RESET	0x68
#define MPU6050_MOT_DETECT_CTRL		0x69
#define MPU6050_USER_CTRL			0x6A
#define MPU6050_PWR_MGMT_1			0x6B
#define MPU6050_PWR_MGMT_2			0x6C
#define MPU6050_FIFO_COUNTH			0x72
#define MPU6050_FIFO_COUNTL			0x73
#define MPU6050_FIFO_R_W			0x74
#define MPU6050_WHO_AM_I			0x75

/* Gyro sensitivities in °/s */
#define MPU6050_GYRO_SENS_250		((float) 131)
#define MPU6050_GYRO_SENS_500		((float) 65.5)
#define MPU6050_GYRO_SENS_1000		((float) 32.8)
#define MPU6050_GYRO_SENS_2000		((float) 16.4)

/* Acce sensitivities in g */
#define MPU6050_ACCE_SENS_2			((float) 16384)
#define MPU6050_ACCE_SENS_4			((float) 8192)
#define MPU6050_ACCE_SENS_8			((float) 4096)
#define MPU6050_ACCE_SENS_16		((float) 2048)

/* Clock Selection */
#define MPU6050_CLOCK_PLL_XGYRO (1)
#define MPU6050_CLOCK_PLL_YGYRO	(2)
#define MPU6050_CLOCK_PLL_ZGYRO (3)

/**
 * @brief  MPU6050 can have 2 different slave addresses, depends on it's input AD0 pin
 *         This feature allows you to use 2 different sensors with this library at the same time
 */
typedef enum {
	MPU6050_Device_0 = 0,   /*!< AD0 pin is set to low */
	MPU6050_Device_1 = 0x02 /*!< AD0 pin is set to high */
} MPU6050_Device_t;

/**
 * @brief  MPU6050 result enumeration	
 */
typedef enum {
	MPU6050_Ok = 0x00,
	MPU6050_ConnectionError
} MPU6050_Status_t;

/**
 * @brief  Parameters for accelerometer range
 */
typedef enum {
	MPU6050_Accelerometer_2G = 0x00, /*!< Range is +- 2G */
	MPU6050_Accelerometer_4G = 0x01, /*!< Range is +- 4G */
	MPU6050_Accelerometer_8G = 0x02, /*!< Range is +- 8G */
	MPU6050_Accelerometer_16G = 0x03 /*!< Range is +- 16G */
} MPU6050_AccRange_t;

/**
 * @brief  Parameters for gyroscope range
 */
typedef enum {
	MPU6050_Gyroscope_250s = 0x00,  /*!< Range is +- 250 degrees/s */
	MPU6050_Gyroscope_500s = 0x01,  /*!< Range is +- 500 degrees/s */
	MPU6050_Gyroscope_1000s = 0x02, /*!< Range is +- 1000 degrees/s */
	MPU6050_Gyroscope_2000s = 0x03  /*!< Range is +- 2000 degrees/s */
} MPU6050_GyroRange_t;


/**
 * @brief Structure representing the IMU internal memory containing
 * sensors data
 */
typedef struct {
	uint16_t Acc_X;		/*!< Accelerometer value X axis */
	uint16_t Acc_Y;		/*!< Accelerometer value Y axis */
	uint16_t Acc_Z;		/*!< Accelerometer value Z axis */	
	uint16_t Temp;		/*!< Temperature in degrees */
	uint16_t Gyro_X;	/*!< Gyroscope value X axis */
	uint16_t Gyro_Y;	/*!< Gyroscope value Y axis */
	uint16_t Gyro_Z;	/*!< Gyroscope value Z axis */
} MPU6050_mem;


/**
 * @brief Structure representing sensors data
 */
typedef struct {
	uint32_t timestamp;
	int16_t Acc_X;		/*!< Accelerometer value X axis */
	int16_t Acc_Y;		/*!< Accelerometer value Y axis */
	int16_t Acc_Z;		/*!< Accelerometer value Z axis */
	int16_t Gyro_X;		/*!< Gyroscope value X axis */
	int16_t Gyro_Y;		/*!< Gyroscope value Y axis */
	int16_t Gyro_Z;		/*!< Gyroscope value Z axis */
	int16_t Temp;		/*!< Temperature in degrees */
} IMU_Data;

/**
 * @brief  Main MPU6050 structure
 */
typedef struct {
	/* Private */
	I2C_HandleTypeDef* I2cHandle;

	uint8_t Address;	/*!< I2C address of device */
	float Gyro_2Real;	/*!< Gyroscope corrector from raw data to "deg/s" */
	float Acce_2Real;	/*!< Accelerometer corrector from raw data to "g" */

	float Acc_scale;	/*!< Accelerometer full scale [g] */
	float Gyro_scale;	/*!< Gyroscope full scale [deg/s] */

	// DATA
	MPU6050_mem mem;	/*!< Data represented as in the IMU memory */	

	IMU_Data priv_data;	/*!< Private sensor data */

	/* Public */
	IMU_Data sensor_data;	/*!< Public sensor data */

} MPU6050_data_str;



/**
 * @defgroup MPU6050_Functions
 * @brief    Library Functions
 * @{
 */

/**
 * @brief  Initializes MPU6050 and I2C peripheral
 * @param  *DataStruct: Pointer to empty @ref MPU6050_data_str structure
 * @param  AccelerometerSensitivity: Set accelerometer sensitivity. 
 * @param  GyroscopeSensitivity: Set gyroscope sensitivity.
 * @retval MPU6050_Status_t
 */
MPU6050_Status_t MPU6050_Init(MPU6050_data_str* data_str,
		MPU6050_AccRange_t AccelerometerRange, 
		MPU6050_GyroRange_t GyroscopeRange);

/**
 * @brief  Requests accelerometer data from sensor
 * @param  *DataStruct: Pointer to @ref MPU6050_data_str structure to store data to
 * @retval MPU6050_Status_t
 */
MPU6050_Status_t MPU6050_ReqAccelerometer(MPU6050_data_str* DataStruct);

/**
 * @brief  Requests gyroscope data from sensor
 * @param  *DataStruct: Pointer to @ref MPU6050_data_str structure to store data to
 * @param  timestamp timestamp of the sensor data
 * @retval MPU6050_Status_t
 */
MPU6050_Status_t MPU6050_ReqGyroscope(MPU6050_data_str* DataStruct, uint32_t timestamp);


/**
 * @brief  Requests temperature data from sensor
 * @param  *DataStruct: Pointer to @ref MPU6050_data_str structure to store data to
 * @param  timestamp timestamp of the sensor data
 * @retval MPU6050_Status_t
 */
MPU6050_Status_t MPU6050_ReqTemperature(MPU6050_data_str* DataStruct, uint32_t timestamp);


/**
 * @brief  Reqs accelerometer, gyroscope and temperature data from sensor
 * @param  *DataStruct Pointer to @ref MPU6050_data_str structure to store data to
 * @param  timestamp Timestamp of the sensor data
 * @retval MPU6050_Status_t
 */
MPU6050_Status_t MPU6050_ReqAll(MPU6050_data_str* DataStruct, uint32_t timestamp);


/**
 * @brief  Get the current accelerometer data 
 * @param  *DataStruct Pointer to @ref MPU6050_data_str structure to data data from
 * @param  acc Array of 3 floats 
 * @param  timestamp Pointer to timestamp variable 
 * @retval None  
 * */
void MPU6050_GetAccelerometer(MPU6050_data_str* ds, float acc[3], uint32_t* timestamp);

/**
 * @brief  Get the current gyroscope data 
 * @param  *DataStruct: Pointer to @ref MPU6050_data_str structure to data data from
 * @param  gyro: Array of 3 floats 
 * @param  *timestamp pointer to timestamp variable 
 * @retval None  
 * */
void MPU6050_GetGyro(MPU6050_data_str* ds, float gyro[3], uint32_t* timestamp);

void MPU6050_UpdateAll(MPU6050_data_str* DataStruct);
void MPU6050_UpdateAccelerometer(MPU6050_data_str* DataStruct);
void MPU6050_UpdateGyro(MPU6050_data_str* DataStruct);
void MPU6050_UpdateTemperature(MPU6050_data_str* DataStruct);

MPU6050_Status_t MPU6050_SetAcceleroemeterScale(MPU6050_data_str* mpu_str, 
		MPU6050_AccRange_t AccelerometerRange);



#endif
