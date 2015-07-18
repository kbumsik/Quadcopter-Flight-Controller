/**
 * @author  Tilen Majerle
 * @email   tilen@majerle.eu
 * @website http://stm32f4-discovery.com
 * @link    http://stm32f4-discovery.com/2014/10/library-43-mpu-6050-6-axes-gyro-accelerometer-stm32f4/
 * @version v1.0
 * @ide     Keil uVision
 * @license GNU GPL v3
 * @brief   MPU6050 library for STM32F4xx
 *
@verbatim
   ----------------------------------------------------------------------
    Copyright (C) Tilen Majerle, 2015

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
@endverbatim
 */
#ifndef KB_MPU9150_H
#define KB_MPU9150_H 100

/* C++ detection */
#ifdef __cplusplus
extern "C" {
#endif

/**
 * @addtogroup TM_STM32F4xx_Libraries
 * @{
 */

/**
 * @defgroup TM_MPU6050
 * @brief    MPU6050 library for STM32F4xx - http://stm32f4-discovery.com/2014/10/library-43-mpu-6050-6-axes-gyro-accelerometer-stm32f4/
 * @{
 *
 * \par Default pinout
 *
@verbatim
MPU6050		STM32F4xx	Descrption

SCL			PA8			Clock line for I2C
SDA			PC9			Data line for I2C
VCC			3.3V
GND			GND
AD0			-			If pin is low, I2C address is 0xD0, if pin is high, the address is 0xD2
@endverbatim
 *
 * \par Changelog
 *
@verbatim
 Version 1.0
  - First release
@endverbatim
 *
 * \par Dependencies
 *
@verbatim
 - STM32F4xx
 - STM32F4xx RCC
 - STM32F4xx GPIO
 - STM32F4xx I2C
 - defines.h
 - TM I2C
@endverbatim
 */

#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_i2c.h"
#include "defines.h"
#include "tm_stm32f4_i2c.h"
#include "tm_stm32f4_mpu6050.h"
/**
 * Descrption of the magnetometer of MPU9150:
 * The magnetometer, AK8975, is connected separatly as an auxiliary I2C device.
 * So it has different I2C address and the MPU6050 must enable Pass-Through Mode to communicate.
 * When the Pass-Through Mode is enabled, ES_DA and ES_CL pin are connected to SDA and SCL directly.
 *
 * AD0 Pin should be connected to GND to use the magnetometer.
 */

/**
 * @defgroup TM_LIB_Macros
 * @brief    Library defines
 * @{
 */

/* Default I2C used */
#ifndef MPU6050_I2C
#define	MPU6050_I2C					I2C3
#define MPU6050_I2C_PINSPACK		TM_I2C_PinsPack_1
#endif

/* Default I2C clock */
#ifndef MPU6050_I2C_CLOCK
#define MPU6050_I2C_CLOCK			400000
#endif

/* Default I2C address */
#define MPU9150_MAGNET_I2C_ADDR			0x0C

/* Who I am register value, meaning the device ID of AKM8975 */
#define MPU9150_MAGNET_I_AM				0x48

/* MPU9150 registers */
#define MPU9150_MAGNET_WIA  		0x00
#define MPU9150_MAGNET_INFO   	0x01
#define MPU9150_MAGNET_ST1  		0x02
#define MPU9150_MAGNET_HXL  		0x03
#define MPU9150_MAGNET_HXH  		0x04
#define MPU9150_MAGNET_HYL  		0x05
#define MPU9150_MAGNET_HYH  		0x06
#define MPU9150_MAGNET_HZL  		0x06
#define MPU9150_MAGNET_HZH  		0x08
#define MPU9150_MAGNET_ST2  		0x09
#define MPU9150_MAGNET_CNTL   	0x0A
#define MPU9150_MAGNET_ASTC   	0x0C
#define MPU9150_MAGNET_I2CDIS   0x0F
#define MPU9150_MAGNET_ASAX   	0x10
#define MPU9150_MAGNET_ASAY   	0x11
#define MPU9150_MAGNET_ASAZ   	0x12

/* magnetometer sensitivity in uT */
#define	MPU9150_MAGNET_SENS	((float) 0.3)

/* Settings for CNTL register */
#define MPU9150_MAGNET_CNTL_POWER_DOWN	0X0
#define MPU9150_MAGNET_CNTL_SINGLE_MEAS	0X1
#define MPU9150_MAGNET_CNTL_SELF_TEST		0X8
#define MPU9150_MAGNET_CNTL_FUSE_ROM		0XF

/* sensitivity adjustment equation */
#define MPU9150_MAGNET_ADJ(VALUE,ASA)	(float)(VALUE*(((ASA-128)*0.5/128)+1))

/**
 * @}
 */

/**
 * @defgroup TM_MPU9150_Typedefs
 * @brief    Library Typedefs
 * @{
 */



/**
 * @brief  Parameters for gyroscope range
 */
typedef enum {
	TM_MPU6050_Gyroscope_250s = 0x00,  /*!< Range is +- 250 degrees/s */
	TM_MPU6050_Gyroscope_500s = 0x01,  /*!< Range is +- 500 degrees/s */
	TM_MPU6050_Gyroscope_1000s = 0x02, /*!< Range is +- 1000 degrees/s */
	TM_MPU6050_Gyroscope_2000s = 0x03  /*!< Range is +- 2000 degrees/s */
} TM_MPU6050_Gyroscope_t;

/**
 * @brief  Main MPU6050 structure
 */
typedef struct {
	/* Private */
	uint8_t Address;         /*!< I2C address of device. Only for private use */
	float Gyro_Mult;         /*!< Gyroscope corrector from raw data to "degrees/s". Only for private use */
	float Acce_Mult;         /*!< Accelerometer corrector from raw data to "g". Only for private use */
	/* Public */
	int16_t Accelerometer_X; /*!< Accelerometer value X axis */
	int16_t Accelerometer_Y; /*!< Accelerometer value Y axis */
	int16_t Accelerometer_Z; /*!< Accelerometer value Z axis */
	int16_t Gyroscope_X;     /*!< Gyroscope value X axis */
	int16_t Gyroscope_Y;     /*!< Gyroscope value Y axis */
	int16_t Gyroscope_Z;     /*!< Gyroscope value Z axis */

	float Temperature;       /*!< Temperature in degrees */
} KB_MPU6050_t;

/**
 * @}
 */

/**
 * @defgroup TM_MPU6050_Functions
 * @brief    Library Functions
 * @{
 */

/**
 * @brief  Initializes MPU6050 and I2C peripheral
 * @param  *DataStruct: Pointer to empty @ref TM_MPU6050_t structure
 * @param   DeviceNumber: MPU6050 has one pin, AD0 which can be used to set address of device.
 *          This feature allows you to use 2 different sensors on the same board with same library.
 *          If you set AD0 pin to low, then this parameter should be TM_MPU6050_Device_0,
 *          but if AD0 pin is high, then you should use TM_MPU6050_Device_1
 *
 *          Parameter can be a value of @ref TM_MPU6050_Device_t enumeration
 * @param  AccelerometerSensitivity: Set accelerometer sensitivity. This parameter can be a value of @ref TM_MPU6050_Accelerometer_t enumeration
 * @param  GyroscopeSensitivity: Set gyroscope sensitivity. This parameter can be a value of @ref TM_MPU6050_Gyroscope_t enumeration
 * @retval Status:
 *            - TM_MPU6050_Result_t: Everything OK
 *            - Other member: in other cases
 */
TM_MPU6050_Result_t TM_MPU6050_Init(TM_MPU6050_t* DataStruct, TM_MPU6050_Device_t DeviceNumber, TM_MPU6050_Accelerometer_t AccelerometerSensitivity, TM_MPU6050_Gyroscope_t GyroscopeSensitivity);

/**
 * @brief  Reads accelerometer data from sensor
 * @param  *DataStruct: Pointer to @ref TM_MPU6050_t structure to store data to
 * @retval Member of @ref TM_MPU6050_Result_t:
 *            - TM_MPU6050_Result_Ok: everything is OK
 *            - Other: in other cases
 */
TM_MPU6050_Result_t TM_MPU6050_ReadAccelerometer(TM_MPU6050_t* DataStruct);

/**
 * @brief  Reads gyroscope data from sensor
 * @param  *DataStruct: Pointer to @ref TM_MPU6050_t structure to store data to
 * @retval Member of @ref TM_MPU6050_Result_t:
 *            - TM_MPU6050_Result_Ok: everything is OK
 *            - Other: in other cases
 */
TM_MPU6050_Result_t TM_MPU6050_ReadGyroscope(TM_MPU6050_t* DataStruct);

/**
 * @brief  Reads temperature data from sensor
 * @param  *DataStruct: Pointer to @ref TM_MPU6050_t structure to store data to
 * @retval Member of @ref TM_MPU6050_Result_t:
 *            - TM_MPU6050_Result_Ok: everything is OK
 *            - Other: in other cases
 */
TM_MPU6050_Result_t TM_MPU6050_ReadTemperature(TM_MPU6050_t* DataStruct);

/**
 * @brief  Reads accelerometer, gyroscope and temperature data from sensor
 * @param  *DataStruct: Pointer to @ref TM_MPU6050_t structure to store data to
 * @retval Member of @ref TM_MPU6050_Result_t:
 *            - TM_MPU6050_Result_Ok: everything is OK
 *            - Other: in other cases
 */
TM_MPU6050_Result_t TM_MPU6050_ReadAll(TM_MPU6050_t* DataStruct);

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/* C++ detection */
#ifdef __cplusplus
}
#endif

#endif
