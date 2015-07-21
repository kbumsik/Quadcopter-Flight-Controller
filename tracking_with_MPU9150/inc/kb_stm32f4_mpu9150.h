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
 * @defgroup KB_MPU9150
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
#define MPU9150_MAGNET_I2C_ADDR		(0x0C<<1)

/* Who I am register value, meaning the device ID of AKM8975 */
#define MPU9150_MAGNET_I_AM			0x48

/* MPU9150 registers */
#define MPU9150_MAGNET_WIA  	0x00
#define MPU9150_MAGNET_INFO   	0x01
#define MPU9150_MAGNET_ST1  	0x02
/* MPU9150 Magnet data registers */
#define MPU9150_MAGNET_HXL  	0x03
#define MPU9150_MAGNET_HXH  	0x04
#define MPU9150_MAGNET_HYL  	0x05
#define MPU9150_MAGNET_HYH  	0x06
#define MPU9150_MAGNET_HZL  	0x06
#define MPU9150_MAGNET_HZH  	0x08
/* Other MPU9150 registers */
#define MPU9150_MAGNET_ST2  	0x09
#define MPU9150_MAGNET_CNTL   	0x0A
#define MPU9150_MAGNET_ASTC   	0x0C
#define MPU9150_MAGNET_I2CDIS   0x0F
/* Sensitivity Adjestment values */
#define MPU9150_MAGNET_ASAX   	0x10
#define MPU9150_MAGNET_ASAY   	0x11
#define MPU9150_MAGNET_ASAZ   	0x12

/* magnetometer sensitivity in uT */
#define	MPU9150_MAGNET_SENS	((float) 0.3)

/* sensitivity adjustment equation */
#define MPU9150_MAGNET_ADJ(VALUE,ASA)	((float)(VALUE*(((ASA-128)*0.5/128)+1)))

/**
 * @}
 */

/**
 * @defgroup KB_MPU9150_Typedefs
 * @brief    Library Typedefs
 * @{
 */


 typedef enum {
 	KB_MPU9150_Result_Ok = 0x00,          /*!< Everything OK */
 	KB_MPU9150_Result_DeviceNotConnected, /*!< There is no device with valid slave address */
 	KB_MPU9150_Result_DeviceInvalid,       /*!< Connected device with address is not MPU6050 */
  KB_MPU9150_Result_NoMagnetometer,      /*!< Failed to connect the Compass */
  KB_MPU9150_Result_UnkownProblem       /*!< Unkown problem */
} KB_MPU9150_Result_t;

/**
 * @brief  Parameters for accelerometer range
 */
#define KB_MPU9150_Accelerometer_2G   TM_MPU6050_Accelerometer_2G
#define KB_MPU9150_Accelerometer_4G   TM_MPU6050_Accelerometer_4G
#define KB_MPU9150_Accelerometer_8G   TM_MPU6050_Accelerometer_8G
#define KB_MPU9150_Accelerometer_16G  TM_MPU6050_Accelerometer_16G
#define KB_MPU9150_Accelerometer_t    TM_MPU6050_Accelerometer_t


/**
 * @brief  Parameters for gyroscope range
 */
#define KB_MPU9150_Gyroscope_250s   TM_MPU6050_Gyroscope_250s
#define KB_MPU9150_Gyroscope_500s   TM_MPU6050_Gyroscope_500s
#define KB_MPU9150_Gyroscope_1000s   TM_MPU6050_Gyroscope_1000s
#define KB_MPU9150_Gyroscope_2000s  TM_MPU6050_Gyroscope_2000s
#define KB_MPU9150_Gyroscope_t    TM_MPU6050_Gyroscope_t

/**
 * @brief  Settings for CNTL register
 */
typedef enum {
	KB_MPU9150_MAGNET_CNTL_POWER_DOWN	=	0X0,
	KB_MPU9150_MAGNET_CNTL_SINGLE_MEAS	=	0X1,
	KB_MPU9150_MAGNET_CNTL_SELF_TEST	=	0X8,
	KB_MPU9150_MAGNET_CNTL_FUSE_ROM	=	0XF,
} KB_MPU9150_MAGNET_CNTL_t;

/**
 * @brief  Main MPU6050 structure
 */
typedef struct {
	/* Private */
	uint8_t Address;         /*!< I2C address of device. Only for private use */
  uint8_t MagnetAddress;   /*!< I2C Magnetometer address */
	float Gyro_Div;         /*!< Gyroscope corrector from raw data to "degrees/s". Only for private use */
	float Acce_Div;         /*!< Accelerometer corrector from raw data to "g". Only for private use */
	float Magnet_Mult;			 /*!< Magnetometer corrector from raw dato to "uT", Only for private use */
	/* Public */
	float Accelerometer_X; /*!< Accelerometer value X axis */
	float Accelerometer_Y; /*!< Accelerometer value Y axis */
	float Accelerometer_Z; /*!< Accelerometer value Z axis */
	float Gyroscope_X;     /*!< Gyroscope value X axis */
	float Gyroscope_Y;     /*!< Gyroscope value Y axis */
	float Gyroscope_Z;     /*!< Gyroscope value Z axis */
	float Magnetometer_X;  /*!< Magnetometer value X axis */
	float Magnetometer_Y;  /*!< Magnetometer value Y axis */
	float Magnetometer_Z;  /*!< Magnetometer value Z axis */

  int8_t  Magnetometer_Adj_X; /*!< Magnetometer adjust value on X axis */
  int8_t  Magnetometer_Adj_Y; /*!< Magnetometer adjust value on X axis */
  int8_t  Magnetometer_Adj_Z; /*!< Magnetometer adjust value on X axis */

	float Temperature;       /*!< Temperature in degrees */
} KB_MPU9150_t;

/**
 * @}
 */

/**
 * @defgroup KB_MPU9150_Functions
 * @brief    Library Functions
 * @{
 */

/**
 * @brief  Initializes MPU9150 and I2C peripheral
 * @param  *DataStruct: Pointer to empty @ref KB_MPU9150_t structure
 *
 * @param  AccelerometerSensitivity: Set accelerometer sensitivity. This parameter can be a value of @ref KB_MPU9150_Accelerometer_t enumeration
 * @param  GyroscopeSensitivity: Set gyroscope sensitivity. This parameter can be a value of @ref KB_MPU9150_Gyroscope_t enumeration
 * @retval Status:
 *            - KB_MPU9150_Result_t: Everything OK
 *            - Other member: in other cases
 */
KB_MPU9150_Result_t
KB_MPU9150_Init(
  KB_MPU9150_t* DataStruct,
  KB_MPU9150_Accelerometer_t AccelerometerSensitivity,
  KB_MPU9150_Gyroscope_t GyroscopeSensitivity);

/**
 * @brief  Reads accelerometer data from sensor
 * @param  *DataStruct: Pointer to @ref KB_MPU9150_t structure to store data to
 * @retval Member of @ref TM_MPU6050_Result_t:
 *            - TM_MPU6050_Result_Ok: everything is OK
 *            - Other: in other cases
 */
KB_MPU9150_Result_t
KB_MPU9150_ReadAccelerometer(KB_MPU9150_t* DataStruct);

/**
 * @brief  Reads gyroscope data from sensor
 * @param  *DataStruct: Pointer to @ref KB_MPU9150_t structure to store data to
 * @retval Member of @ref KB_MPU9150_Result_t:
 *            - KB_MPU9150_Result_Ok: everything is OK
 *            - Other: in other cases
 */
KB_MPU9150_Result_t
KB_MPU9150_ReadGyroscope(KB_MPU9150_t* DataStruct);

/**
 * @brief  Reads temperature data from sensor
 * @param  *DataStruct: Pointer to @ref KB_MPU9150_t structure to store data to
 * @retval Member of @ref KB_MPU9150_Result_t:
 *            - KB_MPU9150_Result_Ok: everything is OK
 *            - Other: in other cases
 */
KB_MPU9150_Result_t
KB_MPU9150_ReadTemperature(KB_MPU9150_t* DataStruct);

/**
 * @brief  Reads temperature data from sensor
 * @param  *DataStruct: Pointer to @ref KB_MPU9150_t structure to store data to
 * @retval Member of @ref KB_MPU9150_Result_t:
 *            - KB_MPU9150_Result_Ok: everything is OK
 *            - Other: in other cases
 */
KB_MPU9150_Result_t
KB_MPU9150_ReadMagnetometer(KB_MPU9150_t* DataStruct);

/**
 * @brief  Reads accelerometer, gyroscope and temperature data from sensor
 * @param  *DataStruct: Pointer to @ref KB_MPU9150_t structure to store data to
 * @retval Member of @ref KB_MPU9150_Result_t:
 *            - KB_MPU9150_Result_Ok: everything is OK
 *            - Other: in other cases
 */
KB_MPU9150_Result_t
KB_MPU9150_ReadAll(KB_MPU9150_t* DataStruct);

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
