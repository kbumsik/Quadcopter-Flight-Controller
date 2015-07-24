/**
 * |----------------------------------------------------------------------
 * | Copyright (C) Tilen Majerle, 2014
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


#include "kb_stm32f4_mpu9150.h"

/* type definitions for private functions */
typedef enum {
	KB_Accelermoter,
	KB_Gyroscope,
	KB_Temperature
} KB_CopyType;

/* Private functions */
void KB_MPU9150_CopyData_MPU6050(KB_MPU9150_t* MPU9150_data, TM_MPU6050_t* MPU6050_data, KB_CopyType copy_type);




KB_MPU9150_Result_t
KB_MPU9150_Init(
	KB_MPU9150_t* DataStruct,
	KB_MPU9150_Accelerometer_t AccelerometerSensitivity,
	KB_MPU9150_Gyroscope_t GyroscopeSensitivity)
{
	uint8_t temp;
	TM_MPU6050_Result_t status;
	TM_MPU6050_t MPU6050_data;
	/* Initialize the device aside from the compass */
	// FIXME: returns invalid.
	status = TM_MPU6050_Init(&MPU6050_data, TM_MPU6050_Device_0, AccelerometerSensitivity, GyroscopeSensitivity);

	/* Set sensitivities for multiplying gyro and accelerometer data */
	switch (AccelerometerSensitivity) {
		case KB_MPU9150_Accelerometer_2G:
			DataStruct->Acce_Div = MPU6050_ACCE_SENS_2;
			break;
		case KB_MPU9150_Accelerometer_4G:
			DataStruct->Acce_Div = MPU6050_ACCE_SENS_4;
			break;
		case KB_MPU9150_Accelerometer_8G:
			DataStruct->Acce_Div = MPU6050_ACCE_SENS_8;
			break;
		case KB_MPU9150_Accelerometer_16G:
			DataStruct->Acce_Div = MPU6050_ACCE_SENS_16;
			break;
		default:
			break;
	}

	switch (GyroscopeSensitivity) {
		case KB_MPU9150_Gyroscope_250s:
			DataStruct->Gyro_Div = MPU6050_GYRO_SENS_250;
			break;
		case KB_MPU9150_Gyroscope_500s:
			DataStruct->Gyro_Div = MPU6050_GYRO_SENS_500;
			break;
		case KB_MPU9150_Gyroscope_1000s:
			DataStruct->Gyro_Div = MPU6050_GYRO_SENS_1000;
			break;
		case KB_MPU9150_Gyroscope_2000s:
			DataStruct->Gyro_Div = MPU6050_GYRO_SENS_2000;
			break;
		default:
			break;
	}

	/* Copy address */
	DataStruct->Address = MPU6050_data.Address;

	/* act according to the status of MPU6050 connection */
	switch(status)
	{
		case TM_MPU6050_Result_Ok:
		{
			/* Initialize the magnetometer then */
			/* enable Pass-Through Mode */
				/* Set 1 I2C_BYASS_EN (Bit1 of INT_PIN_CFG register, 0x37) */
			TM_I2C_Read(MPU6050_I2C, DataStruct->Address, MPU6050_INT_PIN_CFG, &temp);
			TM_I2C_Write(MPU6050_I2C, DataStruct->Address, MPU6050_INT_PIN_CFG, (uint8_t)(temp | 0x02));
				/* Set 0 I2C_MST_EN (BIT5 of USER_CTRL register, 0x6A) */
			TM_I2C_Read(MPU6050_I2C, DataStruct->Address, MPU6050_USER_CTRL, &temp);
			TM_I2C_Write(MPU6050_I2C, DataStruct->Address, MPU6050_USER_CTRL, (uint8_t)(temp & ~0x20));

			/* Try to connect the magnetometer */
			/* Check if the magnetometer is valid device using WIA */
			DataStruct->MagnetAddress = MPU9150_MAGNET_I2C_ADDR;
			if (!TM_I2C_IsDeviceConnected(MPU6050_I2C, DataStruct->MagnetAddress)) {
				/* Return error */
				return KB_MPU9150_Result_NoMagnetometer;
			}
			/* Set I2C magnetometer address */
			TM_I2C_Read(MPU6050_I2C, DataStruct->MagnetAddress, MPU9150_MAGNET_WIA, &temp);
			if(temp != MPU9150_MAGNET_I_AM){
				/* Return error */
				return KB_MPU9150_Result_NoMagnetometer;
			}
			/* Setting CNTL register into Fuse ROM access mode */
			TM_I2C_Write(MPU6050_I2C, DataStruct->MagnetAddress, MPU9150_MAGNET_CNTL, KB_MPU9150_MAGNET_CNTL_FUSE_ROM);
			/* Update sensitivity adjustment value */
			/* fix it to use multiple read */
			TM_I2C_Read(MPU6050_I2C, DataStruct->MagnetAddress, MPU9150_MAGNET_ASAX, &DataStruct->Magnetometer_Adj_X);
			TM_I2C_Read(MPU6050_I2C, DataStruct->MagnetAddress, MPU9150_MAGNET_ASAY, &DataStruct->Magnetometer_Adj_Y);
			TM_I2C_Read(MPU6050_I2C, DataStruct->MagnetAddress, MPU9150_MAGNET_ASAZ, &DataStruct->Magnetometer_Adj_Z);
			/* Setting CNTL register into Single measurement mode */
			TM_I2C_Write(MPU6050_I2C, DataStruct->MagnetAddress, MPU9150_MAGNET_CNTL, KB_MPU9150_MAGNET_CNTL_SINGLE_MEAS);
			/* Update data */
			DataStruct->Magnet_Mult = MPU9150_MAGNET_SENS;

			/* Return OK */
			return KB_MPU9150_Result_Ok;
		}
		case TM_MPU6050_Result_DeviceNotConnected:
		{
			return KB_MPU9150_Result_DeviceNotConnected;
		}
		case TM_MPU6050_Result_DeviceInvalid:
		{
			return KB_MPU9150_Result_DeviceInvalid;
		}
	}
	/* If status value is invalid, just return unkown */
	return KB_MPU9150_Result_UnkownProblem;
}

KB_MPU9150_Result_t
KB_MPU9150_ReadAccelerometer(KB_MPU9150_t* DataStruct)
{
	TM_MPU6050_t MPU6050_data;
	/* Format I2C address */
	MPU6050_data.Address = MPU6050_I2C_ADDR;

	/* Getting the data from MPU6050 library */
	if(TM_MPU6050_ReadAccelerometer(&MPU6050_data) != TM_MPU6050_Result_Ok){
		return KB_MPU9150_Result_UnkownProblem;
	}

	/* Copy the values */
	KB_MPU9150_CopyData_MPU6050(DataStruct, &MPU6050_data, KB_Accelermoter);

	/* return real value */

	return KB_MPU9150_Result_Ok;
}

KB_MPU9150_Result_t
KB_MPU9150_ReadGyroscope(KB_MPU9150_t* DataStruct)
{
	TM_MPU6050_t MPU6050_data;
	/* Format I2C address */
	MPU6050_data.Address = MPU6050_I2C_ADDR;

	/* Getting the data from MPU6050 library */
	if(TM_MPU6050_ReadGyroscope(&MPU6050_data) != TM_MPU6050_Result_Ok){
		return KB_MPU9150_Result_UnkownProblem;
	}

	/* Copy the values */
	KB_MPU9150_CopyData_MPU6050(DataStruct, &MPU6050_data, KB_Gyroscope);
	return KB_MPU9150_Result_Ok;
}

KB_MPU9150_Result_t
KB_MPU9150_ReadTemperature(KB_MPU9150_t* DataStruct)
{
	TM_MPU6050_t MPU6050_data;
	/* Format I2C address */
	MPU6050_data.Address = MPU6050_I2C_ADDR;

	/* Getting the data from MPU6050 library */
	if(TM_MPU6050_ReadTemperature(&MPU6050_data) != TM_MPU6050_Result_Ok){
		return KB_MPU9150_Result_UnkownProblem;
	}

	/* Copy the values */
	KB_MPU9150_CopyData_MPU6050(DataStruct, &MPU6050_data, KB_Temperature);
	return KB_MPU9150_Result_Ok;
}

KB_MPU9150_Result_t
KB_MPU9150_ReadMagnetometer(KB_MPU9150_t* DataStruct)
{
	uint8_t buffer[6];
	int16_t x,y,z;
	/* Set the compass into single measurement mode */
	TM_I2C_Write(MPU6050_I2C, DataStruct->MagnetAddress, MPU9150_MAGNET_CNTL, KB_MPU9150_MAGNET_CNTL_SINGLE_MEAS);
	if (!TM_I2C_IsDeviceConnected(MPU6050_I2C, DataStruct->MagnetAddress)) {
			/* Return error */
			return KB_MPU9150_Result_NoMagnetometer;
	}
	/* Get the magnetometer data */

	/* Read magnet data */
	TM_I2C_ReadMulti(MPU6050_I2C, DataStruct->MagnetAddress, MPU9150_MAGNET_HXL, buffer, 6);

	x = (int16_t)(buffer[1] << 8 | buffer[0]);
	DataStruct->Magnetometer_X = (float)MPU9150_MAGNET_ADJ(x, DataStruct->Magnetometer_Adj_X);

	y = (int16_t)(buffer[3] << 8 | buffer[2]);
	DataStruct->Magnetometer_Y = (float)MPU9150_MAGNET_ADJ(y, DataStruct->Magnetometer_Adj_Y);

	z = (int16_t)(buffer[5] << 8 | buffer[4]);
	DataStruct->Magnetometer_Z = (float)MPU9150_MAGNET_ADJ(z, DataStruct->Magnetometer_Adj_Z);



	/* calibrate the data */
	DataStruct->Magnetometer_X *= DataStruct->Magnet_Mult;
	DataStruct->Magnetometer_Y *= DataStruct->Magnet_Mult;
	DataStruct->Magnetometer_Z *= DataStruct->Magnet_Mult;


	return KB_MPU9150_Result_Ok;
}

KB_MPU9150_Result_t
KB_MPU9150_ReadAll(KB_MPU9150_t* DataStruct)
{
	TM_MPU6050_t MPU6050_data;
	/* Format I2C address */
	MPU6050_data.Address = MPU6050_I2C_ADDR;

	/* Getting the data from MPU6050 library */
	if(TM_MPU6050_ReadAll(&MPU6050_data) != TM_MPU6050_Result_Ok){
		return KB_MPU9150_Result_UnkownProblem;
	}

	/* Copy the values */
	KB_MPU9150_CopyData_MPU6050(DataStruct, &MPU6050_data, KB_Accelermoter);
	KB_MPU9150_CopyData_MPU6050(DataStruct, &MPU6050_data, KB_Gyroscope);
	KB_MPU9150_CopyData_MPU6050(DataStruct, &MPU6050_data, KB_Temperature);

	/* Read the compass */
	KB_MPU9150_ReadMagnetometer(DataStruct);

	return KB_MPU9150_Result_Ok;
}

/* Private Functions */
void
KB_MPU9150_CopyData_MPU6050(
	KB_MPU9150_t* MPU9150_data,
	TM_MPU6050_t* MPU6050_data,
	KB_CopyType copy_type)
{
	switch(copy_type)
	{
		case KB_Accelermoter:
		MPU9150_data->Accelerometer_X = (float)MPU6050_data->Accelerometer_X / MPU9150_data->Acce_Div; /*!< Accelerometer value X axis */
		MPU9150_data->Accelerometer_Y = (float)MPU6050_data->Accelerometer_Y / MPU9150_data->Acce_Div; /*!< Accelerometer value Y axis */
		MPU9150_data->Accelerometer_Z = (float)MPU6050_data->Accelerometer_Z / MPU9150_data->Acce_Div; /*!< Accelerometer value Z axis */
		break;
		case KB_Gyroscope:
		MPU9150_data->Gyroscope_X = (float)MPU6050_data->Gyroscope_X / MPU9150_data->Gyro_Div;     /*!< Gyroscope value X axis */
		MPU9150_data->Gyroscope_Y = (float)MPU6050_data->Gyroscope_Y / MPU9150_data->Gyro_Div;     /*!< Gyroscope value Y axis */
		MPU9150_data->Gyroscope_Z = (float)MPU6050_data->Gyroscope_Z / MPU9150_data->Gyro_Div;     /*!< Gyroscope value Z axis */
		break;
		case KB_Temperature:
		MPU9150_data->Temperature = MPU6050_data->Temperature;       /*!< Temperature in degrees */
		break;
	}
}
