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

/* Private functions */
void KB_MPU9150_CopySettings_MPU6050(KB_MPU9150_t* MPU9150_data, TM_MPU6050_t* MPU6050_data);
void KB_MPU9150_CopyData_MPU6050(KB_MPU9150_t* MPU9150_data, TM_MPU6050_t* MPU6050_data);


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
  status = TM_MPU6050_Init(&MPU6050_data, TM_MPU6050_Device_0, AccelerometerSensitivity, GyroscopeSensitivity);

  switch(status)
  {
    case TM_MPU6050_Result_Ok:
    {
      /* First copy the MPU6050_data to DataStruct */
      KB_MPU9150_CopySettings_MPU6050(DataStruct, &MPU6050_data);
      /* Initialize the magnetometer then */
      // TODO: enable Pass-Through Mode
        //TODO: Set 1 I2C_BYASS_EN (Bit1 of INT_PIN_CFG register, 0x37)
      temp = TM_I2C_Read(MPU6050_I2C, DataStruct->Address, MPU6050_INT_PIN_CFG);
      TM_I2C_Write(MPU6050_I2C, DataStruct->Address, MPU6050_INT_PIN_CFG, (uint8_t)(temp | 0x02));
        //TODO: Set 0 I2C_MST_EN (BIT5 of USER_CTRL register, 0x6A)
      temp = TM_I2C_Read(MPU6050_I2C, DataStruct->Address, MPU6050_USER_CTRL);
      TM_I2C_Write(MPU6050_I2C, DataStruct->Address, MPU6050_USER_CTRL, (uint8_t)(temp & ~0x20));

      // TODO: Try to connect the magnetometer
      // TODO: Check if the magnetometer is valid device using WIA
      DataStruct->MagnetAddress = MPU9150_MAGNET_I2C_ADDR;
      if (!TM_I2C_IsDeviceConnected(MPU6050_I2C, DataStruct->MagnetAddress)) {
    		/* Return error */
    		return KB_MPU9150_Result_NoMagnetometer;
    	}
      /* Set I2C magnetometer address */
      temp = TM_I2C_Read(MPU6050_I2C, DataStruct->MagnetAddress, MPU9150_MAGNET_WIA);
      if(temp != MPU9150_MAGNET_I_AM){
        /* Return error */
        return KB_MPU9150_Result_NoMagnetometer;
      }
      // TODO: Setting CNTL register into Fuse ROM access mode
      TM_I2C_Write(MPU6050_I2C, DataStruct->MagnetAddress, MPU9150_MAGNET_CNTL, KB_MPU9150_MAGNET_CNTL_FUSE_ROM);
      // TODO: Update sensitivity adjustment value
      // TODO: fix it to use multiple read
      DataStruct->Magnetometer_Adj_X = TM_I2C_Read(MPU6050_I2C, DataStruct->MagnetAddress, MPU9150_MAGNET_ASAX);
      DataStruct->Magnetometer_Adj_Y = TM_I2C_Read(MPU6050_I2C, DataStruct->MagnetAddress, MPU9150_MAGNET_ASAY);
      DataStruct->Magnetometer_Adj_Z = TM_I2C_Read(MPU6050_I2C, DataStruct->MagnetAddress, MPU9150_MAGNET_ASAZ);
      // TODO: Setting CNTL register into Single measurement mode
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
  KB_MPU9150_CopyData_MPU6050(DataStruct, &MPU6050_data);

  //TODO: return real value

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
  KB_MPU9150_CopyData_MPU6050(DataStruct, &MPU6050_data);
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
  KB_MPU9150_CopyData_MPU6050(DataStruct, &MPU6050_data);
  return KB_MPU9150_Result_Ok;
}

KB_MPU9150_Result_t
KB_MPU9150_ReadAll(KB_MPU9150_t* DataStruct)
{
  uint8_t data[6];
  TM_MPU6050_t MPU6050_data;
  /* Format I2C address */
  MPU6050_data.Address = MPU6050_I2C_ADDR;

  /* Getting the data from MPU6050 library */
  if(TM_MPU6050_ReadAll(&MPU6050_data) != TM_MPU6050_Result_Ok){
    return KB_MPU9150_Result_UnkownProblem;
  }

  /* Copy the values */
  KB_MPU9150_CopyData_MPU6050(DataStruct, &MPU6050_data);

  /* Set the compass into single measurement mode */
  TM_I2C_Write(MPU6050_I2C, DataStruct->MagnetAddress, MPU9150_MAGNET_CNTL, KB_MPU9150_MAGNET_CNTL_SINGLE_MEAS);
  /* Get the magnetometer data */

  // TODO: Read magnet data
  TM_I2C_ReadMulti(MPU6050_I2C, DataStruct->MagnetAddress, MPU9150_MAGNET_HXL, data, 6);
  DataStruct->Magnetometer_X = (int16_t)(data[1] << 8 | data[0]);
  DataStruct->Magnetometer_Y = (int16_t)(data[3] << 8 | data[2]);
  DataStruct->Magnetometer_Z = (int16_t)(data[5] << 8 | data[4]);

  // TODO: calibrate the data
  DataStruct->Magnetometer_X = MPU9150_MAGNET_ADJ(DataStruct->Magnetometer_X, DataStruct->Magnetometer_Adj_X);
  DataStruct->Magnetometer_Y = MPU9150_MAGNET_ADJ(DataStruct->Magnetometer_Y, DataStruct->Magnetometer_Adj_Y);
  DataStruct->Magnetometer_Z = MPU9150_MAGNET_ADJ(DataStruct->Magnetometer_Z, DataStruct->Magnetometer_Adj_Z);

  return KB_MPU9150_Result_Ok;
}

/* Private Functions */
void
KB_MPU9150_CopySettings_MPU6050(
  KB_MPU9150_t* MPU9150_data,
  TM_MPU6050_t* MPU6050_data)
{
  /* Private */
  MPU9150_data->Address   = MPU6050_data->Address;         /*!< I2C address of device. Only for private use */
  MPU9150_data->Gyro_Mult = MPU6050_data->Gyro_Mult;         /*!< Gyroscope corrector from raw data to "degrees/s". Only for private use */
  MPU9150_data->Acce_Mult = MPU6050_data->Acce_Mult;         /*!< Accelerometer corrector from raw data to "g". Only for private use */
}

void
KB_MPU9150_CopyData_MPU6050(
  KB_MPU9150_t* MPU9150_data,
  TM_MPU6050_t* MPU6050_data)
{
  /* Public */
  MPU9150_data->Accelerometer_X = MPU6050_data->Accelerometer_X; /*!< Accelerometer value X axis */
  MPU9150_data->Accelerometer_Y = MPU6050_data->Accelerometer_Y; /*!< Accelerometer value Y axis */
  MPU9150_data->Accelerometer_Z = MPU6050_data->Accelerometer_Z; /*!< Accelerometer value Z axis */
  MPU9150_data->Gyroscope_X = MPU6050_data->Gyroscope_X;     /*!< Gyroscope value X axis */
  MPU9150_data->Gyroscope_Y = MPU6050_data->Gyroscope_Y;     /*!< Gyroscope value Y axis */
  MPU9150_data->Gyroscope_Z = MPU6050_data->Gyroscope_Z;     /*!< Gyroscope value Z axis */
  MPU9150_data->Temperature = MPU6050_data->Temperature;       /*!< Temperature in degrees */
}
