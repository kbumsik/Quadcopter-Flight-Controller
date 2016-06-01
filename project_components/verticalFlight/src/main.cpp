/* MPU9250 Basic Example Code
 by: Kris Winer
 date: April 1, 2014
 license: Beerware - Use this code however you'd like. If you 
 find it useful you can buy me a beer some time.
 
 Demonstrate basic MPU-9250 functionality including parameterizing the register addresses, initializing the sensor, 
 getting properly scaled accelerometer, gyroscope, and magnetometer data out. Added display functions to 
 allow display to on breadboard monitor. Addition of 9 DoF sensor fusion using open source Madgwick and 
 Mahony filter algorithms. Sketch runs on the 3.3 V 8 MHz Pro Mini and the Teensy 3.1.
 
 SDA and SCL should have external pull-up resistors (to 3.3V).
 10k resistors are on the EMSENSR-9250 breakout board.
 
 Hardware setup:
 MPU9250 Breakout --------- Arduino
 VDD ---------------------- 3.3V
 VDDI --------------------- 3.3V
 SDA ----------------------- A4
 SCL ----------------------- A5
 GND ---------------------- GND
 
 Note: The MPU9250 is an I2C sensor and uses the Arduino Wire library. 
 Because the sensor is not 5V tolerant, we are using a 3.3 V 8 MHz Pro Mini or a 3.3 V Teensy 3.1.
 We have disabled the internal pull-ups used by the Wire library in the Wire.h/twi.c utility file.
 We are also using the 400 kHz fast I2C mode by setting the TWI_FREQ  to 400000L /twi.h utility file.
 */
 
//#include "ST_F401_84MHZ.h" 
//F401_init84 myinit(0);
#include "config.h"
#include "cmsis_os.h"
#include "board_init.h"
#include "MPU9250.h"
#include "main.h"
#include "control.h"

/* Custom macro */
#define wait(NUM)  vTaskDelay((uint32_t) (NUM * 1000) / portTICK_RATE_MS)

float sum = 0;
uint32_t sumCount = 0;

/* external variables */
float pitch, yaw, roll; /* Located in MPU9250.h */

/* Private variables ---------------------------------------------------------*/
/* Task Handlers */
TaskHandle_t xBlinkyHandle;
TaskHandle_t xMotorSpeedHandle;
TaskHandle_t xRemoteScanHandle;
TaskHandle_t xUpdateMPU9250Handle;
/* Queue Handlers */
QueueHandle_t quUARTReceive;

/* Mutex Handlers */
SemaphoreHandle_t muRemote;
SemaphoreHandle_t muIMU;

/* pid_t objects */
controlPid_t xRollPID;
controlPid_t xPitchPID;

I2C_HandleTypeDef hi2c3;

void vBlinkyTask(void *pvParameters);
void vMotorSpeedTask(void *pvParameters);
void vRemoteScanTask(void *pvParameters);
void vUpdateMPU9250(void *pvParameters);

int main()
{
  BaseType_t result;
  /* Custom Code start */
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  quadcopter_Init();

  /* Create Queue */
  quUARTReceive = xQueueCreate(confUART_RECEIVE_QUEUE_LENGTH, /* length of queue */
  sizeof(uint8_t)*confUART_RECEIVE_BUFFER_SIZE); /* size in byte of each item */

  /* USER CODE BEGIN RTOS_MUTEX */
    muRemote = xSemaphoreCreateMutex();
    muIMU = xSemaphoreCreateMutex();
  /* USER CODE END RTOS_MUTEX */

  vControlPIDSetTunings(3, 0.1, 0.1, &xPitchPID);
  vControlPIDSetTunings(3, 0.1, 0.1, &xRollPID);

  /* definition and creation of defaultTask */
    result = xTaskCreate(vBlinkyTask,                  /* Pointer to the function that implements the task */
                  "Blinky",                     /* Text name for the task. This is to facilitate debugging only. It is not used in the scheduler */
                  configMINIMAL_STACK_SIZE,     /* Stack depth in words */
                  NULL,                         /* Pointer to a task parameters */
                  1,                            /* The task priority */
                  &xBlinkyHandle);                        /* Pointer of its task handler, if you don't want to use, you can leave it NULL */

    if (result != pdPASS)
      {
        vLED_0_On();
        while (1)
          ;
      }

  result = xTaskCreate(vMotorSpeedTask,
                  "Scan",
                  configMINIMAL_STACK_SIZE+500,
                  NULL,
                  configMAX_PRIORITIES-1,
                  &xMotorSpeedHandle);

  if (result != pdPASS)
    {
      vLED_0_On();
      while (1)
        ;
    }

  result = xTaskCreate(vRemoteScanTask,
                  "Remote",
                  configMINIMAL_STACK_SIZE+500,
                  NULL,
                  configMAX_PRIORITIES-3,
                  &xRemoteScanHandle);
  if (result != pdPASS)
    {
      vLED_0_On();
      while (1)
        ;
    }
  result = xTaskCreate(vUpdateMPU9250, /* Pointer to the function that implements the task */
                      "MPU9250", /* Text name for the task. This is to facilitate debugging only. It is not used in the scheduler */
                      configMINIMAL_STACK_SIZE+ 1000, /* Stack depth in words */
                      NULL, /* Pointer to a task parameters */
                      configMAX_PRIORITIES-2, /* The task priority */
                      &xUpdateMPU9250Handle); /* Pointer of its task handler, if you don't want to use, you can leave it NULL */

  if (result != pdPASS)
  {
    vLED_0_On();
    while (1)
      ;
  }
  /* Custom Code End */

  /* Start scheduler */
  vTaskStartScheduler();
  /* NOTE: We should never get here as control is now taken by the scheduler */
  while (1)
  {
  }
}

/* vBlinkyTask function */
void vBlinkyTask(void *pvParameters)
{
  portTickType xLastWakeTime;
  /* Initialize xLastWakeTime for vTaskDelayUntil */
  /* This variable is updated every vTaskDelayUntil is called */
  xLastWakeTime = xTaskGetTickCount();

  for(;;)
    {
      vLED_0_Toggle();
      /* Call this Task explicitly every 50ms ,NOT Delay for 50ms */
      vTaskDelayUntil(&xLastWakeTime, (200/portTICK_RATE_MS));
    }

  /* It never goes here, but the task should be deleted when it reached here */
  vTaskDelete(NULL);
}

/* vMotorSpeedTask Task function */
void vMotorSpeedTask(void *pvParameters)
{
  int i = 0;
  portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  eMotorChannel_t eMotorChannel = eMOTOR_CHANNEL_1;
  /* Update the speed of motor */
  swMotorSetSpeed(&xMotorHandle, 1500, eMOTOR_CHANNEL_ALL);

  /* Then start the pwm signal */
  eMotorStart(&xMotorHandle, eMOTOR_CHANNEL_ALL);

  float fRowControl;
  float fPitchControl;

  for (i = 0; i < confREMOTE_NUMBER_OF_CHANNEL; i++)
  {
    pfMotorSpeed[i] = motorSPEED_MID;
  }

  /* Infinite loop */
  for (;;)
  {
    /* calculate PID */
    xSemaphoreTake(muIMU, portMAX_DELAY); /* Take Mutext */
    fPitchControl = fControlPIDCompute(0, pitch, &xPitchPID);
    fRowControl = fControlPIDCompute(0, roll, &xRollPID);
    xSemaphoreGive(muIMU); /* Release mutex */

    pfMotorSpeed[0] = pfMotorSpeed[0] + fPitchControl;
    pfMotorSpeed[1] = pfMotorSpeed[1] - fPitchControl;
    pfMotorSpeed[2] = pfMotorSpeed[2] + fRowControl;
    pfMotorSpeed[3] = pfMotorSpeed[3] - fRowControl;

    /* Start updating the speed of motor */
    /* get remote value */
    xSemaphoreTake(muRemote, portMAX_DELAY); /* Take Mutext */
    for (i = 0; i < confREMOTE_NUMBER_OF_CHANNEL; i++)
    {
      pfMotorSpeed[i] = pfMotorSpeed[i] + xControlMinusAndScale(pfRemote[1],
          motorSPEED_MID, pfRemoteToMotorScale[1])/pfRemoteToMotorScale[1];
    }
    xSemaphoreGive(muRemote); /* Release mutex */

    /* Limit the speed (Don't really needed here though */
    for (i = 0; i < confREMOTE_NUMBER_OF_CHANNEL; i++)
    {
      pfMotorSpeed[i] = xControlLimitter(pfMotorSpeed[i], motorSPEED_MIN,
          motorSPEED_MAX);
    }
    /* Finally update the motor speed */
    eMotorChannel = eMOTOR_CHANNEL_1;
    for (i = 0; i < confREMOTE_NUMBER_OF_CHANNEL; i++)
    {
      swMotorSetSpeed(&xMotorHandle, (int32_t) pfMotorSpeed[i], eMotorChannel);
      eMotorChannel = (eMotorChannel_t)(eMotorChannel + 1);
    }
    eMotorStart(&xMotorHandle, eMOTOR_CHANNEL_ALL); /* Start PWM */
    /* End updating the speed of motor */
    /* Do this every 200ms */
    vTaskDelayUntil(&xLastWakeTime, 200 / portTICK_RATE_MS);
  }

  /* It never goes here, but the task should be deleted when it reached here */
  vTaskDelete(NULL);
}

/* vMotorSpeedTask Task function */
void vRemoteScanTask(void *pvParameters)
{
  portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  int i = 0;
  while (1)
  {

    /* Start Update Remote controller */
    /* get Remote value and scale */
    xSemaphoreTake(muRemote, portMAX_DELAY); /* Take Mutext */
    for (i = 0; i < confREMOTE_NUMBER_OF_CHANNEL; i++)
    {
      pfRemote[i] = xControlLimitter(uwPWMInputDutyCycle(pxRemoteHandle[i]),
          ppuwRemoteRange[i][confREMOTE_MIN], ppuwRemoteRange[i][confREMOTE_MAX]);
    }
    for (i = 0; i < confREMOTE_NUMBER_OF_CHANNEL; i++)
    {
      pfRemote[i] = (float32_t)xControlMinusAndScale(pfRemote[i],
                     ppuwRemoteRange[i][confREMOTE_CENTER], pfRemoteScale[i]);
    }
    /* End Update remote controller */
    xSemaphoreGive(muRemote); /* Release mutex */
    /* Do this every 100ms */
    vTaskDelayUntil(&xLastWakeTime, 100 / portTICK_RATE_MS);
  }
}


void vUpdateMPU9250(void *pvParameters)
{
  portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  uint8_t aTxBuffer[2];

  MPU9250 mpu9250;


  //Set up I2C
  i2c.frequency(400000);  // use fast (400 kHz) I2C

  hi2c3.Instance = I2C3;

  aTxBuffer[0] = 0x7;
  aTxBuffer[1] = 0x5;
  //HAL_I2C_Master_Transmit(&hi2c3, (uint16_t)MPU9250_ADDRESS, (uint8_t*)aTxBuffer, 2, 10000);


  // Read the WHO_AM_I register, this is a good test of communication
  uint8_t whoami = mpu9250.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250); // Read WHO_AM_I register for MPU-9250

  if (whoami == 0x71) // WHO_AM_I should always be 0x68
  {
    wait(1);

    mpu9250.resetMPU9250(); // Reset registers to default in preparation for device calibration
    mpu9250.calibrateMPU9250(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers  
    wait(2);
    mpu9250.initMPU9250();
    mpu9250.initAK8963(magCalibration);
    wait(2);
  }
  else
  {


    while (1)
      ; // Loop forever if communication doesn't happen
  }

  mpu9250.getAres(); // Get accelerometer sensitivity
  mpu9250.getGres(); // Get gyro sensitivity
  mpu9250.getMres(); // Get magnetometer sensitivity
  magbias[0] = +470.; // User environmental x-axis correction in milliGauss, should be automatically calculated
  magbias[1] = +120.;  // User environmental x-axis correction in milliGauss
  magbias[2] = +125.;  // User environmental x-axis correction in milliGauss

  while (1)
  {
    // If intPin goes high, all data registers have new data
    if (mpu9250.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
    {  // On interrupt, check if data ready interrupt

      mpu9250.readAccelData(accelCount);  // Read the x/y/z adc values
      // Now we'll calculate the accleration value into actual g's
      ax = (float) accelCount[0] * aRes - accelBias[0]; // get actual g value, this depends on scale being set
      ay = (float) accelCount[1] * aRes - accelBias[1];
      az = (float) accelCount[2] * aRes - accelBias[2];

      mpu9250.readGyroData(gyroCount);  // Read the x/y/z adc values
      // Calculate the gyro value into actual degrees per second
      gx = (float) gyroCount[0] * gRes - gyroBias[0]; // get actual gyro value, this depends on scale being set
      gy = (float) gyroCount[1] * gRes - gyroBias[1];
      gz = (float) gyroCount[2] * gRes - gyroBias[2];

      mpu9250.readMagData(magCount);  // Read the x/y/z adc values
      // Calculate the magnetometer values in milliGauss
      // Include factory calibration per data sheet and user environmental corrections
      mx = (float) magCount[0] * mRes * magCalibration[0] - magbias[0]; // get actual magnetometer value, this depends on scale being set
      my = (float) magCount[1] * mRes * magCalibration[1] - magbias[1];
      mz = (float) magCount[2] * mRes * magCalibration[2] - magbias[2];
    }

    Now = uwTimerGetMicros();
    deltat = (float) ((Now - lastUpdate) / 1000000.0f); // set integration time by time elapsed since last filter update
    lastUpdate = Now;

    sum += deltat;
    sumCount++;

//    if(lastUpdate - firstUpdate > 10000000.0f) {
//     beta = 0.04;  // decrease filter gain after stabilized
//     zeta = 0.015; // increasey bias drift gain after stabilized
    //   }

    // Pass gyro rate as rad/s
    mpu9250.MadgwickQuaternionUpdate(ax, ay, az, gx * PI / 180.0f,
        gy * PI / 180.0f, gz * PI / 180.0f, my, mx, mz);
    // mpu9250.MahonyQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, my, mx, mz);

    // Serial print and/or display at 0.5 s rate independent of data rates
    delt_t = uwTimerGetMillis() - count;
    if (delt_t > 500)
    { // update LCD once per half-second independent of read rate




      tempCount = mpu9250.readTempData();  // Read the adc values
      temperature = ((float) tempCount) / 333.87f + 21.0f; // Temperature in degrees Centigrade

      xSemaphoreTake(muIMU, portMAX_DELAY); /* Take Mutext */
      // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
      // In this coordinate system, the positive z-axis is down toward Earth.
      // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
      // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
      // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
      // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
      // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
      // applied in the correct order which for this configuration is yaw, pitch, and then roll.
      // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
      yaw = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]),
          q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
      pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
      roll = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]),
          q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
      pitch *= 180.0f / PI;
      yaw *= 180.0f / PI;
      yaw -= 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
      roll *= 180.0f / PI;
      xSemaphoreGive(muIMU); /* Release mutex */


      count = uwTimerGetMillis();
      sum = 0;
      sumCount = 0;
    }
    /* Do this every 100ms */
    vTaskDelayUntil(&xLastWakeTime, 100 / portTICK_RATE_MS);
  }
}

#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
  /* USER CODE END 6 */

}

#endif
