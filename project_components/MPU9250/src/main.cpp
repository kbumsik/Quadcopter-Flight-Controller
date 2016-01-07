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

/* Custom macro */
#define wait(NUM) vTaskDelay(NUM / portTICK_RATE_MS)

// Using NOKIA 5110 monochrome 84 x 48 pixel display
// pin 9 - Serial clock out (SCLK)
// pin 8 - Serial data out (DIN)
// pin 7 - Data/Command select (D/C)
// pin 5 - LCD chip select (CS)
// pin 6 - LCD reset (RST)
//Adafruit_PCD8544 display = Adafruit_PCD8544(9, 8, 7, 5, 6);

float sum = 0;
uint32_t sumCount = 0;

/* custom code */
/* Queue Handlers */
TaskHandle_t xUpdateMPU9250Handle;
QueueHandle_t quUARTReceive;

I2C_HandleTypeDef hi2c3;

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
  /* definition and creation of defaultTask */
  result = xTaskCreate(vUpdateMPU9250, /* Pointer to the function that implements the task */
  "MPU9250", /* Text name for the task. This is to facilitate debugging only. It is not used in the scheduler */
  configMINIMAL_STACK_SIZE+ 2000, /* Stack depth in words */
  NULL, /* Pointer to a task parameters */
  1, /* The task priority */
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

void vUpdateMPU9250(void *pvParameters)
{
  uint8_t aTxBuffer[2];

  MPU9250 mpu9250;


  //Set up I2C
  i2c.frequency(400000);  // use fast (400 kHz) I2C
  printf("CPU SystemCoreClock is %d Hz\r\n", SystemCoreClock);

  hi2c3.Instance = I2C3;

  aTxBuffer[0] = 0x7;
  aTxBuffer[1] = 0x5;
  //HAL_I2C_Master_Transmit(&hi2c3, (uint16_t)MPU9250_ADDRESS, (uint8_t*)aTxBuffer, 2, 10000);


  // Read the WHO_AM_I register, this is a good test of communication
  uint8_t whoami = mpu9250.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250); // Read WHO_AM_I register for MPU-9250
  printf("I AM 0x%x\n\r", whoami);
  printf("I SHOULD BE 0x71\n\r");

  if (whoami == 0x71) // WHO_AM_I should always be 0x68
  {
    printf("MPU9250 is online...\n\r");
    wait(1);

    mpu9250.resetMPU9250(); // Reset registers to default in preparation for device calibration
    mpu9250.calibrateMPU9250(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers  
    printf("x gyro bias = %f\n\r", gyroBias[0]);
    printf("y gyro bias = %f\n\r", gyroBias[1]);
    printf("z gyro bias = %f\n\r", gyroBias[2]);
    printf("x accel bias = %f\n\r", accelBias[0]);
    printf("y accel bias = %f\n\r", accelBias[1]);
    printf("z accel bias = %f\n\r", accelBias[2]);
    wait(2);
    mpu9250.initMPU9250();
    printf("MPU9250 initialized for active data mode....\n\r"); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
    mpu9250.initAK8963(magCalibration);
    printf("AK8963 initialized for active data mode....\n\r"); // Initialize device for active mode read of magnetometer
    printf("Accelerometer full-scale range = %f  g\n\r",
        2.0f * (float) (1 << Ascale));
    printf("Gyroscope full-scale range = %f  deg/s\n\r",
        250.0f * (float) (1 << Gscale));
    if (Mscale == 0)
      printf("Magnetometer resolution = 14  bits\n\r");
    if (Mscale == 1)
      printf("Magnetometer resolution = 16  bits\n\r");
    if (Mmode == 2)
      printf("Magnetometer ODR = 8 Hz\n\r");
    if (Mmode == 6)
      printf("Magnetometer ODR = 100 Hz\n\r");
    wait(2);
  }
  else
  {
    printf("Could not connect to MPU9250: \n\r");
    printf("%#x \n", whoami);

    printf("MPU9250");
    printf("no connection");
    printf("0x\r\n");

    while (1)
      ; // Loop forever if communication doesn't happen
  }

  mpu9250.getAres(); // Get accelerometer sensitivity
  mpu9250.getGres(); // Get gyro sensitivity
  mpu9250.getMres(); // Get magnetometer sensitivity
  printf("Accelerometer sensitivity is %f LSB/g \n\r", 1.0f / aRes);
  printf("Gyroscope sensitivity is %f LSB/deg/s \n\r", 1.0f / gRes);
  printf("Magnetometer sensitivity is %f LSB/G \n\r", 1.0f / mRes);
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

      printf("ax = %f", 1000 * ax);
      printf(" ay = %f", 1000 * ay);
      printf(" az = %f  mg\n\r", 1000 * az);

      printf("gx = %f", gx);
      printf(" gy = %f", gy);
      printf(" gz = %f  deg/s\n\r", gz);

      printf("gx = %f", mx);
      printf(" gy = %f", my);
      printf(" gz = %f  mG\n\r", mz);

      tempCount = mpu9250.readTempData();  // Read the adc values
      temperature = ((float) tempCount) / 333.87f + 21.0f; // Temperature in degrees Centigrade
      printf(" temperature = %f  C\n\r", temperature);

      printf("q0 = %f\n\r", q[0]);
      printf("q1 = %f\n\r", q[1]);
      printf("q2 = %f\n\r", q[2]);
      printf("q3 = %f\n\r", q[3]);

      printf("MPU9250");
      printf("x   y   z");
      printf("%f", (char) (1000 * ax));
      printf("%f", (char) (1000 * ay));
      printf("%f", (char) (1000 * az));
      printf("mg");

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

      printf("Yaw, Pitch, Roll: %f %f %f\n\r", yaw, pitch, roll);
      printf("average rate = %f\n\r", (float) sumCount / sum);

      count = uwTimerGetMillis();
      sum = 0;
      sumCount = 0;
    }
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
   ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif
