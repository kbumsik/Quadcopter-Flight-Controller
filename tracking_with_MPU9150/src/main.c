/**
 *	Keil project example for MPU6050 6-axes
 *
 *	Before you start, select your target, on the right of the "Load" button
 *
 *	@author		Tilen Majerle
 *	@email		tilen@majerle.eu
 *	@website	http://stm32f4-discovery.com
 *	@ide		Keil uVision 5
 *	@packs		STM32F4xx Keil packs version 2.2.0 or greater required
 *	@stdperiph	STM32F4xx Standard peripheral drivers version 1.4.0 or greater required
 */
/* Include core modules */
#include "stm32f4xx.h"
/* Include my libraries here */
#include "defines.h"
#include "tm_stm32f4_delay.h"
#include "tm_stm32f4_usart.h"
#include "tm_stm32f4_mpu6050.h"

#include <stdio.h>
#include <math.h>

/* some constants */
//acceleration due to the gravity in new york. Info is from the internet.
const float g = 9.803;
//LSB Sensitivity of acceleration
const float accel_sens = 4096;

/* parameters that we need to keep track of. all these are with respect to inertia frame*/
//six degrees of freedom
static float x=0, y=0, z=0, a=0, b=0, c=0;
//velocity and angular velocity
static float x_v=0, y_v=0, z_v=0, a_v=0, b_v=0, c_v=0;

/*some temporary placeholder*/
//array of float with 3 elem
static float float_arr_3[3];


/**
 * convert the raw data from MPU to acceleration we are familiar with.
 * this is with respect to body frame. One dimension at a time
 * @val, raw data from the sensor
 */
float convert_to_accel(float val) {
    return (val/accel_sens)*g;
}

/**
 * convert the raw data from MPU to angular acceleration
 * that we are familiar with. One dimension at a time.
 * @val, raw data from the sensor
 */
float convert_to_ang_accel(float val) {
    /*TODO*/
    //no sure know to convert it yet.
    return 0;
}

/**
 * update the rotation of the sensor
 * @delta_time, the duration of time elapsed from previous update
 * @ang_accel_*, angular acceleration with respect to the local frame
 * @side_affect, a, b, c are updated
 */
void update_angular(float ang_accel_a, float ang_accel_b, float ang_accel_c, float delta_time) {
    a = a+a_v*delta_time+1/2*ang_accel_a*delta_time*delta_time;
    b = b+b_v*delta_time+1/2*ang_accel_b*delta_time*delta_time;
    c = c+c_v*delta_time+1/2*ang_accel_c*delta_time*delta_time;
}

/**
 * update the translation of the sensor
 * @delta_time, the duration of time elapsed from previous update
 * @accel_*, the acceleration of each dimensions with respect to inertia frame
 * @side_affect, x, y, z are updated
 */
void update_translation(float accel_x, float accel_y, float accel_z, float delta_time) {
    x = x+1/2*accel_x*delta_time*delta_time+x_v*delta_time;
    y = y+1/2*accel_y*delta_time*delta_time+y_v*delta_time;
    z = z+1/2*accel_z*delta_time*delta_time+z_v*delta_time;
}

/**
 * update the angular volecity
 * @delta_time, the duration of the time elapsed from previous update
 * @ang_accel_*, the angular accelerations for each angles with respect to local frame
 * @side_affect, a_v, b_v, c_v are updated
 */
void update_anglar_v(float ang_accel_a, float ang_accel_b, float ang_accel_c, float delta_time) {
    a_v = a_v+ang_accel_a*delta_time;
    b_v = b_v+ang_accel_b*delta_time;
    c_v = c_v+ang_accel_c*delta_time;
}

/**
 * update the velocity.
 * @delta_time, the duration of time elapsed from previous update
 * @accel_*, acceleration with respect to local frame
 * @side_affect, x_v, y_v, z_v are updated
 */
void update_v(float accel_x, float accel_y, float accel_z, float delta_time){
    x_v=0, y_v=0, z_v=0;
    x_v = x_v+accel_x*delta_time;
    y_v = y_v+accel_x*delta_time;
    z_v = z_v+accel_x*delta_time;
}

/**
 * convert acceleration or force vector from body frame to local frame.
 * this is effective a matrix multiplication. consult the paper for more
 * @val_*, the vector
 * @side_affect, the "return" vector will be store in float_arr_3.
 * 				float_arr_3[0] is new x, float_arr_3[1] is new y
 * 				float_arr_3[3] is new z.
 */
void BtoL(val_x, val_y, val_z) {
    float_arr_3[0] = cos(c)*cos(b)*val_x
    +(-sin(c)*cos(a)+cos(c)*sin(b)*sin(a))*val_y
    +(sin(c)*sin(a)+cos(c)*sin(b)*cos(a))*val_z;
    float_arr_3[1] = sin(c)*cos(b)*val_x
    +(cos(c)*cos(a)+sin(c)*sin(b)*sin(a))*val_y
    +(-cos(c)*sin(a)+sin(c)*sin(b)*cos(a))*val_z;
    float_arr_3[2] = -sin(b)*x+cos(b)*sin(a)*y+cos(b)*cos(a)*z
}

/**
 * convert acceleration or force vector from Local frame to Body frame.
 * this is effective a matrix multiplication. consult the paper for more
 * @val_*, the vector
 * @side_affect, the "return" vector will be store in float_arr_3.
 *                float_arr_3[0] is new x, float_arr_3[1] is new y,
 *                float_arr_3[2].
 */
void LtoB(val_x, val_y, val_z) {
    float_arr_3[0] = cos(c)*cos(b)*val_x+sin(c)*cos(b)*val_y-sin(b)*val_z;
    float_arr_3[1] = (-sin(c)*cos(a)+cos(c)*sin(b)*sin(a))*val_x
    + (cos(c)*cos(a)+sin(c)*sin(b)*sin(a))*val_y
    + (cos(b)*sin(a))*val_z;
    float_arr_3[2] = (sin(c)*sin(a)+cos(c)*sin(b)*cos(a))*val_x
    + (-cos(c)*sin(a)+sin(c)*sin(b)*cos(a))*val_y
    + cos(b)*cos(a)*val_z;
}

int main(void) {
    TM_MPU6050_t MPU6050_Data;
    char str[120];
    
    /* Initialize system */
    SystemInit();
    
    /* Initialize delay */
    TM_DELAY_Init();
    
    /* Initialize USART, TX: PB6 */
    TM_USART_Init(USART1, TM_USART_PinsPack_2, 115200);
    
    /* Initialize MPU6050 sensor */
    if (TM_MPU6050_Init(&MPU6050_Data, TM_MPU6050_Device_0, TM_MPU6050_Accelerometer_8G, TM_MPU6050_Gyroscope_2000s) != TM_MPU6050_Result_Ok) {
        /* Display error to user */
        TM_USART_Puts(USART1, "MPU6050 Error\n");
        
        /* Infinite loop */
        while (1);
    }
    
    while (1) {
        /* Read all data from sensor */
        TM_MPU6050_ReadAll(&MPU6050_Data);
        
        /* Format data */
        sprintf(str, "Accelerometer: (%d, %d, %d)\r\n Gyroscope: (%d, %d, %d)\r\n\r\n",
                MPU6050_Data.Accelerometer_X,
                MPU6050_Data.Accelerometer_Y,
                MPU6050_Data.Accelerometer_Z,
                MPU6050_Data.Gyroscope_X,
                MPU6050_Data.Gyroscope_Y,
                MPU6050_Data.Gyroscope_Z
                );
        
        /* Show to usart */
        TM_USART_Puts(USART1, str);
        
        /* Little delay */
        Delayms(500);
    }
}