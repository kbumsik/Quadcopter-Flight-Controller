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
const float accel_sens = 8192;
//LSB Sensitivity of angular velocity
const float angv_sens = 65.5;
const float ang_cap = 360;  //360 degrees

/* parameters that we need to keep track of. all these are with respect to inertia frame*/
//six degrees of freedom
static float x=0, y=0, z=0, a=0, b=0, c=0;
//velocity and angular velocity
static float x_v=0, y_v=0, z_v=0, a_v=0, b_v=0, c_v=0;
//accelerations
static float x_accel=0, y_accel=0, z_accel=0;

/*some temporary placeholder*/
//array of float with 3 elem
static float float_arr_3[3];


/**
 * convert the raw data from MPU to acceleration we are familiar with.
 * this is with respect to body frame. One dimension at a time.
 * note that, this is not true acceleration, and it needs correction, due
 * to the gravity
 * @val, raw data from the sensor
 * @side_effect, "return" will be stored in float_arr_3
 */
void convert_to_accel(int val1, int val2, int val3) {
    float_arr_3[0] = (val1/accel_sens)*g;
    float_arr_3[1] = (val2/accel_sens)*g;
    float_arr_3[2] = (val3/accel_sens)*g;
}

/**
 * convert the raw data from MPU to angular acceleration
 * that we are familiar with. One dimension at a time.
 * @val, raw data from the sensor
 * @side_effect, "return" will be stored in float_arr_3
 */
void convert_to_angv(int val1, int val2, int val3) {
    float_arr_3[0] = val1/angv_sens;
    float_arr_3[1] = val2/angv_sens;
    float_arr_3[2] = val3/angv_sens;
}

/**
 * update the rotation of the sensor
 * @delta_time, the duration of time elapsed from previous update
 * @side_affect, a, b, c are updated
 */
void update_angular(float delta_time) {
    a = a+a_v*delta_time;
    b = b+b_v*delta_time;
    c = c+c_v*delta_time;
}

/**
 * update the translation of the sensor
 * @delta_time, the duration of time elapsed from previous update
 * @accel_*, the acceleration of each dimensions with respect to inertia frame
 * @side_affect, x, y, z are updated
 */
void update_translation(float delta_time) {
    x = x+1/2*x_accel*delta_time*delta_time+x_v*delta_time;
    y = y+1/2*x_accel*delta_time*delta_time+y_v*delta_time;
    z = z+1/2*x_accel*delta_time*delta_time+z_v*delta_time;
}

/**
 * update the velocity.
 * @delta_time, the duration of time elapsed from previous update
 * @accel_*, acceleration with respect to local frame
 * @side_affect, x_v, y_v, z_v are updated
 */
void update_v(float delta_time){
    x_v = x_v+x_accel*delta_time;
    y_v = y_v+y_accel*delta_time;
    z_v = z_v+z_accel*delta_time;
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
    float_arr_3[2] = -sin(b)*x+cos(b)*sin(a)*y+cos(b)*cos(a)*z;
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

/*  functions and variables related to determine delta time  */
//previous time in unit seconds
static float det_delta_time_prev = -1;
/**
 *  initial the timer for keeping track of delta time
 *  @side_affect, det_delta_time_prev
 */
void det_delta_time_init() {
    //TODO: start the timer
    det_delta_time_prev = 0;
}

/**
 * determine delta_time, which is interval between last call
 * @return, delta_time
 * @side_affect, det_delta_time_prev
 */
float det_delta_time() {
    //TODO: determine the time interval by getting the time now, and
    //      substract it from previous time. and update the previous time
    //TODO: think about how to deal with overflow
}

/* end of determine time*/

/**
 * update state of the drone. all 18 quantities will be updated, the order of
 * update,
 */
void update_state(int accelerometer_X, int accelerometer_Y, int accelerometer_Z,
                  int gyroscope_X, int gyroscope_Y, int gyroscope_Z) {
    //if timer has not started
    if (det_delta_time_prev == -1) {
        det_delta_time_init();
        //note that, here we assume that the drone is on the level ground and
        //static. so all quantities don't need to update. 0 is ok. so just return.
        //if however, this assumption is not true, we need somehow determine the
        //initial state.
        return ;
    }
    
    //del_t is not accurate for each case, coz, we don't take into account
    //that each function takes time.
    float del_t = det_delta_time();
    update_angular(del_t);
    update_translation(del_t);
    update_v(del_t);
    
    
    //update the angular accelerations
    convert_to_angv(gyroscope_X, gyroscope_Y, gyroscope_Z);
    a_v = float_arr_3[0];
    b_v = float_arr_3[1];
    c_v = float_arr_3[2];
    
    //update the acceleration
    convert_to_accel(accelerometer_X,accelerometer_Y,accelerometer_Z);
    x_accel = float_arr_3[0];
    y_accel = float_arr_3[1];
    z_accel = float_arr_3[2];
    BtoL(x_accel, y_accel, z_accel);
    x_accel = -float_arr_3[0];
    y_accel = -float_arr_3[1];
    z_accel = -float_arr_3[2];
    //correct the accel due to the particularity of our sensor
    z_accel += g;
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
    if (TM_MPU6050_Init(&MPU6050_Data, TM_MPU6050_Device_0, TM_MPU6050_Accelerometer_4G, TM_MPU6050_Gyroscope_500s) != TM_MPU6050_Result_Ok) {
        /* Display error to user */
        TM_USART_Puts(USART1, "MPU6050 Error\n");
        
        /* Infinite loop */
        while (1);
    }
    
    
    while (1) {
        /* Read all data from sensor */
        TM_MPU6050_ReadAll(&MPU6050_Data);
        
        /* Format data */
        sprintf(str, "Accelerometer: (%d, %d, %d)\r\n Gyroscope: (%d, %d, %d)\r\n",
                MPU6050_Data.Accelerometer_X,
                MPU6050_Data.Accelerometer_Y,
                MPU6050_Data.Accelerometer_Z,
                MPU6050_Data.Gyroscope_X,
                MPU6050_Data.Gyroscope_Y,
                MPU6050_Data.Gyroscope_Z
                );
        
        //update the state of the system
        update_state(MPU6050_Data.Accelerometer_X,
                     MPU6050_Data.Accelerometer_Y,
                     MPU6050_Data.Accelerometer_Z,
                     MPU6050_Data.Gyroscope_X,
                     MPU6050_Data.Gyroscope_Y,
                     MPU6050_Data.Gyroscope_Z);
        
        //print the data
        sprintf(str, "translation: (%.5f, %.5f, %.5f)\r\n rotation: (%.5f, %.5f, %.5f)\r\n\r\n",
                x,y,z,a,b,c
                );
        
        /* Show to usart */
        TM_USART_Puts(USART1, str);
        
        /* Little delay */
        Delayms(100);
    }
}
