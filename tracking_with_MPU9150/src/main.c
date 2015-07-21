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
#include "kb_stm32f4_mpu9150.h"
#include "kb_stm32f4_functions.h"

#include <stdio.h>
#include <math.h>

/* some constants */
//acceleration due to the gravity in new york. Info is from the internet.
const float g = 9.803;
const float ang_cap = 360;  //360 degrees
#define PI 3.14159265359

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

/*calibration data*/
//for angular velocity
static float a_vcali = 0, b_vcali = 0, c_vcali = 0;


/**
 * convert the raw data from MPU to acceleration we are familiar with.
 * this is with respect to body frame. One dimension at a time.
 * note that, this is not true acceleration, and it needs correction, due
 * to the gravity
 * @val, raw data from the sensor
 * @side_effect, "return" will be stored in float_arr_3
 */
void convert_to_accel(float val1, float val2, float val3) {
    float_arr_3[0] = (val1)*g;
    float_arr_3[1] = (val2)*g;
    float_arr_3[2] = (val3)*g;
}

/**
 * convert the raw data from MPU to angular acceleration
 * that we are familiar with. One dimension at a time.
 * @val, raw data from the sensor
 * @side_effect, "return" will be stored in float_arr_3
 */
void convert_to_angv(float val1, float val2, float val3) {
    float_arr_3[0] = val1-a_vcali;
    float_arr_3[1] = val2-b_vcali;
    float_arr_3[2] = val3-c_vcali;
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
void BtoL(float val_x, float val_y, float val_z) {
    float_arr_3[0] = cos(a/180*PI)*cos(b/180*PI)*val_x
    +(-sin(a/180*PI)*cos(c/180*PI) + cos(a/180*PI)*sin(b/180*PI)*sin(c/180*PI))*val_y
    +(sin(c/180*PI)*sin(a/180*PI) + cos(c/180*PI)*sin(b/180*PI)*cos(a/180*PI))*val_z;
    float_arr_3[1] = sin(a/180*PI)*cos(b/180*PI)*val_x
    +(cos(c/180*PI)*cos(a/180*PI) + sin(c/180*PI)*sin(b/180*PI)*sin(a/180*PI))*val_y
    +(-cos(a/180*PI)*sin(c/180*PI) + sin(a/180*PI)*sin(b/180*PI)*cos(c/180*PI))*val_z;
    float_arr_3[2] = -sin(b/180*PI)*val_x+cos(b/180*PI)*sin(c/180*PI)*val_y+cos(b/180*PI)*cos(c/180*PI)*val_z;
}

/**
 * convert acceleration or force vector from Local frame to Body frame.
 * this is effective a matrix multiplication. consult the paper for more
 * @val_*, the vector
 * @side_affect, the "return" vector will be store in float_arr_3.
 *                float_arr_3[0] is new x, float_arr_3[1] is new y,
 *                float_arr_3[2].
 */
void LtoB(float val_x, float val_y, float val_z) {
    float_arr_3[0] = cos(a/180*PI)*cos(b/180*PI)*val_x+sin(a/180*PI)*cos(b/180*PI)*val_y-sin(b/180*PI)*val_z;
    float_arr_3[1] = (-sin(a/180*PI)*cos(c/180*PI) + cos(a/180*PI)*sin(b/180*PI)*sin(c/180*PI))*val_x
    + (cos(a/180*PI)*cos(c/180*PI) + sin(a/180*PI)*sin(b/180*PI)*sin(c/180*PI))*val_y
    + (cos(b/180*PI)*sin(c/180*PI))*val_z;
    float_arr_3[2] = (sin(a/180*PI)*sin(c/180*PI)+cos(a/180*PI)*sin(b/180*PI)*cos(c/180*PI))*val_x
    + (-cos(a/180*PI)*sin(c/180*PI) + sin(a/180*PI)*sin(b/180*PI)*cos(c/180*PI))*val_y
    + cos(b/180*PI)*cos(c/180*PI)*val_z;
}

/*  functions and variables related to determine delta time  */
//previous time in unit seconds
static float det_delta_time_prev = -1;
/**
 *  initial the timer for keeping track of delta time
 *  @side_affect, det_delta_time_prev
 */
void det_delta_time_init() {
    det_delta_time_prev = TM_DELAY_Time()/1000.0;
}

/**
 * determine delta_time, which is interval between last call
 * @return, delta_time
 * @side_affect, det_delta_time_prev
 */
float det_delta_time() {
    float now = TM_DELAY_Time()/1000.0;
    float r = now - det_delta_time_prev;
    det_delta_time_prev = now;
    return r;
}

/* end of determine time*/

/**
 * update state of the drone. all 18 quantities will be updated, the order of
 * update,
 */
void update_state(float accelerometer_X, float accelerometer_Y, float accelerometer_Z,
                  float gyroscope_X, float gyroscope_Y, float gyroscope_Z) {
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
    KB_MPU9150_t MPU9150_Data;
    char str[120];

    /* Initialize system */
    SystemInit();

    /* Initialize delay */
    TM_DELAY_Init();

    /* Initialize USART, TX: PB6 */
    TM_USART_Init(USART1, TM_USART_PinsPack_2, 115200);

    /* Initialize MPU6050 sensor */
    if (KB_MPU9150_Init(&MPU9150_Data, KB_MPU9150_Accelerometer_4G, KB_MPU9150_Gyroscope_500s) != KB_MPU9150_Result_Ok) {
        /* Display error to user */
        TM_USART_Puts(USART1, "MPU9150 Error\n");

        /* Infinite loop */
        while (1);
    }

    //calibrate the sensor
    int cali_num = 1000;
    int cali_delay_time = 10;
    int cali_i = 0;
    for (; cali_i < cali_num; cali_i++) {
        KB_MPU9150_ReadAll(&MPU9150_Data);
        a_vcali += MPU9150_Data.Gyroscope_X;
        b_vcali += MPU9150_Data.Gyroscope_Y;
        c_vcali += MPU9150_Data.Gyroscope_Z;
    }
    a_vcali = a_vcali/cali_num;
    b_vcali = b_vcali/cali_num;
    c_vcali = c_vcali/cali_num;

    while (1) {
	/* Printing buffers */
	char print_buffer_1[10];
	char print_buffer_2[10];
	char print_buffer_3[10];

        /* Read all data from sensor */
        KB_MPU9150_ReadAll(&MPU9150_Data);

        /*
         sprintf(str, "Accelerometer: (%d, %d, %d)\r\n Gyroscope: (%d, %d, %d)\r\n",
         MPU9150_Data.Accelerometer_X,
         MPU9150_Data.Accelerometer_Y,
         MPU9150_Data.Accelerometer_Z,
         MPU9150_Data.Gyroscope_X,
         MPU9150_Data.Gyroscope_Y,
         MPU9150_Data.Gyroscope_Z
         );
         */

        //update the state of the system
        update_state(MPU9150_Data.Accelerometer_X,
                     MPU9150_Data.Accelerometer_Y,
                     MPU9150_Data.Accelerometer_Z,
                     MPU9150_Data.Gyroscope_X,
                     MPU9150_Data.Gyroscope_Y,
                     MPU9150_Data.Gyroscope_Z);

        /* format data */
	conv_FloatToString(a_v, print_buffer_1);
	conv_FloatToString(b_v, print_buffer_2);
	conv_FloatToString(c_v, print_buffer_3);

        //print the data
        sprintf(str, "the angular velocity: (%s, %s, %s)",
                print_buffer_1, print_buffer_2, print_buffer_3);
        /* Show to usart */
        TM_USART_Puts(USART1, str);

        /* format data */
        conv_FloatToString(a, print_buffer_1);
        conv_FloatToString(b, print_buffer_2);
        conv_FloatToString(c, print_buffer_3);

        //print the data
        sprintf(str, "... angle:(%s, %s, %s)\r\n",
                print_buffer_1, print_buffer_2, print_buffer_3);

        /* Show to usart */
        TM_USART_Puts(USART1, str);

        /* Little delay */
        Delayms(10);
    }
}
