/*
 * cj_state2.h
 *
 *  Description:
 *    uses MPU9150 to track the state of the system
 *    state means angles. getters are also provided to access
 *    data from the sensor MPU9150. This implementation uses
 *    different approach than that of state.h. It is largely 
 *    based on the paper "A Simplified Quaternion-Based Algorithm for
 *    Orientation Estimation From Earth Gravityand Magnetic Field Measurements"
 *
 *  Note:
 *    all return vectors are with respect to inertial frame. The inertial frame is 
 *    the one in the report "Inertia Frame Determination". The calibration is 
 *    based on the assumption that the noise is a guassian with a constant mean. 
 *    In other words, the data is biased and the biase doesn't change.
 *
 *  Created on: Aug 08, 2015
 *      Author: chaojie wang
 */

#ifndef CJ_STATE2_
#define CJ_STATE2_

/* C++ detection */
#ifdef __cplusplus
extern "C" {
#endif

#include <math.h>
#include "tm_stm32f4_delay.h"
#include "kb_stm32f4_mpu9150.h"
#include "cj_helper.h"

#define PI 3.14159265359
#define G 9.802                       #gratity in NYC

/*sensor*/
static KB_MPU9150_t MPU9150_Data;

/*calibration data*/
//angular velocity calibration data
static struct Cj_helper_float3 vcali;
//acceleration calibration data
static struct Cj_helper_float3 acali;
//magnetic vector calibration data
static struct Cj_helper_float3 mcali;

/* parameters that we need to keep track of. all these are with respect to inertia frame*/
//angles in unit degree, a is phi(roll), b is theta(pitch), c is psi(yaw)
static struct Cj_helper_float3 angles;
//angular velocity
static struct Cj_helper_float3 angles_v;
//acceleration, (x,y,z)
static struct Cj_helper_float3 accel;
//magnetic field
static struct Cj_helper_float3 mag_v;

/**
 * initialize the state tracking. set up MPU9150, and calibrate
 * @return, 0: fail, 1:success
 */
int cj_state_init();

/**
 * update the state. All data from the sensor is updated
 */
void cj_state_update();
  
/**
 * get angles. based on the paper to estimate the angles. Assume the motion
 * acceleration is negligible.
 * @a, where the return data will be stored
 */
void cj_state_get_angles(struct Cj_helper_float3* a);

/**
 * get angular velocity.
 * @a, where the return data will be stored
 */
void cj_state_get_angular_velocity(struct Cj_helper_float3* a);

/**
 * get acceleration
 * @a, where the return data will be stored
 */
void cj_state_get_acceleration(struct Cj_helper_float3* a);

/**
 * get magnetical field
 * @a, where the return data will be stored
 */
void cj_state_get_magnetic_vector(struct Cj_helper_float3* a);


#ifdef __cplusplus
}
#endif

#endif /* CJ_STATE2_ */
