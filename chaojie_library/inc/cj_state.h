/*
 * cj_state.h
 *
 *  Description:
 *    uses MPU9150 to track the state of the system
 *    state means angles and angular eccelerations.
 *
 *  Note:
 *    we can extend the state to include the position of the
 *    system. But it require more work. We might do that if
 *    we need to later
 *
 *  Created on: Jul 31, 2015
 *      Author: chaojie wang
 */

#ifndef CJ_STATE_
#define CJ_STATE_

/* C++ detection */
#ifdef __cplusplus
extern "C" {
#endif

#include <math.h>
#include "tm_stm32f4_delay.h"
#include "tm_stm32f4_mpu6050.h"
#include "cj_helper.h"

#define PI 3.14159265359

/*sensor*/
static KB_MPU9150_t MPU9150_Data;
static TM_MPU6050_t MPU6050_Data;

/*calibration data*/
static struct Cj_helper_float3 vcali;

/* parameters that we need to keep track of. all these are with respect to inertia frame*/
//angles
static struct Cj_helper_float3 angles;
//angular velocity
static struct Cj_helper_float3 angles_v;

/* other variables*/
//tracking previous time so that delta time can be determined
static float prev_t;


/**
 * initialize the state tracking. set up MPU9150, determine parameters
 * and calibrate angular velocity
 * @side effect: vcali, angles, angles_v and other constant
 * @return, 0: fail, 1:success
 */
int cj_state_init();

/**
 * update the state
 * @side effect: angles and angles_v
 */
void cj_state_update();

/**
 * Assuming the system is in equilibrium. Calculate the angle between 
 * local frame and body frame. This is same code for doing initial state
 * estimation.
 * @side effect: angles and angles_v
 */
void recalculate_state();

/**
 * get angles.
 * @angles, where the return data will be stored
 */
void cj_state_get_angles(struct Cj_helper_float3* a);

/**
 * get angular velocity.
 * @angles_v, where the return data will be stored
 */
void cj_state_get_angular_velocity(struct Cj_helper_float3* a);


#ifdef __cplusplus
}
#endif

#endif /* CJ_STATE_ */
