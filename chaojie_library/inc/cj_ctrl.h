/*
 * cj_ctrl.h
 *
 *  Description:
 *    This is for controlling the angles. The implement is lqr controller
 *    , but without integral action.
 *
 *  Created on: Aug 16, 2015
 *      Author: chaojie wang
 */

#ifndef CJ_CTRL_
#define CJ_CTRL_

/* C++ detection */
#ifdef __cplusplus
extern "C" {
#endif

#include <math.h>
#include "cj_helper.h"
#include "cj_state2.h"


/**
 *  setting up parameters of the controller, and connection to the motor
 */
void cj_ctrl_init();

/**
 *  set the force to the quadcopter. The signal is from the remote controller
 */
void cj_ctrl_set_force(float f);

/**
 *  set desired pitch angle.
 *  @pitch_angle, the angle in degrees
 */
void cj_ctrl_pitch(float pitch_angle);

/**
 *  set desire roll angle.
 *  @roll_angle, the angle in degrees
 */
void cj_ctrl_roll(float roll_angle);

/**
 *  set desire yaw angle. 
 *  @yaw_angle, the angle in degrees
 */
void cj_ctrl_yaw(float yaw_angle);

/**
 *  apply force on the quadcopter. This should be take high priority and it
 *  should be executed constantly, maybe in a timer interrupt fashion.
 */
void cj_ctrl_apply_ctrl();

#ifdef __cplusplus
}
#endif

#endif /* CJ_CTLR_ */
