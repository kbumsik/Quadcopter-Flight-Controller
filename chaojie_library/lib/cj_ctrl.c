/*
 * cj_ctrl.c
 *
 *  Created on: Aug 16, 2015
 *      Author: chaojie wang
 */

#include "cj_ctrl.h"
#include "tm_stm32f4_delay.h"

#define FORCE_DELAY 0.001                       //time delay between force apply

/* parameters */
static struct Cj_helper_float4 _force;          //set force the for each wings

static float _K1_theta;                         //rql control parameters
static float _K2_theta;
static float _K1_phi;                   
static float _K2_phi;
static float _K1_psi;                   
static float _K2_psi;

static struct Cj_helper_float3 _angle_d         //desire angles

/**
 *  setting up parameters of the controller, and connection to the motor
 */
void cj_ctrl_init() {
    //by default no force is applied
    _force.a = _force.b = _force.c = _force.d = 0;

    //default desire angles
    _angle_d.a = _angle_d.b = _angle_d.c = 0;
    
    /*TODO:
        figure out K for each angle
	consult the review for how to determine them
     *

    /*TODO:
        init the motor?
     */
}

/**
 *  set the force to the quadcopter. The signal is from the remote controller
 */
void cj_ctrl_set_force(float f) {
    _force.a = _force.b = _force.c = _force.d = f;
}

/**
 *  pitch to an angle. The angle along x axis.
 *  @pitch_angle, the angle in degrees
 */
void cj_ctrl_pitch(float pitch_angle) {
    _angle_d.b = pitch_angle;
}

/**
 *  roll to an angle. The angle along y axis.
 *  @roll_angle, the angle in degrees
 */
void cj_ctrl_roll(float roll_angle) {
    _angle_d.a = roll_angle;
}

/**
 *  yaw to an angle. The angle along z axis.
 *  @yaw_angle, the angle in degrees
 */
void cj_ctrl_yaw(float yaw_angle) {
    _angle_d.c = yaw_angle;
}

void cj_ctrl_apply_ctrl() {
    struct Cj_helper_float4 force;
    struct angle;
    struct angle_v;
    
    //controll the pitch first
    force = _force;
    cj_state_update();
    cj_state_get_angles(&angle);
    cj_state_get_angular_velocity(&angle_v);
    
    float u_theta = _K1_theta*(angle.b-_angle_d.b) + _K2_theta*angle_v.b;
    force.c += u_theta/2;
    force.a -= u_theta/2;
    
    apply_force(force);                                          
    Delayms(ceil(FORCE_DELAY*1000));
    
    //controll the roll
    force = _force;
    cj_state_update();
    cj_state_get_angles(&angle);
    cj_state_get_angular_velocity(&angle_v);

    float u_phi = _K1_phi*(angle.a-_angle_d.a) + _K2_phi*angle_v.a;
    force.b += u_phi/2;
    force.d -= u_phi/2;
    
    apply_force(force);
    Delayms(ceil(FORCE_DELAY*1000));

    //lastly, yaw control
    force = _force;
    cj_state_update();
    cj_state_get_angles(&angle);
    cj_state_get_angular_velocity(&angle_v);

    float u_psi = _K1_psi*(angle.c-_angle_d.c) + _K2_psi*angle_v.c;
    force.a = force.c = force.c+u_psi/2;
    force.b = force.d = force.c-u_psi/2;
    
    apply_force(force);
    Delayms(ceil(FORCE_DELAY*1000));

    apply_force(_force);
}

static void apply_force(struct Cj_helper_float4 force) {
    /*TODO:
       figure out how to translate force to 
       the motor(PWM?)
     */
}
