/*
 * cj_state.c
 *
 *  Created on: Aug 8, 2015
 *      Author: chaojie wang
 */


#include "cj_state2.h"

int cj_state_init() {
    //configure the sensor
    if (KB_MPU9150_Init(&MPU9150_Data, KB_MPU9150_Accelerometer_4G, KB_MPU9150_Gyroscope_500s) != KB_MPU9150_Result_Ok) {
	return 0;
    }

    //init the calibration data
    vcali.a = vcali.b = vcali.c = 0;
    acali.a = acali.b = acali.c = 0;
    mcali.a = mcali.b = mcali.c = 0;

    int num_of_itr = 1000;
    int i = 0;
    for (; i < num_of_itr; i++) {
	KB_MPU9150_ReadAll(&MPU9150_Data);
	vcali.a += MPU9150_Data.Gyroscope_X;
	vcali.b += MPU9150_Data.Gyroscope_Y;
	vcali.c += MPU9150_Data.Gyroscope_Z;
    }

    //calibration
    vcali.a = vcali.a/num_of_itr;
    vcali.b = vcali.b/num_of_itr;
    vcali.c = vcali.c/num_of_itr;

    //TODO, calibration of acceleration and magnetic vector
    acali.a = acali.b = acali.c = 0;
    mcali.a = mcali.b = mcali.c = 0;

    return 1;
}

void cj_state_update() {
    KB_MPU9150_ReadAll(&MPU9150_Data);

    Cj_helper_float3 g;
    g.a = accel.a = MPU9150_Data.Accelerometer_X - acali.a;
    g.b = accel.b = MPU9150_Data.Accelerometer_Y - acali.b;
    g.c = accel.c = MPU9150_Data.Accelerometer_Z - acali.c;

    //normalization
    float g_abs = sqrt(g.a*g.a + g.b*g.b + g.c*g.c);
    g.a = g.a/g_abs;
    g.b = g.b/g_abs;
    g.c = g.c/g_abs;
    
    angles.b = asin(g.a);

    //TODO singularity?
    if (-g.b/cos(angles.b) > 0)
	angles.a = acos(-g.c/cos(angles.b));
    else
	angles.a = -acos(-g.c/cos(angles.b));

    Cj_helper_float3 mag_v_n;
    mag_v_n.a = mag_v.a = MPU9150_Data.Magnetometer_X - mcali.a;
    mag_v_n.b = mag_v.b = MPU9150_Data.Magnetometer_Y - mcali.b;
    mag_v_n.c = mag_v.c = MPU9150_Data.Magnetometer_Z - mcali.c;
    //tranform to body frame
    mag_v.a = mag_v_n.b;
    mag_v.b = mag_v_n.a;
    mag_v.c = -mag_v_n.c;
    //normaization only on a and b
    float mag_v_abs = sqrt(mag_v.a*mag_v.a + mag_v.b*mag_v.b); 
    mag_v_n.a = mag_v.a/mag_v_abs;
    mag_v_n.b = mag_v.b/mag_v_abs;

    if (mag_v_n.b > 0)
	angle.c = acos(mag_v_n.a);
    else
	angle.c = -acos(mag_v_n.a);

    //convert the unit from radians to degrees
    angles.a = angles.a*180/PI;
    angles.b = angles.b*180/PI;
    angles.c = angles.b*180/PI;

    //convert the accel to unit of m/s^2
    accel.a = accel.a*G;
    accel.b = accel.b*G;
    accel.c = accel.c*G;
    
    //lastly update the angular velocity
    angles_v.a = (MPU9150_Data.Gyroscope_X - vcali.a)*180/PI;
    angles_v.b = (MPU9150_Data.Gyroscope_Y - vcali.b)*180/PI;
    angles_v.c = (MPU9150_Data.Gyroscope_Z - vcali.c)*180/PI;
}

void cj_state_get_angles(struct Cj_helper_float3* a) {
    a->a = angles.a;
    a->b = angles.b;
    a->c = angles.c;
}

void cj_state_get_angular_velocity(struct Cj_helper_float3* a) {
    a->a = angles_v.a;
    a->b = angles_v.b;
    a->c = angles_v.c;
}

void cj_state_get_acceleration(struct Cj_helper_float3* a) {
    a->a = accel.a
    a->b = accel.b;
    a->c = accel.c;
}

void cj_state_get_magnetic_vector(struct Cj_helper_float3* a) {
    a->a = mag_v.a
    a->b = mag_v.b;
    a->c = mag_v.c;
}

