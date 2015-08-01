/*
 * cj_state.c
 *
 *  Created on: Jul 31, 2015
 *      Author: chaojie wang
 */


#include "cj_state.h"

int cj_state_init() {
	if (TM_MPU6050_Init(&MPU6050_Data, TM_MPU6050_Device_0, TM_MPU6050_Accelerometer_4G, TM_MPU6050_Gyroscope_500s) != TM_MPU6050_Result_Ok) {
			return 0;
	}

	//since it is not in motion, angular velocity is 0
	angles_v.a = 0;
	angles_v.b = 0;
	angles_v.c = 0;

	vcali.a = 0;
	vcali.b = 0;
	vcali.c = 0;

	struct Cj_helper_float3 g;
	g.a = 0;
	g.b = 0;
	g.c = 0;

	struct Cj_helper_float3 n;
	n.a = 0;
	n.b = 0;
	n.c = 0;

	int num_of_itr = 1000;
	int i = 0;
	for (; i < num_of_itr; i++) {
		TM_MPU6050_ReadAll(&MPU6050_Data);
		vcali.a += MPU6050_Data.Gyroscope_X;
		vcali.b += MPU6050_Data.Gyroscope_Y;
		vcali.c += MPU6050_Data.Gyroscope_Z;

		g.a += MPU6050_Data.Accelerometer_X;
		g.b += MPU6050_Data.Accelerometer_Y;
		g.c += MPU6050_Data.Accelerometer_Z;

		n.a += MPU6050_Data.Magnetometer_X;
		n.b += MPU6050_Data.Magnetometer_Y;
		n.c += MPU6050_Data.Magnetometer_Z;
	}

	//calibration
	vcali.a = vcali.a/num_of_itr;
	vcali.b = vcali.b/num_of_itr;
	vcali.c = vcali.c/num_of_itr;

	//get two angles from g vector
	g.a = g.a/num_of_itr;
	g.b = g.b/num_of_itr;
	g.c = g.c/num_of_itr;
	float g_abs = sqrt(g.a*g.a + g.b*g.b + g.c*g.c);

	angles.b = asin(g.a/g_abs);
	angles.c = asin(-g.b/(g_abs*cos(angles.b)));

	//get the last angle
	n.a = n.a/num_of_itr;
	n.b = n.b/num_of_itr;
	n.c = n.c/num_of_itr;
	float n_abs = sqrt(n.a*n.a+n.b*n.b+n.c*n.c);

	angles.a = acos(n.a/(n_abs*cos(angles.b)));

	prev_t = TM_DELAY_Time()/1000.0;

	return 1;
}

void cj_state_update() {
	/*The implement is plain update for simplicity. Kalman filter is not implemented yet*/
	float delta_time = TM_DELAY_Time()/1000.0 - prev_t;
	prev_t = TM_DELAY_Time();

	angles.a = angles.a + delta_time*angles_v.a;
	angles.b = angles.b + delta_time*angles_v.b;
	angles.c = angles.c + delta_time*angles_v.c;

	TM_MPU6050_ReadAll(&MPU6050_Data);
	angles_v.a = MPU6050_Data.Gyroscope_X;
	angles_v.b = MPU6050_Data.Gyroscope_Y;
	angles_v.c = MPU6050_Data.Gyroscope_Z;
}

void cj_state_get_angles(struct Cj_helper_float3* a) {
	*a = angles;
}

void cj_state_get_angular_velocity(struct Cj_helper_float3* a) {
	*a = angles_v;
}




