/*
 * cj_helper.h
 *
 * Description: some important helper functions
 *
 *  Created on: Jul 31, 2015
 *      Author: chaojie wang
 */

#ifndef CJ_HELPER_H_
#define CJ_HELPER_H_

#include <math.h>
#define PI 3.14159265359

//placeholder of 3 float
typedef struct Cj_helper_float3 {
	float a;
	float b;
	float c;
};

/**
 * given angles and a vector in body frame. convert the vector to local frame
 *
 * @v_b, the vector in body frame
 * @angles, the angles in degree
 * @v_l, the output
 */
void BtoL(struct Cj_helper_float3 v_b, struct Cj_helper_float3 angles, struct Cj_helper_float3* v_l);

/**
 * given angles and a vector in local frame. convert the vector to local frame
 *
 * @v_l, the vector in local frame
 * @angles, the angles in degree
 * @v_b, the output
 */
void LtoB(struct Cj_helper_float3 v_l, struct Cj_helper_float3 angles, struct Cj_helper_float3* v_b);



#endif /* CJ_HELPER_H_ */
