/*
 * cj_helper.c
 *
 *  Created on: Jul 31, 2015
 *      Author: chaojie wang
 */


#include "cj_helper.h"


void BtoL(struct Cj_helper_float3 v_b, struct Cj_helper_float3 angles, struct Cj_helper_float3* v_l) {
	v_l->a = cos(angles.a/180*PI)*cos(angles.b/180*PI)*v_b.a
				+(-sin(angles.a/180*PI)*cos(angles.c/180*PI) +
						cos(angles.a/180*PI)*sin(angles.b/180*PI)*sin(angles.c/180*PI))*v_b.b
				+(sin(angles.c/180*PI)*sin(angles.a/180*PI) +
						cos(angles.c/180*PI)*sin(angles.b/180*PI)*cos(angles.a/180*PI))*v_b.c;

	v_l->b = sin(angles.a/180*PI)*cos(angles.b/180*PI)*v_b.a
			+(cos(angles.c/180*PI)*cos(angles.a/180*PI) +
					sin(angles.c/180*PI)*sin(angles.b/180*PI)*sin(angles.a/180*PI))*v_b.b
			+(-cos(angles.a/180*PI)*sin(angles.c/180*PI) +
					sin(angles.a/180*PI)*sin(angles.b/180*PI)*cos(angles.c/180*PI))*v_b.c;

	v_l->c = sin(angles.a/180*PI)*cos(angles.b/180*PI)*v_b.a
				+(cos(angles.c/180*PI)*cos(angles.a/180*PI) +
						sin(angles.c/180*PI)*sin(angles.b/180*PI)*sin(angles.a/180*PI))*v_b.b
				+(-cos(angles.a/180*PI)*sin(angles.c/180*PI) +
						sin(angles.a/180*PI)*sin(angles.b/180*PI)*cos(angles.c/180*PI))*v_b.c;
}


void LtoB(struct Cj_helper_float3 v_l, struct Cj_helper_float3 angles, struct Cj_helper_float3* v_b) {
	v_b->a = cos(angles.a/180*PI)*cos(angles.b/180*PI)*v_l.a+
			sin(angles.a/180*PI)*cos(angles.b/180*PI)*v_l.a-sin(angles.b/180*PI)*v_l.c;
	v_b->b = (-sin(angles.a/180*PI)*cos(angles.c/180*PI) +
			cos(angles.a/180*PI)*sin(angles.b/180*PI)*sin(angles.c/180*PI))*v_l.a
			+ (cos(angles.a/180*PI)*cos(angles.c/180*PI) +
					sin(angles.a/180*PI)*sin(angles.b/180*PI)*sin(angles.c/180*PI))*v_l.b
			+ (cos(angles.b/180*PI)*sin(angles.c/180*PI))*v_l.c;
	v_b->c = (sin(angles.a/180*PI)*sin(angles.c/180*PI)+
			cos(angles.a/180*PI)*sin(angles.b/180*PI)*cos(angles.c/180*PI))*v_l.a
				+ (-cos(angles.a/180*PI)*sin(angles.c/180*PI) +
						sin(angles.a/180*PI)*sin(angles.b/180*PI)*cos(angles.c/180*PI))*v_l.b
				+ cos(angles.b/180*PI)*cos(angles.c/180*PI)*v_l.c;
}
