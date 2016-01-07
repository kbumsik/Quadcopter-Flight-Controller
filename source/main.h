/*
 * main.h
 *
 *  Created on: Jan 4, 2016
 *      Author: Bumsik Kim
 */

#ifndef MAIN_H_
#define MAIN_H_

#include "config.h"
#include "arm_math.h"
#include "PWMInput.h"
#include "Motor.h"


/* Global variables */
TIM_HandleTypeDef xTIM1Handle;
TIM_HandleTypeDef xTIM2Handle;
TIM_HandleTypeDef xTIM3Handle;
TIM_HandleTypeDef xTIM5Handle;

/* Remote -------------------------------------------------------------------*/
const uint32_t ppuwRemoteRange[confREMOTE_NUMBER_OF_CHANNEL][3]=
{
  {confREMOTE_CH1_MIN, confREMOTE_CH1_CENTER, confREMOTE_CH1_MAX},
  {confREMOTE_CH2_MIN, confREMOTE_CH2_CENTER, confREMOTE_CH2_MAX},
  {confREMOTE_CH3_MIN, confREMOTE_CH3_CENTER, confREMOTE_CH3_MAX},
  {confREMOTE_CH4_MIN, confREMOTE_CH4_CENTER, confREMOTE_CH4_MAX}
};	/* array to store remote controller's range */

const float32_t pfRemoteScale[confREMOTE_NUMBER_OF_CHANNEL]=
{
  (float)1000/(confREMOTE_CH1_MAX-confREMOTE_CH1_MIN),
  (float)1000/(confREMOTE_CH2_MAX-confREMOTE_CH2_MIN),
  (float)1000/(confREMOTE_CH3_MAX-confREMOTE_CH3_MIN),
  (float)1000/(confREMOTE_CH4_MAX-confREMOTE_CH4_MIN)
}; /* array to store remote controller's scale */

float32_t pfRemote[confREMOTE_NUMBER_OF_CHANNEL]; /* array of remote controller's value */

arm_matrix_instance_f32 xPWMInput = {1, confREMOTE_NUMBER_OF_CHANNEL, pfRemote}; /* matrix structure of ::pfPWMInput */

TIM_HandleTypeDef* pxRemoteHandle[confREMOTE_NUMBER_OF_CHANNEL] = 
{
  &xTIM1Handle,
  &xTIM2Handle,
  &xTIM3Handle,
  &xTIM5Handle
}; /* Array of pointer of handler */

/* Motor ------------------------------------------------------------------- */
const float32_t pfRemoteToMotorScale[confREMOTE_NUMBER_OF_CHANNEL]=
{
  (float)(motorSPEED_MAX-motorSPEED_MIN)/1000,
  (float)(motorSPEED_MAX-motorSPEED_MIN)/1000,
  (float)(motorSPEED_MAX-motorSPEED_MIN)/1000,
  (float)(motorSPEED_MAX-motorSPEED_MIN)/1000
}; /* array to store remote controller's scale */

float32_t pfMotorSpeed[confREMOTE_NUMBER_OF_CHANNEL];

#endif /* MAIN_H_ */
