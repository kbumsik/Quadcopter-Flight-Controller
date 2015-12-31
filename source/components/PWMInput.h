/*
 * PWMInput.h
 *
 *  Created on: Dec 26, 2015
 *      Author: Bumsik Kim
 */

#ifndef PWMINPUT_H_
#define PWMINPUT_H_

/* Global variables */
extern TIM_HandleTypeDef xTIM1Handle;
extern TIM_HandleTypeDef xTIM2Handle;
extern TIM_HandleTypeDef xTIM3Handle;
extern TIM_HandleTypeDef xTIM5Handle;

#ifdef __cplusplus
extern "C"{
#endif

/* Function Prototypes */
void vPWMInputInit(TIM_HandleTypeDef* pxTIMHandle, TIM_TypeDef* pxTIMx, uint32_t xChannel);
void vPWMInputStart(TIM_HandleTypeDef* pxTIMHandle);
uint32_t ulPWMInputPeriod(TIM_HandleTypeDef* pxTIMHandle);
uint32_t ulPWMInputDutyCycle(TIM_HandleTypeDef* pxTIMHandle);

/* callback functions */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *pxTIMHandle);

#ifdef __cplusplus
}
#endif

#endif /* PWMINPUT_H_ */
