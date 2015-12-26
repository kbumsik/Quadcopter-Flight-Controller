/*
 * PWMInput_it.c
 *
 *  Created on: Dec 26, 2015
 *      Author: Bumsik Kim
 */

/* Includes ------------------------------------------------------------------*/
#include "config.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"


/* Private variable for Timer-------------------------------------------------*/
TIM_HandleTypeDef    TimHandle;
/**
  * @brief  This function handles TIM interrupt request. for RFreciever
  * @param  None
  * @retval None
  */
void PWMinput_TIMx_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&TimHandle);
}
