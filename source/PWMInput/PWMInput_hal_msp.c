/*
 * PWMInput_hal_msp.c
 *
 *  Created on: Dec 26, 2015
 *      Author: Bumsik Kim
 */

/* Includes ------------------------------------------------------------------*/
#include "config.h"
#include "stm32f4xx_hal.h"


/**
  * @brief TIM MSP Initialization for RFreceiver
  *        This function configures the hardware resources used in this example:
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration
  * @param htim: TIM handle pointer
  * @retval None
  */
void HAL_TIM_IC_MspInit(TIM_HandleTypeDef *htim)
{
  GPIO_InitTypeDef   GPIO_InitStruct;

  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* TIMx Peripheral clock enable */
  PWMinput_TIMx_CLK_ENABLE();

  /* Enable GPIO channels Clock */
  PWMinput_TIMx_CHANNEL_GPIO_PORT();

  /* Configure  (PWMinput_TIMx_Channel) in Alternate function, push-pull and 100MHz speed */
  GPIO_InitStruct.Pin = GPIO_PIN_CHANNEL2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF_PWMinput_TIMx;
  HAL_GPIO_Init(GPIO_PORT, &GPIO_InitStruct);

  /*##-2- Configure the NVIC for PWMinput_TIMx ########################################*/
  /* Set the PWMinput_TIMx priority */
  HAL_NVIC_SetPriority(PWMinput_TIMx_IRQn, 1, 1);

  /* Enable the PWMinput_TIMx global Interrupt */
  HAL_NVIC_EnableIRQ(PWMinput_TIMx_IRQn);
}
