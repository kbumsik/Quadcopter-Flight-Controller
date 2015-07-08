/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/

#include <stdint.h>
#include "stm32f4xx.h"
#include "stm32f4xx_nucleo.h"
			

#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_rcc.h"

static GPIO_InitTypeDef GPIO_InitStruct;

/* LED Setting */
#define LED_PIN GPIO_PIN_5
#define LED_PORT GPIOA

int main(void)
{

	HAL_Init();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	GPIO_InitStruct.Pin = LED_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
	HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);
	HAL_GPIO_WritePin(LED_PORT,LED_PIN,GPIO_PIN_SET);
	while(1)
	{
		HAL_Delay(1000);
		HAL_GPIO_WritePin(LED_PORT,LED_PIN,GPIO_PIN_RESET);
		HAL_Delay(1000);
		HAL_GPIO_WritePin(LED_PORT,LED_PIN,GPIO_PIN_SET);
	}
}
