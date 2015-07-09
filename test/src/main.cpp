/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/


#define STM32F411xE
#define USE_HAL_DRIVER

#include <stdint.h>
#include "stm32f4xx.h"
#include "stm32f4xx_nucleo.h"

/* test blinky */
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_rcc.h"
#define LD2_PORT GPIOA
#define LD2_PIN GPIO_PIN_5

/* RF24 */
#include "RF24.h"

static GPIO_InitTypeDef GPIO_InitStruct;
int main(void)
{

	/* test blinky */
	HAL_Init();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	GPIO_InitStruct.Pin = LD2_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
	HAL_GPIO_Init(LD2_PORT, &GPIO_InitStruct);
	HAL_GPIO_WritePin(LD2_PORT,LD2_PIN,GPIO_PIN_SET);
	while(1)
	{
		HAL_Delay(1000);
		HAL_GPIO_WritePin(LD2_PORT,LD2_PIN,GPIO_PIN_RESET);
		HAL_Delay(1000);
		HAL_GPIO_WritePin(LD2_PORT,LD2_PIN,GPIO_PIN_SET);
	}
}
