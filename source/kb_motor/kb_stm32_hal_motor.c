/*
 * kb_stm32_hal_motor.c
 *
 *  Created on: Aug 1, 2015
 *      Author: BumSik
 */


#include "kb_stm32_hal_motor.h"


/* Private variables ---------------------------------------------------------*/
/* Timer handler declaration */
extern TIM_HandleTypeDef	TimHandle;

/* Timer Output Compare Configuration Structure declaration */
static TIM_OC_InitTypeDef	sConfig;

/* Counter Prescaler value */
static uint32_t uhPrescalerValue;

/* private functions decleration*/
static void GPIO_Init(void);

/**
 * Function Implementations
 */
KB_STM32_Status_t
KB_STM32_Motor_Init(void){
	/* Enable GPIO Pin for alterative function */
	GPIO_Init();

	/* Enable MOTOR_TIMx Clock */
	MOTOR_TIMx_CLK_ENABLE();

	/* Compute the prescaler value to have TIM3 counter clock equal to 21 MHz */

	/* Initialize MOTOR_TIMx peripheral as follow:
	     + Prescaler = ((SystemCoreClock) /50 MHz) - 1 = TIM_PRESCALER
	     + Period = PERIOD_VALUE
	     + ClockDivision = 0
	     + Counter direction = Up
	*/
	uhPrescalerValue = (uint32_t)MOTOR_TIM_PRESCALER;
	TimHandle.Instance = MOTOR_TIMx;
	TimHandle.Init.Prescaler = uhPrescalerValue;
	TimHandle.Init.Period = MOTOR_PERIOD_VALUE;
	TimHandle.Init.ClockDivision = 0;
	TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
	if(HAL_TIM_PWM_Init(&TimHandle) != HAL_OK)
	{
		/* Initialization Error */
		return KB_STM32_Error;
	}
	return KB_STM32_OK;
}

KB_STM32_Status_t
KB_STM32_Motor_SetSpeed(
	int speed,
	KB_STM32_Motor_Channel_t channel)
{
	/* Verify the speed value */
	if (speed >= MOTOR_SPEED_MAX)
	{
		speed = MOTOR_SPEED_MAX;
	}
	else if (speed < MOTOR_SPEED_MIN)
	{
		speed = MOTOR_SPEED_MIN;
	}

	/*##-2- Configure the PWM channels #########################################*/
	/* Common configuration for all channels */
	sConfig.OCMode = TIM_OCMODE_PWM1;
	sConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfig.OCFastMode = TIM_OCFAST_DISABLE;

	switch(channel)
	{
		
	case KB_STM32_Motor_Channel_1:
		/* Set the pulse value for channel 1 */
		sConfig.Pulse = speed;
		if(HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_1) != HAL_OK)
		{
			/* Configuration Error */
			return KB_STM32_Error;
		}
	break;
	case KB_STM32_Motor_Channel_2:
		/* Set the pulse value for channel 2 */
		sConfig.Pulse = speed;
		if(HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_2) != HAL_OK)
		{
			/* Configuration Error */
			return KB_STM32_Error;
		}
	break;
	case KB_STM32_Motor_Channel_3:
		/* Set the pulse value for channel 3 */
		sConfig.Pulse = speed;
		if(HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_3) != HAL_OK)
		{
			/* Configuration Error */
			return KB_STM32_Error;
		}
	break;
	case KB_STM32_Motor_Channel_4:
		/* Set the pulse value for channel 4 */
		sConfig.Pulse = speed;
		if(HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_4) != HAL_OK)
		{
			/* Configuration Error */
			return KB_STM32_Error;
		}
	break;
	default:
		/* Return Error in other cases */
		return KB_STM32_Error;
	}

	/* return OK */
	return KB_STM32_OK;
}

KB_STM32_Status_t
KB_STM32_Motor_Start(void)
{
	/*##- Start PWM signals generation #######################################*/
	/* Start channel 1 */
	if(HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_1) != HAL_OK)
	{
		/* PWM Generation Error */
		return KB_STM32_Error;
	}
	/* Start channel 2 */
	if(HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_2) != HAL_OK)
	{
		/* PWM Generation Error */
		return KB_STM32_Error;
	}
	/* Start channel 3 */
	if(HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_3) != HAL_OK)
	{
		/* PWM Generation Error */
		return KB_STM32_Error;
	}
	/* Start channel 4 */
	if(HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_4) != HAL_OK)
	{
		/* PWM Generation Error */
		return KB_STM32_Error;
	}

	/* Return OK */
	return KB_STM32_OK;
}

KB_STM32_Status_t
KB_STM32_Motor_Stop(void)
{
	/*##- Stop PWM signals generation #######################################*/
	/* Start channel 1 */
	if(HAL_TIM_PWM_Stop(&TimHandle, TIM_CHANNEL_1) != HAL_OK)
	{
		/* PWM Generation Error */
		return KB_STM32_Error;
	}
	/* Start channel 2 */
	if(HAL_TIM_PWM_Stop(&TimHandle, TIM_CHANNEL_2) != HAL_OK)
	{
		/* PWM Generation Error */
		return KB_STM32_Error;
	}
	/* Start channel 3 */
	if(HAL_TIM_PWM_Stop(&TimHandle, TIM_CHANNEL_3) != HAL_OK)
	{
		/* PWM Generation Error */
		return KB_STM32_Error;
	}
	/* Start channel 4 */
	if(HAL_TIM_PWM_Stop(&TimHandle, TIM_CHANNEL_4) != HAL_OK)
	{
		/* PWM Generation Error */
		return KB_STM32_Error;
	}

	/* Return OK */
	return KB_STM32_OK;
}

KB_STM32_Status_t
KB_STM32_Motor_DeInit(void)
{
	if(HAL_TIM_PWM_DeInit(&TimHandle) != HAL_OK)
	{
		/* Initialization Error */
		return KB_STM32_Error;
	}

	/* Enable MOTOR_TIMx Clock */
	MOTOR_TIMx_CLK_DISABLE();

	/* Return OK */
	return KB_STM32_OK;
}


/**
 * Private Function Implementations
 */

/**
 * @brief      Initialize all configured GPIO Pins
 */
static void
GPIO_Init(void){
	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	MOTOR_TIMx_CHANNEL_GPIO_PORT();

	/* GPIO Setting for PWM */
	GPIO_InitStruct.Pin = (MOTOR_GPIO_PIN_CHANNEL_1|
						MOTOR_GPIO_PIN_CHANNEL_2|
						MOTOR_GPIO_PIN_CHANNEL_3|
						MOTOR_GPIO_PIN_CHANNEL_4);
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
	GPIO_InitStruct.Alternate = MOTOR_GPIO_ALTERNATE;
	HAL_GPIO_Init(MOTOR_GPIO_PORT, &GPIO_InitStruct);
}
