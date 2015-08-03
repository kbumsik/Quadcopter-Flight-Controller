/*
 * kb_stm32_hal_motor.h
 *
 *  Created on: Aug 1, 2015
 *      Author: BumSik
 *      
 * -----------------------------------------------------------------------
 *TIM2 Configuration: generate 4 PWM signals with 4 different duty cycles.
 *  In this example TIM2 input clock (TIM2CLK) is set to 2 * APB1 clock (PCLK1),
 *  since APB1 prescaler is different from 1.
 *    TIM2CLK = 2 * PCLK1
 *    PCLK1 = HCLK 
 *    => TIM2CLK = HCLK = SystemCoreClock
 *  To get TIM2 counter clock at 50 MHz, the prescaler is computed as follows:
 *     Prescaler = (TIM2CLK / TIM2 counter clock) - 1
 *     Prescaler = ((SystemCoreClock) /50 MHz) - 1
 *               = 2
 *  To get TIM2 output clock at 500 Hz, the period (ARR) is computed as follows:
 *     ARR = (TIM2 counter clock / TIM2 output clock) - 1
 *         = 100,000
 *  Note:
 *   SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f4xx.c file.
 *   Each time the core clock (HCLK) changes, user had to update SystemCoreClock
 *   variable value. Otherwise, any configuration based on this variable will be incorrect.
 *   This variable is updated in three ways:
 *    1) by calling CMSIS function SystemCoreClockUpdate()
 *    2) by calling HAL API function HAL_RCC_GetSysClockFreq()
 *    3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency
 *-----------------------------------------------------------------------
 * *GPIO Pin Configuration: 
 *     Channel 1: PA0
 *     Channel 2: PA1
 *     Channel 3: PA2
 *     Channel 4: PA3
 *-----------------------------------------------------------------------
 */

#ifndef KB_STM32_HAL_MOTOR_H_
#define KB_STM32_HAL_MOTOR_H_ 100


#endif /* KB_STM32_HAL_MOTOR_H_ */

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal_gpio.h"

/* User can use this section to tailor TIMx instance used and associated
   resources */
/* Definition for TIMx clock resources */
#define MOTOR_TIMx							TIM2
#define MOTOR_TIMx_CLK_ENABLE()				__HAL_RCC_TIM2_CLK_ENABLE()
#define MOTOR_TIMx_CLK_DISABLE()			__HAL_RCC_TIM2_CLK_DISABLE()

/* Prescaler Definition at f = 50MHz */
#define MOTOR_TIM_PRESCALER					(((SystemCoreClock) /50000000) - 1)

/* Definition for TIMx Channel Pins */
#define MOTOR_GPIO_PORT 					GPIOA
#define MOTOR_TIMx_CHANNEL_GPIO_PORT()		__HAL_RCC_GPIOA_CLK_ENABLE()
#define MOTOR_GPIO_ALTERNATE				GPIO_AF1_TIM2
#define MOTOR_GPIO_PIN_CHANNEL_1			GPIO_PIN_1
#define MOTOR_GPIO_PIN_CHANNEL_2			GPIO_PIN_2
#define MOTOR_GPIO_PIN_CHANNEL_3			GPIO_PIN_3
#define MOTOR_GPIO_PIN_CHANNEL_4			GPIO_PIN_4

/* Definition for PWM pulse */
#define MOTOR_PERIOD_VALUE		(100000 - 1)  /* Period Value  */

/* Motor Maxium speed and limit */
#define MOTOR_SPEED_MAX		(100000)
/* 80% of the maximum speed */
#define MOTOR_SPEED_LIMIT	(80000)

/**
 * Typedef
 */
 typedef enum{
 	KB_STM32_OK,
 	KB_STM32_Error
 }KB_STM32_Status_t;

typedef enum{
	KB_STM32_Motor_Channel_1,
	KB_STM32_Motor_Channel_2,
	KB_STM32_Motor_Channel_3,
	KB_STM32_Motor_Channel_4
}KB_STM32_Motor_Channel_t;

/**
 * Function delaration
 */

 /**
  * @brief      Initialize motor
  *
  * @return     Status of the result
  */
KB_STM32_Status_t KB_STM32_Motor_Init(void);

/**
 * @brief      Set the speed of a motor
 *
 * @param[in]  speed    speed of the motor the user want to set.
 * 						This value cannot exceed MOTOR_SPEED_LIMIT	
 * @param[in]  channel  The channel of the motor.
 *
 * @return     Status of the result
 */		
KB_STM32_Status_t KB_STM32_Motor_SetSpeed(int32_t speed, KB_STM32_Motor_Channel_t channel);

/**
 * @brief      Start rotating the motors
 *
 * @return     Status of the result
 */
KB_STM32_Status_t KB_STM32_Motor_Start(void);

/**
 * @brief      Stop rotation the motors
 *
 * @return     Status of the result
 */
KB_STM32_Status_t KB_STM32_Motor_Stop(void);

/**
 * @brief      Deinitialize the motors
 *
 * @return     Status of the result
 */
KB_STM32_Status_t KB_STM32_Motor_DeInit(void);
