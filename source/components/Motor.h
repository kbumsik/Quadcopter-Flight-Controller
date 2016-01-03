/*
 * kb_stm32_hal_motor.h
 *
 *  Created on: Aug 1, 2015
 *      Author: BumSik
 *      
 * -----------------------------------------------------------------------
 *TIM4 Configuration: generate 4 PWM signals with 4 different duty cycles.
 *  In this example TIM4 input clock (TIM4CLK) is set to 2 * APB1 clock (PCLK1),
 *  since APB1 prescaler is different from 1.
 *    TIM4CLK = 2 * PCLK1
 *    PCLK1 = HCLK / 4
 *    => TIM4CLK = HCLK / 2 = SystemCoreClock / 2
 *  To get TIM4 counter clock at 1 MHz, the prescaler is computed as follows:
 *     Prescaler = (TIM4CLK / TIM4 counter clock) - 1
 *     Prescaler = (((SystemCoreClock)/2) /1 MHz) - 1
 *               = 49
 *  To get TIM4 output clock at 50 Hz, the period (ARR) is computed as follows:
 *     ARR = (TIM4 counter clock / TIM4 output clock) - 1
 *         = 19,999
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

#include "components_common.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal_gpio.h"

/* User can use this section to tailor TIMx instance used and associated
   resources */
/* Definition for TIMx clock resources */
#define MOTOR_TIMx							TIM4
#define MOTOR_TIMx_CLK_ENABLE()				__HAL_RCC_TIM4_CLK_ENABLE()
#define MOTOR_TIMx_CLK_DISABLE()			__HAL_RCC_TIM4_CLK_DISABLE()

/* Prescaler Definition at f = 50MHz */
#define MOTOR_TIM_PRESCALER					(((SystemCoreClock)/2000000) - 1)

/* Definition for TIMx Channel Pins */
#define MOTOR_GPIO_PORT 					GPIOB
#define MOTOR_TIMx_CHANNEL_GPIO_PORT()		__HAL_RCC_GPIOB_CLK_ENABLE()
#define MOTOR_GPIO_ALTERNATE				GPIO_AF2_TIM4
#define MOTOR_GPIO_PIN_CHANNEL_1			GPIO_PIN_6
#define MOTOR_GPIO_PIN_CHANNEL_2			GPIO_PIN_7
#define MOTOR_GPIO_PIN_CHANNEL_3			GPIO_PIN_8
#define MOTOR_GPIO_PIN_CHANNEL_4			GPIO_PIN_9

/* Definition for PWM pulse */
#define MOTOR_PERIOD_VALUE		(19999 - 1)  /* Period Value  */

/* Motor Maxium and minimum speed */
#define MOTOR_SPEED_MAX		(2000)
#define MOTOR_SPEED_MIN		(1000)

/**
 * Typedef
 */

typedef enum{
  MOTOR_CHANNEL_1,
  MOTOR_CHANNEL_2,
  MOTOR_CHANNEL_3,
  MOTOR_CHANNEL_4,
  MOTOR_CHANNEL_ALL
}MotorChannel_t;

/* External Variables */
extern TIM_HandleTypeDef xMotorHandle;

#ifdef __cplusplus
 extern "C" {
#endif
/**
 * Function delaration
 */

/**
 * @brief      Initialize motor
 *
 * @return     Status of the result
 */
Status_t xMotorInit(TIM_HandleTypeDef* pxTIMHandle);

/**
 * @brief      Initialize all configured GPIO Pins
 */
void vMotorGPIOInit(TIM_HandleTypeDef* pxTIMHandle);

/**
 * @brief      Set the speed of a motor
 *
 * @param[in]  speed    speed of the motor the user want to set.
 * 						This value cannot exceed MOTOR_SPEED_LIMIT	
 * @param[in]  channel  The channel of the motor.
 *
 * @return     speed set to the motor
 */
// FIXME:Seems like it needs xMotorStart again after this function. Make it that function is not need
int slMotorSetSpeed(TIM_HandleTypeDef* pxTIMHandle, int speed, MotorChannel_t channel);

/**
 * @brief      Start rotating the motors
 *
 * @return     Status of the result
 */
Status_t xMotorStart(TIM_HandleTypeDef* pxTIMHandle, MotorChannel_t xChannel);

/**
 * @brief      Stop rotation the motors
 *
 * @return     Status of the result
 */
Status_t xMotorStop(TIM_HandleTypeDef* pxTIMHandle, MotorChannel_t xChannel);

/**
 * @brief      Deinitialize the motors
 *
 * @return     Status of the result
 */
Status_t xMotorDeInit(TIM_HandleTypeDef* pxTIMHandle);

#ifdef __cplusplus
}
#endif

#endif /* KB_STM32_HAL_MOTOR_H_ */
