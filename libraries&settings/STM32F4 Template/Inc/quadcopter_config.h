/*
 * drone_config.h
 *
 *  Created on: Jul 24, 2015
 *      Author: BumSik
 */

#ifndef DRONE_CONFIG_H_
#define DRONE_CONFIG_H_ 100

/* Basic CMSIS setting */
//#define STM32
//#define STM32F4
//#define STM32F411RETx
//#define STM32F411xE

/* definition for HAL Driver */
#ifndef USE_HAL_DRIVER
	#define USE_HAL_DRIVER
#endif

/* definition for NUCLEO Board */
#ifndef NUCLEO_F411RE
	#define NUCLEO_F411RE
	#define LED_PIN GPIO_PIN_5
	#define LED_PORT GPIOA
#endif

#ifdef USE_HAL_DRIVER
	#ifdef LED_PIN
		#ifdef LED_PORT
			#define LED_TOGGLE()	HAL_GPIO_TogglePin(LED_PORT, LED_PIN)
			#define LED_ON()		HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET)
			#define LED_OFF()		HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET)
#endif
#endif
#endif
/* definition for TM's library */
#ifndef STM32F4xx
	#define STM32F4xx
#endif

#endif /* DRONE_CONFIG_H_ */
