/*
 * drone_config.h
 *
 *  Created on: Jul 24, 2015
 *      Author: BumSik
 */

#ifndef DRONE_CONFIG_H_
#define DRONE_CONFIG_H_ 100

/* Basic CMSIS setting */
#ifndef STM32F411xE
	#define STM32F411xE
 	#ifndef __FPU_PRESENT
 		#define __FPU_PRESENT 1
	#endif
 	#ifndef ARM_MATH_CM4
 		#define ARM_MATH_CM4
 	#endif
#endif

/* definition for HAL Driver */
#ifndef USE_HAL_DRIVER
	#define USE_HAL_DRIVER
#endif

/* definition for NUCLEO Board */
#ifndef NUCLEO_F411RE
	#define NUCLEO_F411RE
 	#ifndef LED_PIN
		#define LED_PIN GPIO_PIN_5
		#define LED_PORT GPIOA
 	#endif
#endif

/* definition for TM's library */
#ifndef STM32F4xx
	#define STM32F4xx
#endif

#endif /* DRONE_CONFIG_H_ */
