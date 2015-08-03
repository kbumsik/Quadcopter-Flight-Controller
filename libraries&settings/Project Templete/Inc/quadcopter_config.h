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

/* definition for TM's library */
#ifndef STM32F4xx
	#define STM32F4xx
#endif

#endif /* DRONE_CONFIG_H_ */
