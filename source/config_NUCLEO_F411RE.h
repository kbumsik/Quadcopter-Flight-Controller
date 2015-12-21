/*
 * config_NUCLEO_F411RE.h
 *
 *  Created on: Dec 21, 2015
 *      Author: Bumsik Kim
 */

#ifndef CONFIG_NUCLEO_F411RE_H_
#define CONFIG_NUCLEO_F411RE_H_

#define portLED_0_PIN	GPIO_PIN_5
#define portLED_0_PORT	GPIOA

#ifdef USE_HAL_DRIVER
	#define vLED_0_Toggle()	HAL_GPIO_TogglePin(portLED_0_PORT, portLED_0_PIN)
	#define vLED_0_On()		HAL_GPIO_WritePin(portLED_0_PORT, portLED_0_PIN, GPIO_PIN_SET)
	#define vLED_0_Off()	HAL_GPIO_WritePin(portLED_0_PORT, portLED_0_PIN, GPIO_PIN_RESET)
#else
	#error "Not using HAL so no LED pins are defined!"
#endif

#endif /* CONFIG_NUCLEO_F411RE_H_ */
