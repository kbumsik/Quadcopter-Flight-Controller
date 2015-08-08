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
#ifndef STM32F411xE
	#define STM32F411xE
#endif

/* definition for HAL Driver */
#ifndef USE_HAL_DRIVER
	#define USE_HAL_DRIVER
#endif

/* definition for NUCLEO Board */
#ifndef NUCLEO_F411RE
	#define NUCLEO_F411RE
#endif

#ifndef LED_PIN
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
#ifndef STM32F4XX
	#define STM32F4XX
#endif
#ifndef STM32F4xx
	#define STM32F4xx
#endif

/**
 * @defgroup NRF24L01+ Settings
 * @brief    Pin settings for NRF24L01+
 * @verbatim
NRF24L01+	STM32Fxxx	DESCRIPTION

GND			GND			Ground
VCC			3.3V		3.3V
CE			PC15		RF activated pin
CSN			PC14		Chip select pin for SPI
SCK			PC10		SCK pin for SPI
MOSI		PC12		MOSI pin for SPI
MISO		PC11		MISO pin for SPI
IRQ			Not used(PD2)	Interrupt pin. Goes low when active. Pin functionality is active, but not used in library
@endverbatim
 * @{
 */
/* Default SPI used */
#define NRF24L01_SPI				SPI3
#define NRF24L01_SPI_PINS			TM_SPI_PinsPack_2

/* SPI chip enable pin */
#define NRF24L01_CSN_PORT				GPIOC
#define NRF24L01_CSN_PIN				GPIO_PIN_2
#define NRF24L01_CSN_PORT_CLK_ENABLE	__HAL_RCC_GPIOC_CLK_ENABLE

/* Chip enable for transmitting */
#define NRF24L01_CE_PORT				GPIOC
#define NRF24L01_CE_PIN					GPIO_PIN_3
#define NRF24L01_CE_PORT_CLK_ENABLE		__HAL_RCC_GPIOC_CLK_ENABLE
/**
 * @}
 */

#endif /* DRONE_CONFIG_H_ */
