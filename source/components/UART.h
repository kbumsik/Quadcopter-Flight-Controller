/*
 * uart.h
 *
 *  Created on: Dec 30, 2015
 *      Author: Bumsik Kim
 */

#ifndef UART_H_
#define UART_H_

/* Includes */
#include "components_common.h"
#include "config.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal_gpio.h"
#include "cmsis_os.h"

/**
 * @defgroup UART settings
 * @brief Pin and Peripheral settings for UART
 * @{
 */
#ifndef confUARTx
  /* Peripheral number definition */
  #define confUARTx     USART1
  /* Definition for USARTx clock resources */
  #define confUART_CLK_ENABLE()              __HAL_RCC_USART1_CLK_ENABLE();
  #define confUART_RX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
  #define confUART_TX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
  #define confUART_FORCE_RESET()             __HAL_RCC_USART1_FORCE_RESET()
  #define confUART_RELEASE_RESET()           __HAL_RCC_USART1_RELEASE_RESET()
  
  /* Definition for USARTx Pins */
  #define confUART_TX_PIN                    GPIO_PIN_15
  #define confUART_TX_GPIO_PORT              GPIOA  
  #define confUART_TX_AF                     GPIO_AF7_USART1
  #define confUART_RX_PIN                    GPIO_PIN_10
  #define confUART_RX_GPIO_PORT              GPIOA 
  #define confUART_RX_AF                     GPIO_AF7_USART1

  /* Definition for UART Initialization */
  #define confUART_BAUDRATE   115200
  #define confUART_WORDLENGTH UART_WORDLENGTH_8B
  #define confUART_STOPBITS   UART_STOPBITS_1
  #define confUART_PARITY     UART_PARITY_NONE

  /* Definition for UART Interrupt */
  #define confUART_IRQn                      USART1_IRQn
  #define confUART_IRQHandler                USART1_IRQHandler
#endif
/**
 * @}
 */

/* global variables */
extern UART_HandleTypeDef xUARTHandle;
extern QueueHandle_t qUARTReceive;

/* Function prototypes */
Status_t vUARTInit(UART_HandleTypeDef* pxUARTHandle, USART_TypeDef* pxUARTx);
Status_t vUARTGPIOInit(UART_HandleTypeDef* pxUARTHandle);
void vUARTIRQHandler(UART_HandleTypeDef *pxUARTHandle);

#endif /* UART_H_ */
