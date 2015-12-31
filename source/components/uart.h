/*
 * uart.h
 *
 *  Created on: Dec 30, 2015
 *      Author: Bumsik Kim
 */

#ifndef UART_H_
#define UART_H_

/* Includes */
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

/* global variables */
extern UART_HandleTypeDef xUARTHandle;
extern QueueHandle_t qUARTReceive;

/* Function prototypes */
void vUARTInit(UART_HandleTypeDef* pxUARTHandle, USART_TypeDef* pxUARTx);
void vUARTIRQHandler(UART_HandleTypeDef *pxUARTHandle);



#endif /* UART_H_ */
