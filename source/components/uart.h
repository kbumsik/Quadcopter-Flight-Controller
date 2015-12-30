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
extern QueueHandle_t qUARTReceive;

/* Function prototypes */
void vUARTIRQHandler(UART_HandleTypeDef *pxUART);



#endif /* UART_H_ */
