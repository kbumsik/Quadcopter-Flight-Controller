/*
 * uart.c
 *
 *  Created on: Dec 30, 2015
 *      Author: Bumsik Kim
 */

/* includes -----------------------*/
#include "config.h"
#include "uart.h"
#include "cmsis_os.h"

/* Variables ---------------------*/

/* private */
static uint8_t pcInputBuffer[confUART_RECEIVE_BUFFER_SIZE]; /* input buffer */
static uint8_t ucInputIndex = 0; /* input count */

/* Private functions */
static void vUARTReceive(UART_HandleTypeDef *pxUART);

/**
  * @brief  This function handles UART interrupt request.
  * @param  huart: pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
void vUARTIRQHandler(UART_HandleTypeDef *pxUART)
{
  uint32_t tmp1 = 0, tmp2 = 0;

  tmp1 = __HAL_UART_GET_FLAG(pxUART, UART_FLAG_RXNE);
  tmp2 = __HAL_UART_GET_IT_SOURCE(pxUART, UART_IT_RXNE);
  /* UART in mode Receiver ---------------------------------------------------*/
  if((tmp1 != RESET) && (tmp2 != RESET))
    {
      vUARTReceive(pxUART);
    }
}

/**
  * @brief  Receives an amount of data in non blocking mode
  * @param  huart: pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval HAL status
  */
void vUARTReceive(UART_HandleTypeDef *pxUART)
{
  BaseType_t result;
  BaseType_t xHigherPriorityWoken;

  /* copy data */
  pcInputBuffer[ucInputIndex] = (uint8_t)(pxUART->Instance->DR & (uint8_t)0x00FF);


  /* if the received character is newline */
  if (pcInputBuffer[ucInputIndex] == (uint8_t)'\n')
  {
    /* put null */
    pcInputBuffer[ucInputIndex+1] = (uint8_t)'\0';
    /* Put in the queue */
    result = xQueueSendToBackFromISR(qUARTReceive, (void*) pcInputBuffer, &xHigherPriorityWoken);
    /* TODO: what if the Queue is full? */
    ucInputIndex = 0;
    portYIELD_FROM_ISR(xHigherPriorityWoken);
    return;
  }
  /* Increase the buffer index */
  ucInputIndex += 1;
  /* if the index buffer reaches the boundary */
  if (ucInputIndex == confUART_RECEIVE_BUFFER_SIZE-1)
  {
    /* put null */
    pcInputBuffer[ucInputIndex] = (uint8_t)'\0';
    /* TODO: What if the end of line is without \n? what happened to scanf()? */
    /* Put in the queue */
    result = xQueueSendToBackFromISR(qUARTReceive, (void*) pcInputBuffer, &xHigherPriorityWoken);
    /* TODO: what if the Queue is full? */
    ucInputIndex = 0;
    portYIELD_FROM_ISR(xHigherPriorityWoken);
    return;
  }
  return;
}
