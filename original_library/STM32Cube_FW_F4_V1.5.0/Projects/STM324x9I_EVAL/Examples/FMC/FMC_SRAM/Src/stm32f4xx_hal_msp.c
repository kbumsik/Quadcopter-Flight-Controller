/**
  ******************************************************************************
  * @file    FMC/FMC_SRAM/Src/stm32f4xx_hal_msp.c
  * @author  MCD Application Team
  * @version V1.2.1
  * @date    13-March-2015
  * @brief   HAL MSP module.    
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************  
  */ 

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/** @addtogroup STM32F4xx_HAL_Examples
  * @{
  */

/** @defgroup FMC_SRAM_Basic
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @defgroup HAL_MSP_Private_Functions
  * @{
  */

/**
  * @brief SRAM MSP Initialization 
  *        This function configures the hardware resources used in this example: 
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration  
  * @param hsram: SRAM handle pointer
  * @retval None
  */
void HAL_SRAM_MspInit(SRAM_HandleTypeDef *hsram)
{
  GPIO_InitTypeDef GPIO_Init_Structure;

  /* Enable FMC clock */
  __HAL_RCC_FMC_CLK_ENABLE();
  
  /* Enable GPIOs clock */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  
  /* Common GPIO configuration */
  GPIO_Init_Structure.Mode      = GPIO_MODE_AF_PP;
  GPIO_Init_Structure.Pull      = GPIO_PULLUP;
  GPIO_Init_Structure.Speed     = GPIO_SPEED_HIGH;
  GPIO_Init_Structure.Alternate = GPIO_AF12_FMC;
  
  /* GPIOD configuration */
  GPIO_Init_Structure.Pin   = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_8     |\
                              GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 |\
                              GPIO_PIN_14 | GPIO_PIN_15;   
  HAL_GPIO_Init(GPIOD, &GPIO_Init_Structure);

  /* GPIOE configuration */  
  GPIO_Init_Structure.Pin   = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_3| GPIO_PIN_4 | GPIO_PIN_7     |\
                              GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 |\
                              GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  HAL_GPIO_Init(GPIOE, &GPIO_Init_Structure);
  
  /* GPIOF configuration */  
  GPIO_Init_Structure.Pin   = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2| GPIO_PIN_3 | GPIO_PIN_4     |\
                              GPIO_PIN_5 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  HAL_GPIO_Init(GPIOF, &GPIO_Init_Structure);
  
  /* GPIOG configuration */  
  GPIO_Init_Structure.Pin   = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2| GPIO_PIN_3 | GPIO_PIN_4 |\
                              GPIO_PIN_5 | GPIO_PIN_9;
  HAL_GPIO_Init(GPIOG, &GPIO_Init_Structure);
    
}

/**
  * @brief SRAM MSP De-Initialization 
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  *          - Revert GPIO configuration to their default state
  * @param hsram: SRAM handle pointer
  * @retval None
  */
void HAL_SRAM_MspDeInit(SRAM_HandleTypeDef *hsram)
{
  /*## Disable peripherals and GPIO Clocks ###################################*/
  /* Configure FMC as alternate function  */
  HAL_GPIO_DeInit(GPIOD, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_3| GPIO_PIN_4 | GPIO_PIN_5     |\
                         GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 |\
                         GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);
  
  HAL_GPIO_DeInit(GPIOE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_3| GPIO_PIN_4 | GPIO_PIN_7     |\
                         GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 |\
                         GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);
                  
  HAL_GPIO_DeInit(GPIOF, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2| GPIO_PIN_3 | GPIO_PIN_4     |\
                         GPIO_PIN_5 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);
 
  HAL_GPIO_DeInit(GPIOG, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2| GPIO_PIN_3 | GPIO_PIN_4 |\
                         GPIO_PIN_5 | GPIO_PIN_9);
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
