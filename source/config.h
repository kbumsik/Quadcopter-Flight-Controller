/**
  ******************************************************************************
  * @file    GPIO/GPIO_IOToggle/Inc/main.h 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    26-June-2014
  * @brief   Header for main.c module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/**
 * @brief defines for CMSIS and HAL
 */
#ifndef STM32F411xE
    #define STM32F411xE
#endif

#ifndef ARM_MATH_CM4
    #define ARM_MATH_CM4
#endif

#ifndef USE_HAL_DRIVER
    #define USE_HAL_DRIVER
#endif

/**
 * @brief defines for TM's library
 */
#ifndef STM32F4XX
    #define STM32F4XX
#endif
#ifndef STM32F4xx
    #define STM32F4xx
#endif


/* ########################## Module Selection ############################## */

/**
  * @brief This is the list of modules to be used in the HAL driver
  */
#define confPWMINPUT_ENABLED    1
#define confUART_ENABLED        1
#define confMOTOR_ENABLED       1
#define confI2C_ENABLED         1
#define confTIMER_ENABLED       1


/* ################### Board configuration ################################# */
/**
 * @brief include a board
 */

/* definition for NUCLEO Board */
/* Define only one board!!! */
#define INCLUDE_NUCLEO_F411RE
//#define INCLUDE_NUCLEO_F466RE
/* Add custom board here */

/**
 * @defgroup Board specific settings
 * @brief Defines board-specific settings
 * @{
 */
#define confMCU_CLOCK_MHZ 100
#define confNUCLEO_USE_STLINK_COM_PORT_FOR_UART
 /**
  * @}
  */

/* ###################### Extra configuration ############################## */
#define confUART_RECEIVE_QUEUE_LENGTH 3 /* length of input buffer */
#define confUART_RECEIVE_BUFFER_SIZE 30 /* size of input buffer */

/**
 * @defgroup Remote controller settings
 * @brief Settings for Remote controller like duty cycle of the controller
 * @{
 */
#define confREMOTE_NUMBER_OF_CHANNEL    4

/* This will be used for array index */
#define confREMOTE_MIN    0
#define confREMOTE_CENTER 1
#define confREMOTE_MAX    2

#define confREMOTE_CH1_MIN      1246
#define confREMOTE_CH1_CENTER   1666
#define confREMOTE_CH1_MAX      2031

#define confREMOTE_CH2_MIN      1157
#define confREMOTE_CH2_CENTER   1524
#define confREMOTE_CH2_MAX      1899

#define confREMOTE_CH3_MIN      1108
#define confREMOTE_CH3_CENTER   1523
#define confREMOTE_CH3_MAX      1915

#define confREMOTE_CH4_MIN      1071
#define confREMOTE_CH4_CENTER   1511
#define confREMOTE_CH4_MAX      1955
/**
 * @}
 */


/* inclues ------------------------------------------------------------------*/

/**
 * @brief include board header files
 */
#ifdef INCLUDE_NUCLEO_F411RE
  #include "config_NUCLEO_F411RE.h"
#endif

/**
 * @brief Include basic files
 */
#include <stdint.h>
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "FreeRTOSConfig.h"


/**
  * @brief Include module's header file
  */

#ifdef confPWMINPUT_ENABLED
  #include "PWMInput.h"
#endif

#ifdef confUART_ENABLED
  #include "UART.h"
#endif

#ifdef confMOTOR_ENABLED
  #include "Motor.h"
#endif

#ifdef confI2C_ENABLED
#endif

#ifdef confTIMER_ENABLED
  #include "timer.h"
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
