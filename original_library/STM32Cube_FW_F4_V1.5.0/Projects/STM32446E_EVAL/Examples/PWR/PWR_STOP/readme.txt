/**
  @page PWR_STOP Power Stop Mode Example
  
  @verbatim
  ******************** (C) COPYRIGHT 2015 STMicroelectronics *******************
  * @file    PWR/PWR_STOP/readme.txt 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    13-March-2015
  * @brief   Description of the Power Stop Mode example.
  ******************************************************************************
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
  @endverbatim

@par Example Description 
 
This example shows how to enter the system to STOP mode and wake-up from this
mode using Key push button EXTI15_10.

In the associated software
  - the system clock is set to 180 MHz.
  - the EXTI15_10 is configured to generate an interrupt on falling edge.
  - the SysTick is programmed to generate an interrupt each 1 ms. 

The system enters STOP mode after 5s and will wait for the User push-button is pressed
to wake up from STOP mode. Current consumption could be monitored through an ampere-meter.
After the system woken up from STOP, the clock system is reconfigured because the default clock 
after wake up is the HSI. 

LED1 is used to monitor the system state as following:
- LED1 is OFF: System is in STOP mode.
- LED1 is ON: System is in RUN mode.

This behavior is repeated in an infinite loop.
 
    - STOP Mode
    ============ 
          - Regulator in LP mode
          - VREFINT OFF with fast wakeup enabled
          - HSI as SysClk after Wake Up
          - No IWDG
          - Wakeup using EXTI Line (User push-button PC.13)

@note This example can not be used in DEBUG mode, this is due to the fact 
      that the Cortex-M4 core is no longer clocked during low power mode 
      so debugging features are disabled

@note Care must be taken when using HAL_Delay(), this function provides accurate delay (in milliseconds)
      based on variable incremented in SysTick ISR. This implies that if HAL_Delay() is called from
      a peripheral ISR process, then the SysTick interrupt must have higher priority (numerically lower)
      than the peripheral interrupt. Otherwise the caller ISR process will be blocked.
      To change the SysTick interrupt priority you have to use HAL_NVIC_SetPriority() function.
      
@note The application need to ensure that the SysTick time base is always set to 1 millisecond
      to have correct HAL operation.
      
@note  Care must be taken when HAL_RCCEx_PeriphCLKConfig() is used to select the RTC clock source; in this 
       case the Backup domain will be reset in order to modify the RTC Clock source, as consequence RTC  
       registers (including the backup registers) and RCC_BDCR register are set to their reset values.

@par Directory contents 

  - PWR/PWR_STOP/Inc/stm32f4xx_conf.h         HAL Configuration file
  - PWR/PWR_STOP/Inc/stm32f4xx_it.h           Header for stm32f4xx_it.c
  - PWR/PWR_STOP/Inc/main.h                   Header file for main.c
  - PWR/PWR_STOP/Src/system_stm32f4xx.c       STM32F4xx system clock configuration file
  - PWR/PWR_STOP/Src/stm32f4xx_it.c           Interrupt handlers
  - PWR/PWR_STOP/Src/main.c                   Main program
  - PWR/PWR_STOP/Src/stm32f4xx_hal_msp.c      HAL MSP module

@par Hardware and Software environment

  - This example runs on STM32F446xx devices.
  
  - This example has been tested with STMicroelectronics STM32446E-EVAL 
    evaluation boards and can be easily tailored to any other supported device 
    and development board.   
      
  - STM32446E-EVAL Set-up
    - Use LED1 connected to PB11 pin 
    - Use the User push-button connected to pin PC.13 (EXTI15_10)


@par How to use it ? 

In order to make the program work, you must do the following :
 - Open your preferred toolchain 
 - Rebuild all files and load your image into target memory
 - Run the example