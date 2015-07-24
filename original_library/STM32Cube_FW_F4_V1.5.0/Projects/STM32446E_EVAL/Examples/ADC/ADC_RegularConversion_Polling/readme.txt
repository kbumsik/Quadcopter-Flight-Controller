/**
  @page ADC_RegularConversion_Polling conversion using Polling

  @verbatim
  ******************** (C) COPYRIGHT 2015 STMicroelectronics *******************
  * @file    ADC/ADC_RegularConversion_Polling/readme.txt 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    13-March-2015
  * @brief   Description of the ADC RegularConversion Polling example.
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

This example describes how to use the ADC1 with channel ADC_CHANNEL_4 in Polling mode to convert data.

When the end of conversion occurs, the converted data of ADC1 DR register is 
affected to the uhADCxConvertedValue variable.

Conversion time (reference manual, "Reset and clock control" and "ADC clocks" sections):
  1) the system clock is 180 MHz. APB2 = 90MHz and ADC clock = APB2/2
  2) __HAL_RCC_ADC_CONFIG() macro in HAL_ADC_MspInit() API sets the system clock as ADC asynchronous clock source 
  2) ClockPrescaler field of initialization structure is set to ADC_CLOCKPRESCALER_PCLK_DIV2
     => the input ADC clock is set in asynchronous clock mode   
     => the input ADC clock is the undivided system clock 


  Sampling time is set to ADC_SAMPLETIME_6CYCLES_5 (6.5 cycles).
  ConvTime = Sampling time + 12.5 ADC clock cycles.
           = 19 clock cycles
           = (19 / 45 MHz) = 422.2 ns 
                    

User can vary the ADC_CHANNEL_4 voltage using the Eval Board potentiometer (CN21) connected to PA.04.
SB94 must be closed to use the potentiometer.

STM32 Eval board's LEDs can be used to monitor the transfer status:
  - LED3 is ON when there is an error in initialization.

@par Directory contents 

  - ADC/ADC_RegularConversion_Polling/Inc/stm32f4xx_hal_conf.h    HAL configuration file
  - ADC/ADC_RegularConversion_Polling/Inc/stm32f4xx_it.h          Interrupt handlers header file
  - ADC/ADC_RegularConversion_Polling/Inc/main.h                  Header for main.c module  
  - ADC/ADC_RegularConversion_Polling/Src/stm32f4xx_it.c          Interrupt handlers
  - ADC/ADC_RegularConversion_Polling/Src/main.c                  Main program
  - ADC/ADC_RegularConversion_Polling/Src/stm32f4xx_hal_msp.c     HAL MSP file 
  - ADC/ADC_RegularConversion_Polling/Src/system_stm32f4xx.c      STM32F4xx system source file

@par Hardware and Software environment 

  - This example runs on STM32F446xx devices.
  
  - This example has been tested with STM32446E-EVAL board revB and can be
    easily tailored to any other supported device and development board.

@par How to use it ? 

In order to make the program work, you must do the following :
 - Open your preferred toolchain 
 - Rebuild all files and load your image into target memory
 - Run the example

 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
