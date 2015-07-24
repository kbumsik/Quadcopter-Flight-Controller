/**
  @page ADC_RegularConversion_Interrupt conversion using Interrupt

  @verbatim
  ******************** (C) COPYRIGHT 2015 STMicroelectronics *******************
  * @file    ADC/ADC_RegularConversion_Interrupt/readme.txt 
  * @author  MCD Application Team
  * @version V1.2.1
  * @date    13-March-2015
  * @brief   Description of the ADC RegularConversion Interrupt example.
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

This example describes how to use the ADC3 and Interrupt to convert continuously 
data from ADC3 channel8.

The ADC3 is configured to convert continuously channel8(for STM324x9I-EVAL RevB).

Each time an end of conversion occurs an interrupt is generated and the converted 
data of ADC3 DR register is affected to the uhADCxConvertedValue variable in the 
ADC conversion complete call back function.

In this example, the system clock is 144MHz, APB2 = 72MHz and ADC clock = APB2/2. 
Since ADC3 clock is 36 MHz and sampling time is set to 3 cycles, the conversion 
time to 12bit data is 12 cycles so the total conversion time is (12+3)/36= 0.41us(2.4Msps).

User can vary the ADC3 channel8(for STM324x9I-EVAL RevB) voltage using the Eval Board potentiometer

STM32 Eval board's LEDs can be used to monitor the transfer status:
  - LED3 is ON when there are an error in initialization.

@note Care must be taken when using HAL_Delay(), this function provides accurate delay (in milliseconds)
      based on variable incremented in SysTick ISR. This implies that if HAL_Delay() is called from
      a peripheral ISR process, then the SysTick interrupt must have higher priority (numerically lower)
      than the peripheral interrupt. Otherwise the caller ISR process will be blocked.
      To change the SysTick interrupt priority you have to use HAL_NVIC_SetPriority() function.
      
@note The application needs to ensure that the SysTick time base is always set to 1 millisecond
      to have correct HAL operation.


@par Directory contents 

  - ADC/ADC_RegularConversion_Interrupt/Inc/stm32f4xx_hal_conf.h    HAL configuration file
  - ADC/ADC_RegularConversion_Interrupt/Inc/stm32f4xx_it.h          Interrupt handlers header file
  - ADC/ADC_RegularConversion_Interrupt/Inc/main.h                  Main program header file  
  - ADC/ADC_RegularConversion_Interrupt/Src/stm32f4xx_it.c          Interrupt handlers
  - ADC/ADC_RegularConversion_Interrupt/Src/main.c                  Main program
  - ADC/ADC_RegularConversion_Interrupt/Src/stm32f4xx_hal_msp.c     HAL MSP module
  - ADC/ADC_RegularConversion_Interrupt/Src/system_stm32f4xx.c      STM32F4xx system clock configuration file

@par Hardware and Software environment 

  - This example runs on STM32F429xx/STM32F439xx devices.
    
  - This example has been tested with STM324x9I-EVAL RevB evaluation board and can be
    easily tailored to any other supported device and development board.
  
  - STM324x9I-EVAL RevB Set-up
    - Use the Potentiometer (RV1) of the Eval board (connected to PF.10).


@par How to use it ? 

In order to make the program work, you must do the following :
 - Open your preferred toolchain 
 - Rebuild all files and load your image into target memory
 - Run the example

 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
