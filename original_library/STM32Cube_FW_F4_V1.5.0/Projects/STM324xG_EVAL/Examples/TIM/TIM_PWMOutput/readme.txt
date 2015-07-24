/**
  @page TIM_PWMOutput TIM PWM Output example

  @verbatim
  ******************** (C) COPYRIGHT 2015 STMicroelectronics *******************
  * @file    TIM/TIM_PWMOutput/readme.txt 
  * @author  MCD Application Team
  * @version V1.2.1
  * @date    13-March-2015
  * @brief   Description of the PWM signals generation using Timer3 example.
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

This example shows how to configure the TIM peripheral in PWM (Pulse Width Modulation) 
mode.

The TIM3CLK frequency is set to SystemCoreClock / 2 (Hz), to get TIM3 counter
clock at 21 MHz the Prescaler is computed as following:
   - Prescaler = (TIM3CLK / TIM3 counter clock) - 1

SystemCoreClock is set to 168 MHz for STM32F4xx Devices.

The TIM3 generates 4 PWM signals at 31.530 KHz:

    TIM3_Output Frequency = TIM3 counter clock/(ARR + 1)
                          = 21 MHz / 666
                          = 31.530 KHz
                        
The TIM3 CCR1 register value is equal to 333, so the TIM3 Channel 1 generates a 
PWM signal with a frequency equal to 31 KHz and a duty cycle equal to 50%:
TIM3 Channel1 duty cycle = (TIM3_CCR1/ TIM3_ARR + 1)* 100 = 50%

The TIM3 CCR2 register value is equal to 249, so the TIM3 Channel 2 generates a 
PWM signal with a frequency equal to 31 KHz and a duty cycle equal to 37.5%:
TIM3 Channel2 duty cycle = (TIM3_CCR2/ TIM3_ARR + 1)* 100 = 37.5%

The TIM3 CCR3 register value is equal to 166, so the TIM3 Channel 3 generates a 
PWM signal with a frequency equal to 31 KHz and a duty cycle equal to 25%:
TIM3 Channel3 duty cycle = (TIM3_CCR3/ TIM3_ARR + 1)* 100 = 25%

The TIM3 CCR4 register value is equal to 83, so the TIM3 Channel 4 generates a 
PWM signal with a frequency equal to 31 KHz and a duty cycle equal to 12.5%:
TIM3 Channel4 duty cycle = (TIM3_CCR4/ TIM3_ARR + 1)* 100 = 12.5%

The PWM waveforms can be displayed using an oscilloscope.

                             
@note Care must be taken when using HAL_Delay(), this function provides accurate delay (in milliseconds)
      based on variable incremented in SysTick ISR. This implies that if HAL_Delay() is called from
      a peripheral ISR process, then the SysTick interrupt must have higher priority (numerically lower)
      than the peripheral interrupt. Otherwise the caller ISR process will be blocked.
      To change the SysTick interrupt priority you have to use HAL_NVIC_SetPriority() function.
      
@note The application needs to ensure that the SysTick time base is always set to 1 millisecond
      to have correct HAL operation.

@note LED4 is turned ON, because it's sharing the same GPIO pin with the TIM3 channel 2.


@par Directory contents 

  - TIM/TIM_PWMOutput/Inc/stm32f4xx_hal_conf.h     HAL configuration file
  - TIM/TIM_PWMOutput/Inc/stm32f4xx_it.h           Interrupt handlers header file
  - TIM/TIM_PWMOutput/Inc/main.h                   Main program header file  
  - TIM/TIM_PWMOutput/Src/stm32f4xx_it.c           Interrupt handlers
  - TIM/TIM_PWMOutput/Src/main.c                   Main program
  - TIM/TIM_PWMOutput/Src/stm32f4xx_hal_msp.c      HAL MSP module
  - TIM/TIM_PWMOutput/Src/system_stm32f4xx.c       STM32F4xx system clock configuration file

                                     
@par Hardware and Software environment 

  - This example runs on STM32F407xx/417xx devices.
    
  - This example has been tested with STMicroelectronics STM324xG-EVAL RevC 
    evaluation boards and can be easily tailored to any other supported device and development board

  - STM324xG-EVAL RevC Set-up 
    - Connect the following pins to an oscilloscope to monitor the different 
      waveforms:
        - PC.06: (TIM3_CH1)
        - PC.07: (TIM3_CH2)
        - PC.08: (TIM3_CH3)
        - PC.09: (TIM3_CH4)  


@par How to use it ? 

In order to make the program work, you must do the following :
 - Open your preferred toolchain 
 - Rebuild all files and load your image into target memory
 - Run the example

 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
