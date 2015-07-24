/**
  @page TIM_OCToggle TIM Output Compare Toggle example

  @verbatim
  ******************** (C) COPYRIGHT 2015 STMicroelectronics *******************
  * @file    TIM/TIM_OCToggle/readme.txt 
  * @author  MCD Application Team
  * @version V1.2.1
  * @date    13-March-2015
  * @brief   This example shows how to configure the Timer to generate four 
  *          delayed signals.
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

This example shows how to configure the TIM3 peripheral to generate four different 
signals with four different frequencies.

The TIM3CLK frequency is set to SystemCoreClock / 2 (Hz), and we want to get TIM3
counter clock at 21 MHz so the Prescaler is computed as following:
- Prescaler = (TIM3CLK / TIM3 counter clock) - 1
SystemCoreClock is set to 180 MHz for STM32F4xx devices

The TIM3 CCR1 register value is equal to 40961: 
CC1 update rate = TIM3 counter clock / CCR1_Val = 439.44 Hz,
so the TIM3 Channel 1 generates a periodic signal with a frequency equal to 219.72 Hz.

The TIM3 CCR2 register is equal to 20480:
CC2 update rate = TIM3 counter clock / CCR2_Val = 878.9 Hz
so the TIM3 channel 2 generates a periodic signal with a frequency equal to 439.45 Hz.

The TIM3 CCR3 register is equal to 10240:
CC3 update rate = TIM3 counter clock / CCR3_Val = 1757.81 Hz
so the TIM3 channel 3 generates a periodic signal with a frequency equal to 878.9 Hz.

The TIM3 CCR4 register is equal to 5120:
CC4 update rate = TIM3 counter clock / CCR4_Val =  3515.62 Hz
so the TIM3 channel 4 generates a periodic signal with a frequency equal to 1757.81 Hz.


@note Care must be taken when using HAL_Delay(), this function provides accurate delay (in milliseconds)
      based on variable incremented in SysTick ISR. This implies that if HAL_Delay() is called from
      a peripheral ISR process, then the SysTick interrupt must have higher priority (numerically lower)
      than the peripheral interrupt. Otherwise the caller ISR process will be blocked.
      To change the SysTick interrupt priority you have to use HAL_NVIC_SetPriority() function.
      
@note The application needs to ensure that the SysTick time base is always set to 1 millisecond
      to have correct HAL operation.


@par Directory contents 

  - TIM/TIM_OCToggle/Inc/stm32f4xx_hal_conf.h     HAL configuration file
  - TIM/TIM_OCToggle/Inc/stm32f4xx_it.h           Interrupt handlers header file
  - TIM/TIM_OCToggle/Inc/main.h                   Main program header file  
  - TIM/TIM_OCToggle/Src/stm32f4xx_it.c           Interrupt handlers
  - TIM/TIM_OCToggle/Src/main.c                   Main program
  - TIM/TIM_OCToggle/Src/stm32f4xx_hal_msp.c     HAL MSP module
  - TIM/TIM_OCToggle/Src/system_stm32f4xx.c      STM32F4xx system clock configuration file

      
@par Hardware and Software environment 

  - This example runs on STM32F429xx/STM32F439xx devices.
    
  - This example has been tested with STMicroelectronics STM324x9I-EVAL RevB 
    evaluation boards and can be easily tailored to any other supported device and development board

  - STM324x9I-EVAL RevB Set-up 
    - Connect the TIM3 pins to an oscilloscope to monitor the different waveforms:
       - PC.06 (TIM3_CH1)
       - PC.07 (TIM3_CH2)
       - PC.08 (TIM3_CH3)
       - PC.09 (TIM3_CH4) 


@par How to use it ? 

In order to make the program work, you must do the following :
 - Open your preferred toolchain 
 - Rebuild all files and load your image into target memory
 - Run the example
 
 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
