/**
  @page TIM_OCToggle TIM_OCToggle example
  
  @verbatim
  ******************** (C) COPYRIGHT 2015 STMicroelectronics *******************
  * @file    TIM/TIM_OCToggle/readme.txt 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    13-March-2015
  * @brief   This example shows how to configure the Timer to generate four different 
  *          signals with four different frequencies.
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

  This example shows how to configure the TIM peripheral to generate four different 
  signals with four different frequencies.

  The TIM1 frequency is set to SystemCoreClock, and the objective is
  to get TIM1 counter clock at 20 MHz so the Prescaler is computed as following:
     - Prescaler = (TIM1CLK /TIM1 counter clock) - 1
   
  SystemCoreClock is set to 180 MHz for STM32F4xx Devices.

  The TIM1 CCR1 register value is equal to 40961:
  CC1 update rate = TIM1 counter clock / CCR1_Val = 488.269 Hz,
  so the TIM1 Channel 1 generates a periodic signal with a frequency equal to 244.13 Hz.

  The TIM1 CCR2 register value is equal to 20480:
  CC2 update rate = TIM1 counter clock / CCR2_Val = 976.56 Hz,
  so the TIM1 channel 2 generates a periodic signal with a frequency equal to 488.28 Hz.

  The TIM1 CCR3 register value is equal to 10240:
  CC3 update rate = TIM1 counter clock / CCR3_Val = 1953.1 Hz,
  so the TIM1 channel 3 generates a periodic signal with a frequency equal to 976.56 Hz.

  The TIM1 CCR4 register value is equal to 5120:
  CC4 update rate = TIM1 counter clock / CCR4_Val =  3906.25 Hz,
  so the TIM1 channel 4 generates a periodic signal with a frequency equal to 1953.12 Hz.

@note Care must be taken when using HAL_Delay(), this function provides accurate delay (in milliseconds)
      based on variable incremented in SysTick ISR. This implies that if HAL_Delay() is called from
      a peripheral ISR process, then the SysTick interrupt must have higher priority (numerically lower)
      than the peripheral interrupt. Otherwise the caller ISR process will be blocked.
      To change the SysTick interrupt priority you have to use HAL_NVIC_SetPriority() function.
      
@note The application need to ensure that the SysTick time base is always set to 1 millisecond
      to have correct HAL operation.

@par Directory contents 

  - TIM/TIM_OCToggle/Inc/stm32f4xx_hal_conf.h    HAL configuration file
  - TIM/TIM_OCToggle/Inc/stm32f4xx_it.h          Interrupt handlers header file
  - TIM/TIM_OCToggle/Inc/main.h                  Header for main.c module  
  - TIM/TIM_OCToggle/Src/stm32f4xx_it.c          Interrupt handlers
  - TIM/TIM_OCToggle/Src/main.c                  Main program
  - TIM/TIM_OCToggle/Src/stm32f4xx_hal_msp.c     HAL MSP file
  - TIM/TIM_OCToggle/Src/system_stm32f4xx.c      STM32F4xx system source file


@par Hardware and Software environment

  - This example runs on STM32F446xx devices.
    
  - This example has been tested with STMicroelectronics STM32446E-EVAL 
    board and can be easily tailored to any other supported device 
    and development board.      

  - STM32446E-EVAL Set-up
   Connect the following pins to an oscilloscope to monitor the different waveforms:
      - PA.08: (TIM1_CH1) ((pin 1 in CN6 connector))
      - PA.09: (TIM1_CH2) ((pin 11 in CN10 connector))
      - PA.10: (TIM1_CH3) ((pin 13 in CN10 connector))
      - PA.11: (TIM1_CH4) ((pin 17 in CN10 connector))

@par How to use it ? 

In order to make the program work, you must do the following :
 - Open your preferred toolchain
 - Rebuild all files and load your image into target memory
 - Run the example

 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
