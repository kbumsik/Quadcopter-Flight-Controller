/**
  @page TIM_CascadeSynchro TIM Cascade Synchro example

  @verbatim
  ******************** (C) COPYRIGHT 2015 STMicroelectronics *******************
  * @file    TIM/TIM_CascadeSynchro/readme.txt 
  * @author  MCD Application Team
  * @version V1.2.1
  * @date    13-March-2015
  * @brief   This example shows how to command 2 Timers as slaves (TIM3 & TIM4) 
  *          using a Timer as master (TIM2) example
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

This example shows how to synchronize TIM peripherals in cascade mode.
In this example three timers are used:

Timers synchronization in cascade mode:
                         

1/ TIM2 is configured as Master Timer:
   - PWM Mode is used
   - The TIM2 Update event is used as Trigger Output 
 
2)TIM3 is slave for TIM2 and Master for TIM4,
  - PWM Mode is used
  - The ITR1(TIM2) is used as input trigger 
  - Gated mode is used, so start and stop of slave counter
    are controlled by the Master trigger output signal(TIM2 update event).
  - The TIM3 Update event is used as Trigger Output.

3)TIM4 is slave for TIM3,
  - PWM Mode is used
  - The ITR2(TIM3) is used as input trigger
  - Gated mode is used, so start and stop of slave counter  are controlled by the
    Master trigger output signal(TIM3 update event).

The TIM2 counter clock is 84 MHz.

  The Master Timer TIM2 is running at:
  TIM2 frequency = TIM2 counter clock / (TIM2_Period + 1) = 328.125 KHz
  and the duty cycle is equal to: TIM2_CCR1/(TIM2_ARR + 1) = 25%

  The TIM3 is running at: 
  (TIM2 frequency)/ ((TIM3 period +1)* (TIM2_Repetition_Counter+1)) = 82.02 KHz and
  a duty cycle equal to TIM3_CCR1/(TIM3_ARR + 1) = 25%

  The TIM4 is running at:
  (TIM2 frequency)/ ((TIM4 period +1)* (Repetition_Counter+1)) = 20.5 KHz and
  a duty cycle equal to TIM4_CCR1/(TIM4_ARR + 1) = 25%
                                                       
The PWM waveform can be displayed using an oscilloscope.


@note Care must be taken when using HAL_Delay(), this function provides accurate delay (in milliseconds)
      based on variable incremented in SysTick ISR. This implies that if HAL_Delay() is called from
      a peripheral ISR process, then the SysTick interrupt must have higher priority (numerically lower)
      than the peripheral interrupt. Otherwise the caller ISR process will be blocked.
      To change the SysTick interrupt priority you have to use HAL_NVIC_SetPriority() function.
      
@note The application needs to ensure that the SysTick time base is always set to 1 millisecond
      to have correct HAL operation.


@par Directory contents 

  - TIM/TIM_CascadeSynchro/Inc/stm32f4xx_hal_conf.h      HAL configuration file
  - TIM/TIM_CascadeSynchro/Inc/stm32f4xx_it.h            Interrupt handlers header file
  - TIM/TIM_CascadeSynchro/Inc/main.h                    Main program header file
  - TIM/TIM_CascadeSynchro/Src/stm32f4xx_it.c            Interrupt handlers
  - TIM/TIM_CascadeSynchro/Src/main.c                    Main program
  - TIM/TIM_CascadeSynchro/Src/stm32f4xx_hal_msp.c       HAL MSP module
  - TIM/TIM_CascadeSynchro/Src/system_stm32f4xx.c        STM32F4xx system clock configuration file 
   

@par Hardware and Software environment

  - This example runs on STM32F407xx/417xx  devices.
    
  - This example has been tested with STMicroelectronics STM324xG-EVAL RevC 
    evaluation boards and can be easily tailored to any other supported device and development board

  - STM324xG-EVAL RevC Set-up 
   Connect the following pins to an oscilloscope to monitor the different waveforms:
      - TIM2 CH1 (PA.00)
      - TIM3 CH1 (PB.04)
      - TIM4 CH1 (PB.06)


@par How to use it ? 

In order to make the program work, you must do the following :
 - Open your preferred toolchain 
 - Rebuild all files and load your image into target memory
 - Run the example

 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
