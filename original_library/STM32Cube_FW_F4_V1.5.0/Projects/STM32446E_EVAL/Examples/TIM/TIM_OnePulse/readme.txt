/**
  @page TIM_OnePulse TIM One Pulse example
  
  @verbatim
  ******************** (C) COPYRIGHT 2015 STMicroelectronics *******************
  * @file    TIM/TIM_OnePulse/readme.txt 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    13-March-2015
  * @brief   Description of the TIM One Pulse example.      
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

This example shows how to use the TIM peripheral to generate a One pulse Mode 
after a Rising edge of an external signal is received in Timer Input pin.


Clock setup for TIM5
================================

  TIM5CLK = SystemCoreClock = 180 MHz.
  
  Prescaler = (TIM5CLK /TIM5 counter clock) - 1
  
  The prescaler value is computed in order to have TIM5 counter clock 
  set at 18000000 Hz.
  
  The Autoreload value is 65535 (TIM5->ARR), so the maximum frequency value to 
  trigger the TIM5 input is 18000000/65535 [Hz].
 
Configuration of TIM5 in One Pulse Mode
===================================================
 
  - The external signal is connected to TIM5_CH2 pin (PA.01), 
    and a rising edge on this input is used to trigger the Timer.
  - The One Pulse signal is output on TIM5_CH1 (PA.00).

  The delay value is fixed to:
   - Delay =  CCR1/TIM5 counter clock 
           = 16383 / 18000000 [sec]
           
  The pulse value is fixed to : 
   - Pulse value = (TIM_Period - TIM_Pulse)/TIM5 counter clock  
                 = (65535 - 16383) / 18000000 [sec]

  The one pulse waveform can be displayed using an oscilloscope and it looks
  like this.
  
                               ____
                               |   |
  CH2 _________________________|   |__________________________________________
 
                                             ___________________________
                                            |                           |
  CH1 ______________________________________|                           |_____
                               <---Delay----><------Pulse--------------->
  


@par Directory contents 

  - TIM/TIM_OnePulse/Inc/stm32f4xx_hal_conf.h    HAL configuration file
  - TIM/TIM_OnePulse/Inc/stm32f4xx_it.h          Interrupt handlers header file
  - TIM/TIM_OnePulse/Inc/main.h                  Header for main.c module  
  - TIM/TIM_OnePulse/Src/stm32f4xx_it.c          Interrupt handlers
  - TIM/TIM_OnePulse/Src/main.c                  Main program
  - TIM/TIM_OnePulse/Src/stm32f4xx_hal_msp.c     HAL MSP file
  - TIM/TIM_OnePulse/Src/system_stm32f4xx.c      STM32F4xx system source file


@par Hardware and Software environment

   - This example runs on STM32F446xx devices.
   - In this example, the clock is set to 180 MHz.
    
  - This example has been tested with STM32446E-EVAL board and can be
    easily tailored to any other supported device and development board.

  - STM32446E-EVAL Set-up
   - Connect the external signal to the TIM5_CH2 pin (PA.01) ()
   - Connect the TIM5_CH1 pin(PA.00) () to an oscilloscope to monitor the waveform.  

@par How to use it ? 

In order to make the program work, you must do the following :
 - Open your preferred toolchain
 - Rebuild all files: Project->Rebuild all
 - Load project image: Project->Download and Debug
 - Run program: Debug->Go(F5) 

 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
