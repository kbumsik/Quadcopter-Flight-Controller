/**
  @page CAN_Networking CAN Networking example
  
  @verbatim
  ******************** (C) COPYRIGHT 2015 STMicroelectronics *******************
  * @file    CAN/CAN_Networking/readme.txt 
  * @author  MCD Application Team
  * @version V1.2.1
  * @date    13-March-2015
  * @brief   Description of the CAN Networking example.
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

This example shows how to configure the CAN peripheral to send and receive 
CAN frames in normal mode. The sent frames are used to control LEDs by pressing  
key push button.
	
The CAN serial communication link is a bus to which a number of units may be
connected. This number has no theoretical limit. Practically the total number
of units will be limited by delay times and/or electrical loads on the bus line.

This program behaves as follows:
  - After reset, all LEDs are OFF on the N eval boards
  - By Pressing on KEY Button : LED1 turns ON and all other LEDs are OFF, on the N
    eval boards connected to the bus. 
  - Press on KEY Button again to send CAN Frame to command LEDn+1 ON, all other LEDs 
    are OFF on the N eval boards.
    
 @note This example is tested with a bus of 3 units. The same program example is 
       loaded in all units to send and receive frames.
 @note Any unit in the CAN bus may play the role of sender (by pressing KEY button)
       or receiver.
	
The CAN is configured as follows:
	- ID Filter  = All identifiers are allowed
	- RTR = Data
	- DLC = 1 byte
	- Data: LED number that should be turned ON


@note Care must be taken when using HAL_Delay(), this function provides accurate delay (in milliseconds)
      based on variable incremented in SysTick ISR. This implies that if HAL_Delay() is called from
      a peripheral ISR process, then the SysTick interrupt must have higher priority (numerically lower)
      than the peripheral interrupt. Otherwise the caller ISR process will be blocked.
      To change the SysTick interrupt priority you have to use HAL_NVIC_SetPriority() function.
      
@note The application needs to ensure that the SysTick time base is always set to 1 millisecond
      to have correct HAL operation.


@par Directory contents 

  - CAN/CAN_Networking/Inc/stm32f4xx_hal_conf.h    HAL configuration file
  - CAN/CAN_Networking/Inc/stm32f4xx_it.h          Interrupt handlers header file
  - CAN/CAN_Networking/Inc/main.h                  Main program header file  
  - CAN/CAN_Networking/Src/stm32f4xx_it.c          Interrupt handlers
  - CAN/CAN_Networking/Src/main.c                  Main program
  - CAN/CAN_Networking/Src/stm32f4xx_hal_msp.c     HAL MSP module
  - CAN/CAN_Networking/Src/system_stm32f4xx.c      STM32F4xx system clock configuration file
  

@par Hardware and Software environment

  - This example runs on STM32F429xx/STM32F439xx devices.
    
  - This example has been tested with STM324x9I-EVAL RevB evaluation board and can be
    easily tailored to any other supported device and development board.

  - STM324x9I-EVAL RevB Set-up 
    - Use LED1, LED2, LED3 and LED4 connected respectively to PG.06, PG.07, PG.10
      and PG.12 pins
    - Use KEY Push Button connected to PC13 
    - Connect a female/female CAN cable between at least 2 EVAL CAN connectors (CN22)
    - Please ensure that jumper JP10 is in position 1-2 
    - Please ensure that jumper JP11 is fitted.
    - Please ensure that jumper JP16 is fitted.
  
  
@par How to use it ? 

In order to make the program work, you must do the following :
 - Open your preferred toolchain 
 - Rebuild all files and load your image into target memory
 - Run the example
                       
 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
 