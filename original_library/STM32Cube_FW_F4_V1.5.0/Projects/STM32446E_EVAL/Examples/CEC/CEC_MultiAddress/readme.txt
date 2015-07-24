/**
  @page CEC CEC_MultiAddress example
  
  @verbatim
  ******************** (C) COPYRIGHT 2015 STMicroelectronics *******************
  * @file    CEC/CEC_MultiAddress/readme.txt 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    13-March-2015
  * @brief   Description of the CEC Multiple Address communication example.
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

This example shows how to configure and use the CEC peripheral to receive and 
transmit messages in the case where one device supports two distinct logical
addresses at the same time.


- Hardware Description

To use this example, two STM32446E-EVAL boards (called Device_1 and 
Device_2) are loaded with the same software then connected through CEC lines
(PB3 which is internally connected to PB6) and GND.
 /|\  In the firmware file main.h, uncomment the dedicated line to use
/_!_\ the CEC peripheral as STM32 device_1 or STM32 device_2.


@verbatim
*------------------------------------------------------------------------------*
|           STM32446E-EVAL                               STM32446E-EVAL        |
|         Device Address :0x01                    Device Address :0x03         |
|                                                 Device Address :0x05)        |
|         ____________________                   ____________________          |
|        |                    |                 |                    |         |
|        |                    |                 |                    |         | 
|        |     __________     |                 |     __________     |         |
|        |    |          |PB3 |                 |PB3 |          |    |         |
|        |    |   CEC    |____|____CECLine______|____|   CEC    |    |         |
|        |    | Device_1 |    |                 |    | Device_2 |    |         |
|        |    |__________|    |                 |    |__________|    |         |
|        |                    |                 |                    |         |
|        |  O LD1             |                 |  O LD1             |         |
|        |  O LD2    Joystick |                 |  O LD2    Joystick |         |
|        |  O LD3        _    |                 |  O LD3        _    |         |
|        |  O LD4       |_|   |                 |  O LD4       |_|   |         |
|        |                    |                 |                    |         |
|        |             GND O--|-----------------|--O GND             |         |
|        |____________________|                 |____________________|         |
|                                                                              |
|                                                                              |
*------------------------------------------------------------------------------*
@endverbatim


- Software Description

The test unrolls as follows.

On Device_1 TX side, three possible messages can be transmitted and are 
indicated as below on the transmitting board:
 - when User push-button is pressed, a ping message (no payload) is addressed
  to Device_2 first logical address; LED1 toggles
 - when Joystick DOWN push-button is pressed, a ping message (no payload) is addressed
  to Device_2 second logical address; LED4 toggles  
 - when Joystick Selection push-button is pressed, a broadcast message (with empty
 payload) is sent by Device_1; LED2 toggles

When Joystick UP push-button is pressed, LED3 toggles (no message sent)  


Accordingly, the following happens on Device_2 RX side in case of successful
reception:
 - when User push-button is pressed on TX side, 
      * LED1 is turned on
     *  LED3 and LED4 are turned off 
 - when Joystick DOWN push-button is pressed on TX side, 
      * LED4 is turned on
      * LED1 and LED3 are turned off      
 - when Joystick Selection push-button is pressed on TX side, all LEDs are turned off 


Conversely, on Device_2 TX side, only ping messages addressed to the 
single Device_1 logical address or broadcast messages are sent:
 - when User push-button is pressed, a ping message (no payload) is addressed
  to Device_2; LED1 toggles
 - when Joystick DOWN push-button is pressed, a ping message (no payload) is addressed
  to Device_2; LED4 toggles  
 - when Joystick Selection push-button is pressed, a broadcast message (with empty
 payload) is sent; LED2 toggles


Accordingly, the following happens on Device_1 RX side in case of successful
transmission:
 - when Tamper or Joystick DOWN push-button are pressed on TX side, 
      * LED1 and LED4 are turned on
      * LED 3 is turned off     
 - when Joystick Selection push-button is pressed on TX side, all LEDs are turned off 


For Device_1 and _2, in case of transmission or reception error, 
LED3 is turned on.


Practically, 2 EXTI lines (EXTI15_10 and EXTI0) are configured to 
generate an interrupt on each falling or rising edge. 
A specific message is then transmitted by the CEC IP
and a LED connected to a specific MFX GPIO pin is toggled.
    - EXTI0 is mapped to MFX used to manage Joystick pins
    - EXTI15_10 is mapped to PC.13

Then, on TX side,
  - when rising edge is detected on EXTI0-SEL joystick button, LED2 toggles
  - when falling edge is detected on EXTI15_10 due to line PC.13, LED1 toggles
  - when falling edge is detected on EXTI0-UP joystick button LED3 toggles
  - when falling edge is detected on EXTI0-DOWN joystick button LED4 toggles

In this example, HCLK is configured at 180 MHz.

@par Directory contents 

  - CEC/CEC_MultiAddress/Inc/stm32f4xx_hal_conf.h    HAL configuration file
  - CEC/CEC_MultiAddress/Inc/stm32f4xx_it.h          Interrupt handlers header file
  - CEC/CEC_MultiAddress/Inc/main.h                  Header for main.c module  
  - CEC/CEC_MultiAddress/Src/stm32f4xx_it.c          Interrupt handlers
  - CEC/CEC_MultiAddress/Src/system_stm32f4xx.c      STM32F4xx system source file
  - CEC/CEC_MultiAddress/Src/main.c                  Main program
  - CEC/CEC_MultiAddress/Src/stm32f4xx_hal_msp.c     IP hardware resources initialization
  
@par Hardware and Software environment

  - This example runs on STM32446E Devices.
    
  - This example has been tested with STM32446E-EVAL board and can be
    easily tailored to any other supported device and development board.      


@par How to use it ? 

In order to make the program work, you must do the following :
 - Open your preferred toolchain 
 - in main.h, uncomment DEVICE_1 for first board, uncomment DEVICE_2 for second board
 - Rebuild all files and load your image into target memory
 - Be aware that PB6 pin is missing but PB3 is connected directly to it (SB23 is closed).
 - With a wire, connect PB3-PB3 between the 2 boards
 - Add a pull-up resistor of 27kohm between PB3 and V3.3
 - Connect the ground of the 2 boards
 - Run the example

 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
