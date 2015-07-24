/**
  @page SPI_FullDuplex_AdvComPolling SPI Full Duplex Polling example
  
  @verbatim
  ******************** (C) COPYRIGHT 2015 STMicroelectronics *******************
  * @file    SPI/SPI_FullDuplex_AdvComPolling/readme.txt 
  * @author  MCD Application Team
  * @version V1.2.1
  * @date    13-March-2015
  * @brief   Description of the SPI Full Duplex Polling example.
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
 
This example guides you through the different configuration steps by mean of HAL API 
to ensure SPI Data buffer transmission and reception using Polling, in an advance 
communication mode: Master board is always sending command to slave before any
transmission and Slave board is sending acknowledge before going further.

- Hardware Description

To use this example, you need to load it on two STM32 boards (let's call them 
BoardA and BoardB) then connect these two boards through SPI lines and GND.
In the firmware example two project are provided to use the SPI peripheral as STM32
Master device or as STM32 Slave .
   _________________________                        _________________________
  |           ______________|                      |______________           |
  |          |SPI2          |                      |          SPI2|          |
  |          |              |                      |              |          |
  |          |     CLK(PB13)|______________________|(PB13)CLK     |          |
  |          |              |                      |              |          |
  |          |    MISO(PB14)|______________________|(PB14)MISO    |          |
  |          |              |                      |              |          |
  |          |    MOSI(PB15)|______________________|(PB15)MOSI    |          |
  |          |              |                      |              |          |
  |          |              |                      |              |          |
  |          |              |                      |              |          |
  |          |______________|                      |______________|          |
  |      __                 |                      |                         |
  |     |__|                |                      |                         |
  |     USER                |                      |                         |
  |                      GND|______________________|GND                      |
  |                         |                      |                         |
  |_STM32F4 Master _________|                      |_STM32F4 Slave __________|
  
@note The connection between the pins should use a short wires and a common Ground.

- Software Description  

At the beginning of the main program the HAL_Init() function is called to reset 
all the peripherals, initialize the Flash interface and the systick.
Then the SystemClock_Config() function is used to configure the system
clock (SYSCLK) to run at 168 MHz.

The SPI peripheral configuration is ensured by the HAL_SPI_Init() function.
This later is calling the HAL_SPI_MspInit()function which core is implementing
the configuration of the needed SPI resources according to the used hardware (CLOCK & 
GPIO). You may update this function to change SPI configuration.

The SPI peripheral is configured in full duplex mode with baudrate = FCPU/4. 
                       
In Master board, when the user button is pressed (used for only synchronization at startup)
these steps are executed: 
 - Synchro with the Slave
 - Requests read operation by sending specific command to the slave that contain
   the master read command and the number of data to be read
 - Synchro with the Slave
 - Receive the ACK from the Slave
 - Synchro with the Slave  
 - Receive the requested data "aRxBuffer" from the slave
 - Synchro with the Slave
 - Send the ACK to the Slave
 - Checks the correctness of data and in case of mismatch between sent and received data,
   LED5 is On and communication is stopped (using infinite loop). Otherwise LED6 is Toggle
   indicate that the data is received correctly.
 - Synchro with the Slave
 - Requests write operation by sending specific command to the slave that contain
   the master write command and the number of data to be written
 - Synchro with the Slave 
 - Receive the ACK from the Slave
 - Synchro with the Slave 
 - Send the requested data "aTxMasterBuffer" to the slave
 - Synchro with the Slave 
 - Receive the ACK from the Slave
These previously steps are repeated in an infinitely loop and a delay 100ms is inserted at
the end of data transfer to permit the user to see LED4 toggling.

In Slave board, these steps are executed: 
 - Synchro with the Master
 - Receive requests from the master
 - Synchro with the Master
 - Send the ACK to the Master
 - Synchro with the Master
 - If the "ADDRCMD_MASTER_READ" command is received: 
   - Send the requested data "aTxSlaveBuffer" to the Master
   - Synchro with the Master
   - Receive the ACK from the Master
 - If the "ADDRCMD_MASTER_WRITE" command is received: 
   - Receive the requested data "aRxBuffer" from the Master
   - Synchro with the Master
   - Send the ACK to the Master
   - checks the correctness of data is performed and in case of mismatch between sent and
     received data, LED5 is On and communication is stopped (using infinite loop).
     Otherwise LED6 Toggles indicating that the data is received correctly.

These previously steps are repeated in an infinitely loop.

STM32F4-Discovery board's LEDs can be used to monitor the transfer status:
 - LED3 (orange) toggles on Master board waiting for user button to be pushed.
 - LED4 (green) toggles on Master side at end of each loop.
 - LED6 (blue) toggles on when the read comparison is OK.
 - LED5 (red) turns ON when there is an error in transmission/reception process.  

@note SPIx instance used and associated resources can be updated in "main.h"
       file depending hardware configuration used.

@note Care must be taken when using HAL_Delay(), this function provides accurate delay (in milliseconds)
      based on variable incremented in SysTick ISR. This implies that if HAL_Delay() is called from
      a peripheral ISR process, then the SysTick interrupt must have higher priority (numerically lower)
      than the peripheral interrupt. Otherwise the caller ISR process will be blocked.
      To change the SysTick interrupt priority you have to use HAL_NVIC_SetPriority() function.
      
@note The application needs to ensure that the SysTick time base is always set to 1 millisecond
      to have correct HAL operation.


@par Directory contents 

  - SPI/SPI_FullDuplex_AdvComPolling/Inc/stm32f4xx_hal_conf.h    HAL configuration file
  - SPI/SPI_FullDuplex_AdvComPolling/Inc/stm32f4xx_it.h          SPI interrupt handlers header file
  - SPI/SPI_FullDuplex_AdvComPolling/Inc/main.h                  Main program header file  
  - SPI/SPI_FullDuplex_AdvComPolling/Src/stm32f4xx_it.c          SPI interrupt handlers
  - SPI/SPI_FullDuplex_AdvComPolling/Src/main.c                  Main program
  - SPI/SPI_FullDuplex_AdvComPolling/Src/system_stm32f4xx.c      STM32F4xx system clock configuration file
  - SPI/SPI_FullDuplex_AdvComPolling/Src/stm32f4xx_hal_msp.c     HAL MSP module
  

@par Hardware and Software environment
  
  - This example runs on STM32F407xx devices.
      
  - This example has been tested with STMicroelectronics STM32F4-Discovery RevB & RevC 
    boards and can be easily tailored to any other supported device 
    and development board.

  - STM32F4-Discovery Set-up
    - Use LED1, LED2, LED3 and LED4 connected respectively to PD.12, PD.13, PD.14
      and PD.15 pins
    - Use the USer Push Button 
    - Connect BoardA SPI2 SCK pin  (PB13) to BoardB SPI2 SCK pin  (PB13)
    - Connect BoardA SPI2 MOSI pin (PB14) to BoardB SPI2 MOSI pin (PB14)
    - Connect BoardA SPI2 MISO pin (PB15) to BoardB SPI2 MOSI pin (PB15)
    - Connect BoardA GND to BoardB GND
    
    
@par How to use it ?
 
In order to make the program work, you must do the following :
- Open your preferred toolchain 
- Rebuild all files and load your image into target memory
- Firstly, Run the Master example, then run the Slave example

 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */ 
