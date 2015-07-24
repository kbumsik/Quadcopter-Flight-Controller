/**
  @page SPI_FullDuplex_ComPolling SPI Full Duplex Polling example

  @verbatim
  ******************** (C) COPYRIGHT 2015 STMicroelectronics *******************
  * @file    SPI/SPI_FullDuplex_ComPolling/readme.txt 
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
The communication is done with 2 boards through SPI.
   _________________________                        _________________________
  |           ______________|                      |______________           |
  |          |SPI4          |                      |          SPI4|          |
  |          |              |                      |              |          |
  |          |     CLK(PE02)|______________________|(PE02)CLK     |          |
  |          |              |                      |              |          |
  |          |    MISO(PE05)|______________________|(PE05)MISO    |          |
  |          |              |                      |              |          |
  |          |    MOSI(PE06)|______________________|(PE06)MOSI    |          |
  |          |              |                      |              |          |
  |          |______________|                      |______________|          |
  |      __                 |                      |      __                 |
  |     |__|                |                      |     |__|                |
  |     USER                |                      |     USER                |
  |                      GND|______________________|GND                      |
  |                         |                      |                         |
  |_STM32F429i______________|                      |_STM32F429i______________|

This example guides you through the different configuration steps by mean of HAL API 
to ensure SPI Data buffer transmission and reception using Polling.

HAL architecture allows user to easily change code to move to IT or DMA mode. 
To see other communication modes please check following examples:
SPI/SPI_FullDuplex_ComDMA
SPI/SPI_FullDuplex_ComIT

At the beginning of the main program the HAL_Init() function is called to reset 
all the peripherals, initialize the Flash interface and the systick.
Then the SystemClock_Config() function is used to configure the system
clock (SYSCLK) to run at 180 MHz.

The SPI peripheral configuration is ensured by the HAL_SPI_Init() function.
This later is calling the HAL_SPI_MspInit()function which core is implementing
the configuration of the needed SPI resources according to the used hardware (CLOCK & 
GPIO). You may update this function to change SPI configuration.

The SPI communication is then initiated.
The HAL_SPI_TransmitReceive() function allows the reception and the 
transmission of a predefined data buffer at the same time (Full Duplex Mode) 
The user can choose between Master and Slave through "#define MASTER_BOARD"
in the "main.c" file.
If the Master board is used, the "#define MASTER_BOARD" must be uncommented.
If the Slave board is used the "#define MASTER_BOARD" must be commented.

For this example the aTxBuffer is predefined and the aRxBuffer size is same as aTxBuffer.

In a first step after the user press the User Key, SPI Master starts the 
communication by sending aTxBuffer and receiving aRxBuffer through 
HAL_SPI_TransmitReceive(), at the same time SPI Slave transmits aTxBuffer 
and receives aRxBuffer through HAL_SPI_TransmitReceive(). 
The end of this step is monitored through the HAL_SPI_GetState() function
result.
Finally, aRxBuffer and aTxBuffer are compared through Buffercmp() in order to 
check buffers correctness.  

STM32 Discovery board's LEDs can be used to monitor the transfer status:
 - LED3 toggles on master board waiting user button to be pressed. Once done LED3 turns off.
 - LED4 turns ON when there is an error in transmission/reception process
 - LED3 turns ON when the transmission/reception process is complete.
 - LED4 toggle when there is a timeout error in transmission/reception process.  

 @note SPIx instance used and associated resources can be updated in "main.h"
       file depending hardware configuration used.

 @note Timeout is set to 5 Seconds which means that if no communication occurs 
       during 5 Seconds, a Timeout Error will be generated.

 @note You need to perform a reset on Slave board, then perform it on Master board
       to have the correct behaviour of this example.


@note Care must be taken when using HAL_Delay(), this function provides accurate delay (in milliseconds)
      based on variable incremented in SysTick ISR. This implies that if HAL_Delay() is called from
      a peripheral ISR process, then the SysTick interrupt must have higher priority (numerically lower)
      than the peripheral interrupt. Otherwise the caller ISR process will be blocked.
      To change the SysTick interrupt priority you have to use HAL_NVIC_SetPriority() function.
      
@note The application needs to ensure that the SysTick time base is always set to 1 millisecond
      to have correct HAL operation.


@par Directory contents 

  - SPI/SPI_FullDuplex_ComPolling/Inc/stm32f4xx_hal_conf.h    HAL configuration file
  - SPI/SPI_FullDuplex_ComPolling/Inc/stm32f4xx_it.h          SPI interrupt handlers header file
  - SPI/SPI_FullDuplex_ComPolling/Inc/main.h                  Main program header file  
  - SPI/SPI_FullDuplex_ComPolling/Src/stm32f4xx_it.c          SPI interrupt handlers
  - SPI/SPI_FullDuplex_ComPolling/Src/main.c                  Main program
  - SPI/SPI_FullDuplex_ComPolling/Src/system_stm32f4xx.c      STM32F4xx system clock configuration file
  - SPI/SPI_FullDuplex_ComPolling/Src/stm32f4xx_hal_msp.c     HAL MSP module
  

@par Hardware and Software environment 

  - This example runs on STM32F429xx devices.
    
  - This example has been tested with STMicroelectronics STM32F429I-Discovery RevB
    boards and can be easily tailored to any other supported device 
    and development board.

  - STM32F429I-Discovery RevB Set-up
    - Connect Master board PE02 to Slave Board PE02
    - Connect Master board PE05 to Slave Board PE05
    - Connect Master board PE06 to Slave Board PE06
    - Connect Master board GND  to Slave Board GND


@par How to use it ? 

In order to make the program work, you must do the following :
 - Open your preferred toolchain 
 - Rebuild all files and load your image into target memory
    o Uncomment "#define MASTER_BOARD" and load the project in Master Board
    o Comment "#define MASTER_BOARD" and load the project in Slave Board
 - Run the example

 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
