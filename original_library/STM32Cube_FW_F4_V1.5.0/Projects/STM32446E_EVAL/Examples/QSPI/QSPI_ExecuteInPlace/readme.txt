/**
  @page QSPI_ExecuteInPlace QSPI execute in place example
  
  @verbatim
  ******************** (C) COPYRIGHT 2015 STMicroelectronics *******************
  * @file    QSPI/QSPI_ExecuteInPlace/readme.txt 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    13-March-2015
  * @brief   Description of the code execution from QSPI memory example.
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

This example describes how to erase part of the QSPI memory, write data in DMA mode and access to QSPI memory 
in memory-mapped mode to check the data in a forever loop.

At the startup, the QSPI memory is erased, then the data are copied from the initialization
section of the flash to the QSPI memory. And finally the QSPI is configured in memory-mapped 
mode and the code in executed in a forever loop.

LED1 and LED3 toggle in a forever loop.
LED3 is on as soon as an error is returned by HAL API, and no more LED toggles.

In this example, HCLK is configured at 180 MHz.
QSPI CLKDIV is set to 1, so QSPI frequency is = 180 MHz/(1+1) = 90MHz for Write
and Read operations.

@note  In order to account for external signal delays, SampleShifting parameter should be set to
       HALFCYCLE. This bit allows the data is to be sampled correctly at 90MHZ. Otherwise
       Read operation will not work at frequency greater than 60MHz.

@note Care must be taken when using HAL_Delay(), this function provides accurate delay (in milliseconds)
      based on variable incremented in SysTick ISR. This implies that if HAL_Delay() is called from
      a peripheral ISR process, then the SysTick interrupt must have higher priority (numerically lower)
      than the peripheral interrupt. Otherwise the caller ISR process will be blocked.
      To change the SysTick interrupt priority you have to use HAL_NVIC_SetPriority() function.
      
@note The application need to ensure that the SysTick time base is always set to 1 millisecond
      to have correct HAL operation.

@par Directory contents 

  - QSPI/QSPI_ExecuteInPlace/Inc/stm32f4xx_hal_conf.h HAL configuration file
  - QSPI/QSPI_ExecuteInPlace/Inc/stm32f4xx_it.h       Interrupt handlers header file
  - QSPI/QSPI_ExecuteInPlace/Inc/main.h               Header for main.c module  
  - QSPI/QSPI_ExecuteInPlace/Src/stm32f4xx_it.c       Interrupt handlers
  - QSPI/QSPI_ExecuteInPlace/Src/main.c               Main program
  - QSPI/QSPI_ExecuteInPlaceQSPI_ReadWrite_DMA/Src/system_stm32f4xx.c   STM32F4xx system source file
  - QSPI/QSPI_ExecuteInPlace/Src/stm32f4xx_hal_msp.c  HAL MSP file    


@par Hardware and Software environment

  - This example runs on STM32F446xx devices.

  - This example has been tested with STM32446E-EVAL board and can be
    easily tailored to any other supported device and development board.

  - STM32446E-EVAL Set-up :
     Make sure that board is configured for QuadSPI memory interface as 
      described in MB1162_UM.pdf

  - A specific region for the QSPI memory has been added in the scatter file 
   (STM32F446xx_flash.icf for IAR toolchain) in order to map functions in this memory

@par How to use it ? 

In order to make the program work, you must do the following :
 - Open your preferred toolchain
 - Rebuild all files and load your image into target memory
 - Run the example

 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
