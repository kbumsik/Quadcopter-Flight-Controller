/**
  @page FMC_SDRAM_DataMemory FMC SDRAM data memory example
  
  @verbatim
  ******************* (C) COPYRIGHT 2015 STMicroelectronics ********************
  * @file    FMC/FMC_SDRAM_DataMemory/readme.txt 
  * @author  MCD Application Team
  * @version V1.2.1
  * @date    13-March-2015
  * @brief   Description of the FMC SDRAM_DataMemory example.
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
to use the MT48LC4M32B2B5-7 SDRAM mounted on STM324x9I-EVAL as data memory 
(including heap and stack).
  
Then the SystemClock_Config() function is used to configure the system clock (SYSCLK) 
to run at 180 MHz. 

The example scenario does not reflect a real application case, its purpose is to
provide only the procedure to follow to use the external SDRAM as data memory.

This example does not use the default library startup file. It uses a modified 
startup file provided with the example. The user has to add the new startup 
file in the project source list. While startup, the SDRAM memory is configured 
and initialized to be ready to contain data.
  
The user has to configure his preferred toolchain using the provided linker file.
The RAM zone is modified in order to use the external SDRAM memory as a RAM.

At this stage, all the used data can be located in the external SDRAM memory.

The user can use the debugger's watch to evaluate "uwTabAddr" and "MSPValue" variables
values which should be equal to "0xC00xxxxx".

@note Care must be taken when using HAL_Delay(), this function provides accurate delay (in milliseconds)
      based on variable incremented in SysTick ISR. This implies that if HAL_Delay() is called from
      a peripheral ISR process, then the SysTick interrupt must have higher priority (numerically lower)
      than the peripheral interrupt. Otherwise the caller ISR process will be blocked.
      To change the SysTick interrupt priority you have to use HAL_NVIC_SetPriority() function.
      
@note The application needs to ensure that the SysTick time base is always set to 1 millisecond
      to have correct HAL operation.


@par Directory contents
 
 - FMC/FMC_SDRAM_DataMemory/Inc/stm32f4xx_hal_conf.h        Library Configuration file
 - FMC/FMC_SDRAM_DataMemory/Inc/stm32f4xx_it.h              Interrupt handlers header file
 - FMC/FMC_SDRAM_DataMemory/Inc//main.h                     Main program header file 
 - FMC/FMC_SDRAM_DataMemory/Src/stm32f4xx_it.c              Interrupt handlers
 - FMC/FMC_SDRAM_DataMemory/Src/main.c                      Main program 
 - FMC/FMC_SDRAM_DataMemory/Src/system_stm32f4xx.c          STM32F4xx system clock configuration file                                       


@par Hardware and Software environment 

  - This example runs on STM32F429xx/STM32F439xx devices.
  
  - This example has been tested with STMicroelectronics STM324x9I-EVAL RevB 
    evaluation boards and can be easily tailored to any other supported device 
    and development board.


@par How to use it ? 

In order to make the program work, you must do the following :
 - Open your preferred toolchain 
 - Rebuild all files and load your image into target memory
 - Run the example

 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
