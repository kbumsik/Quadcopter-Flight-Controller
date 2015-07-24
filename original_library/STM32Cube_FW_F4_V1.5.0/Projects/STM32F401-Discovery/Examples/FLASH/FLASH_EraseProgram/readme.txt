/**
  @page FLASH_EraseProgram FLASH Erase and Program

  @verbatim
  ******************** (C) COPYRIGHT 2015 STMicroelectronics *******************
  * @file    FLASH/FLASH_EraseProgram/readme.txt
  * @author  MCD Application Team
  * @version V1.2.1
  * @date    13-March-2015
  * @brief   Description of the FLASH Erase and Program example.
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

This example provides a description of how to erase and program the STM32F4xx FLASH.

After Reset, the Flash memory Program/Erase Controller is locked. To unlock it,
the FLASH_Unlock function is used.
Before programming the desired addresses, an erase operation is performed using 
the flash erase sector feature.  A programmation of the erase procedure is done 
in filling the erase init structure (erase will be done by sectors from the 1st 
sector to the calculation of sectors number) 
Then all these sectors will be erased one by one by calling HAL_FLASHEx_Erase function. 

@note: if problem occurs on a sector, erase will be stopped and faulty sector will 
be returned to user (through variable 'SectorError').

Once this operation is finished, the word programming operation will be performed by 
using the HAL_FLASH_Program function. The written data is then checked and the
result of the programming operation is stored into the MemoryProgramStatus variable.

STM32 Discovery board's LEDs can be used to monitor the transfer status:
 - LED4 (GREEN) is ON when there are no errors detected after programmation
 - LED5 (RED) is ON when there are errors dectected after programmation
 - LED6 (BLUE) is ON when there is an issue during erase procedure
 - LED3 (ORANGE) is ON when there is an issue during program procedure

@note Care must be taken when using HAL_Delay(), this function provides accurate delay (in milliseconds)
      based on variable incremented in SysTick ISR. This implies that if HAL_Delay() is called from
      a peripheral ISR process, then the SysTick interrupt must have higher priority (numerically lower)
      than the peripheral interrupt. Otherwise the caller ISR process will be blocked.
      To change the SysTick interrupt priority you have to use HAL_NVIC_SetPriority() function.

@note The application needs to ensure that the SysTick time base is always set to 1 millisecond
      to have correct HAL operation.

@par Directory contents 

  - FLASH/FLASH_EraseProgram/Src/stm32f4xx_it.c       Interrupt handlers
  - FLASH/FLASH_EraseProgram/Src/main.c               Main program
  - FLASH/FLASH_EraseProgram/Src/system_stm32f4xx.c   STM32F4xx system clock configuration file
  - FLASH/FLASH_EraseProgram/Inc/stm32f4xx_hal_conf.h HAL Configuration file
  - FLASH/FLASH_EraseProgram/Inc/stm32f4xx_it.h       Interrupt handlers header file
  - FLASH/FLASH_EraseProgram/Inc/main.h               Main program header file

@par Hardware and Software environment 

  - This example runs on STM32F401xCx Devices.
    
  - This example has been tested with STM32F401-Discovery RevB board (MB1115B) and can be
    easily tailored to any other supported device and development board.
  
@par How to use it ? 

In order to make the program work, you must do the following :
 - Open your preferred toolchain 
 - Rebuild all files and load your image into target memory
 - Run the example

 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
