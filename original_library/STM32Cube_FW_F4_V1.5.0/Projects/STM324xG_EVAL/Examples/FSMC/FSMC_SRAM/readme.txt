/**
  @page FSMC_SRAM FSMC SRAM memory basic functionalities use example
  
  @verbatim
  ******************* (C) COPYRIGHT 2015 STMicroelectronics ********************
  * @file    FSMC/FSMC_SRAM/readme.txt 
  * @author  MCD Application Team
  * @version V1.2.1
  * @date    13-March-2015
  * @brief   Description of the FSMC SRAM memory basic functionalities use example.
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
to configure the FMC controller to access the IS61WV102416BLL-10MLI SRAM memory mounted 
on STM324xG-EVAL evaluation board.
 
At the beginning of the main program the HAL_Init() function is called to reset 
all the peripherals, initialize the Flash interface and the systick.
Then the SystemClock_Config() function is used to configure the system
clock (SYSCLK) to run at 168 MHz.
  
The goal of this example is to explain the different steps to configure the FMC 
and make the SRAM device ready for access, without using the MSP layer.          

In this example, the SRAM device is configured and initialized explicitly 
following all initialization sequence steps. After initializing the device, user 
can perform read/write operations on it. A data buffer is written to the SRAM 
memory, then read back and checked to verify its correctness.

The user can chose his own configuration by commenting/uncommenting the defines for  
undesired/desired configurations , for example, to enable Continuous clock feature, 
uncomment the define for the configuration "FMC_CONTINUOUS_CLOCK_SYNC_ASYNC" in "main.h".
  
If the data is read correctly from SRAM, the LED1 is ON, otherwise the LED2 is ON.   
LED3 is ON when there is an error in initialization.

@note Care must be taken when using HAL_Delay(), this function provides accurate delay (in milliseconds)
      based on variable incremented in SysTick ISR. This implies that if HAL_Delay() is called from
      a peripheral ISR process, then the SysTick interrupt must have higher priority (numerically lower)
      than the peripheral interrupt. Otherwise the caller ISR process will be blocked.
      To change the SysTick interrupt priority you have to use HAL_NVIC_SetPriority() function.
      
@note The application needs to ensure that the SysTick time base is always set to 1 millisecond
      to have correct HAL operation.


@par Directory contents
                        
 - FSMC/FSMC_SRAM/Inc/stm32f4xx_hal_conf.h        HAL Configuration file
 - FSMC/FSMC_SRAM/Inc/main.h                      Main program header file 
 - FSMC/FSMC_SRAM/Inc/stm32f4xx_it.h              Interrupt handlers header file
 - FSMC/FSMC_SRAM/Src/main.c                      Main program  
 - FSMC/FSMC_SRAM/Src/stm32f4xx_hal_msp.c         HAL MSP module  
 - FSMC/FSMC_SRAM/Src/stm32f4xx_it.c              Interrupt handlers
 - FSMC/FSMC_SRAM/Src/system_stm32f4xx.c          STM32F4xx system clock configuration file                                    
 
 
@par Hardware and Software environment 

  - This example runs on STM32F407xx/417xx devices.  
    
  - This example has been tested with STMicroelectronics STM324xG-EVAL RevC 
    evaluation boards and can be easily tailored to any other supported device 
    and development board.
    
  - STM324xG-EVAL RevC Set-up  
    - Please ensure that the jumper JP10 is not fitted in the position 2-3.  


@par How to use it ? 

In order to make the program work, you must do the following :
 - Open your preferred toolchain
 - Rebuild all files and load your image into target memory
 - Run the example

 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
