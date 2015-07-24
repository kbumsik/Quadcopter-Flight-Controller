/**
  @page HASH_SHA1MD5_DMA  HASH digest calculation using SHA1 and MD5 example
  with DMA transfer.
  
  @verbatim
  ******************** (C) COPYRIGHT 2015 STMicroelectronics *******************
  * @file    HASH/HASH_SHA1MD5_DMA/readme.txt
  * @author  MCD Application Team
  * @version V1.2.1
  * @date    13-March-2015
  * @brief   Description of the HASH digest calculation using SHA1 and MD5 example.
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

This example provides a short description of how to use the HASH peripheral to 
hash data using SHA-1 and MD5 Algorithms.

For this example, DMA is used to transfer data from memory to the HASH processor.
The message to hash is a 2048 bit data.
The SHA-1 message digest result is a 160 bit data, and the MD5 message digest
result is a 128 bit data.
The expected HASH digests (for SHA1 and MD5) are already computed using an online
HASH tool. Those values are compared to those computed by the HASH peripheral.
In case there is a mismatch the red LED is turned ON.
In case the SHA1 digest is computed correctly the green LED (LED1) is turned ON.
In case the MD5 digest is computed correctly the blue LED (LED4) is turned ON.

@note Care must be taken when using HAL_Delay(), this function provides accurate delay (in milliseconds)
      based on variable incremented in SysTick ISR. This implies that if HAL_Delay() is called from
      a peripheral ISR process, then the SysTick interrupt must have higher priority (numerically lower)
      than the peripheral interrupt. Otherwise the caller ISR process will be blocked.
      To change the SysTick interrupt priority you have to use HAL_NVIC_SetPriority() function.
      
@note The application needs to ensure that the SysTick time base is always set to 1 millisecond
      to have correct HAL operation.


@par Directory contents 

  - HASH/HASH_SHA1MD5_DMA/Inc/stm32f4xx_hal_conf.h    HAL configuration file
  - HASH/HASH_SHA1MD5_DMA/Inc/stm32f4xx_it.h          Interrupt handlers header file
  - HASH/HASH_SHA1MD5_DMA/Inc/main.h                  Main program header file  
  - HASH/HASH_SHA1MD5_DMA/Src/stm32f4xx_it.c          Interrupt handlers
  - HASH/HASH_SHA1MD5_DMA/Src/main.c                  Main program
  - HASH/HASH_SHA1MD5_DMA/Src/stm32f4xx_hal_msp.c     HAL MSP module
  - HASH/HASH_SHA1MD5_DMA/Src/system_stm32f4xx.c      STM32F4xx system clock configuration file


@par Hardware and Software environment

  - This example runs on STM32F43xxx devices.
    
  - This example has been tested with STM324x9I-EVAL RevB evaluation board and
    can be easily tailored to any other supported device and development board.      


@par How to use it ? 

In order to make the program work, you must do the following :
 - Open your preferred toolchain 
 - Rebuild all files and load your image into target memory
 - Run the example   
   
 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
 