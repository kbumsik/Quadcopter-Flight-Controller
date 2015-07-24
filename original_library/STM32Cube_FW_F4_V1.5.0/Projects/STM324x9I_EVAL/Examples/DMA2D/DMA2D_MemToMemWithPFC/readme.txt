/**
  @page DMA2D_MemToMemWithPFC DMA2D Memory to Memory with PFC example
  
  @verbatim
  ******************** (C) COPYRIGHT 2015 STMicroelectronics *******************
  * @file    DMA2D/DMA2D_MemToMemWithPFC/readme.txt 
  * @author  MCD Application Team
  * @version V1.2.1
  * @date    13-March-2015
  * @brief   Description of the DMA2D Memory to Memory with PFC example.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
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

  This example provides a description of how to configure DMA2D peripheral in 
  Memory_to_Memory with pixel format conversion transfer mode.

  At the beginning of the main program the HAL_Init() function is called to reset 
  all the peripherals, initialize the Flash interface and the systick.
  Then the SystemClock_Config() function is used to configure the system
  clock (SYSCLK) to run at 180 MHz.
 
  In this basic example the goal is to explain the different fields of the DMA2D 
  structure in the case of Memory_to_Memory with pixel format conversion transfer mode
  and the difference between pixel coded on 32bits and coded on 16bits.
 
  An image is transferred from flash memory to internal RAM and during the transfer,
  a pixel format conversion is applied from RGB565 to ARGB8888. 
  The original image and the transferred image are displayed on the LCD to see 
  the difference between an image coded on 16 bits and an image coded on 32 bits.
 
 In this example two LTDC layers are used to display the original and the converted
 images as following :
  - Layer 1 is configured to display the original image with RGB565 as 
    pixel format and 320x120 size.
  - Layer 2 is configured to display the converted image with ARGB8888 as 
    pixel format and 320x120 size.

 @note : 
 The C file of the image used in this example are generated with 
 STemWin bitmap converter released with this package.
 \Middlewares\ST\STemWin\Software\BmpCvtST.exe
 Use the bitmap file under resources repository
 
  @note :
  how to calculate the size of the transferred data ? 
  => selected color mode gives the number of bits per pixel and we have 
    the number of pixel per line and the number of line, therefore :
    
    data_size = (bits per pixel) X (pixel per line) X (number of line)
    
 How to convert pixel format from ARGB8888 to ARGB4444 ?
 => only the four MSB are taken into account 
 eg : 0x AB  CD  12  34 --> 0x A   C   1   3    
        |_| |_| |_| |_|       |_| |_| |_| |_|
         A   R   G   B         A   R   G   B
         
 In general, 
 => if the number of bits per pixel in source data is more then the number of 
    bits per pixel in destination data, only the MSB are taken into account
    
 => else, if the number of bits per pixel in source data is less then the number 
    of bits per pixel in destination data, a bit replication of MSB in LSB is applied
    
    eg : from ARGB1555 to ARGB8888       PFC
    0xAB35 --> 0b1 01010 11001 10101   -------> 0b11111111 01010010 11001110 10101101     
                 A   R     G     B          --> 0xFF       52       CE       AD

STM32 Eval board's LEDs can be used to monitor the transfer status:
 - LED1 is ON when the transfer is complete.
 - LED2 is ON when there is a transfer error.
 - LED3 is ON when there is an error in transfer/Init process.
         
@note Care must be taken when using HAL_Delay(), this function provides accurate delay (in milliseconds)
      based on variable incremented in SysTick ISR. This implies that if HAL_Delay() is called from
      a peripheral ISR process, then the SysTick interrupt must have higher priority (numerically lower)
      than the peripheral interrupt. Otherwise the caller ISR process will be blocked.
      To change the SysTick interrupt priority you have to use HAL_NVIC_SetPriority() function.
      
@note The application needs to ensure that the SysTick time base is always set to 1 millisecond
      to have correct HAL operation.


@par Directory contents

    - DMA2D/DMA2D_MemToMemWithPFC/Inc/main.h                Main configuration file
    - DMA2D/DMA2D_MemToMemWithPFC/Inc/stm32f4xx_it.h        Interrupt handlers header file
    - DMA2D/DMA2D_MemToMemWithPFC/Inc/stm32f4xx_hal_conf.h  HAL configuration file
    - DMA2D/DMA2D_MemToMemWithPFC/Inc/ARGB8888_300x120.h    image to be converted and transferred. 
    - DMA2D/DMA2D_MemToMemWithPFC/Src/main.c                Main program  
    - DMA2D/DMA2D_MemToMemWithPFC/Src/stm32f4xx_it.c        Interrupt handlers
    - DMA2D/DMA2D_MemToMemWithPFC/Src/stm32f4xx_hal_msp.c   HAL MSP module    
    - DMA2D/DMA2D_MemToMemWithPFC/Src/system_stm32f4xx.c    STM32F4xx system clock configuration file


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
                                   