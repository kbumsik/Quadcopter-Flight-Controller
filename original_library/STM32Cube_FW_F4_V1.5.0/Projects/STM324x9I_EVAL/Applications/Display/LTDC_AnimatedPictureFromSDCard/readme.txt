/**
  @page LTDC_AnimatedPictureFromSDCard LTDC animated picture from SD card application
  
  @verbatim
  ******************************************************************************
  * @file    Display/LTDC_AnimatedPictureFromSDCard/readme.txt 
  * @author  MCD Application Team
  * @version V1.2.1
  * @date    13-March-2015
  * @brief   Description of the LTDC animated picture from SD card application.
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

@par Application Description

  This application describes how to display on LCD an animated picture saved under
  microSD.
  
  The animated picture is the display of a sequence of images with a determined 
  frequency to be seen like one animated image.
 
  The user has to copy the two directories "BACK" and "TOP" available 
  with this under "/Utilities/Pictures/Animated" under the micro SD card root. 
 
  * Background picture display
  ----------------------------           
  Once the LCD, SD card and file system are initialized and configured, a check
  of the existence and the content of the "/BACK" directory is done.

  * Foreground animated pictures display
  -------------------------------------  
  A content check of the "/TOP" directory is done and the number of ".BMP" files 
  is retained.

@note : The maximum number of BMP file is fixed at 25. It can be raised 
        until reaching the maximum of SD card memory space.

@note : the system clock (SYSCLK) is configured to run at 175 MHz and 50 MHz is provided 
        at the output PLL divided by PLL_Q. This frequency permit to reach 25 MHz clock 
        needed for SD operation and in line with microSD specification. 

@note : When the SDcard is used the Camera module must be unplugged, this is due to
        the shared pins between the two devices. 
		
  The following steps are performed to scroll all the images stored in the 
  SD Card :

  - The foreground layer is set, the image copied from SD card (from "/TOP" directory)
    to intermediate SDRAM memory space and then copied to LCD frame buffer. 
 
  - The color keying feature is applied to remove the bottom of foreground layer (transparent)
    and then replaced by the background layer.
 
  - Jump to next image  

@note :  
  => If the "/Foreground" directory is empty, a warning message is displayed on the LCD : 
     "  No Bitmap files...  "
    
  => If the file type stored in the "/BACK" or "/TOP" directories is not supported,
     a warning message is displayed on the LCD : " file type not supported. "
    
  => If the SD card is not inserted, a warning message is displayed on the LCD :
     " Please insert SD Card. "
      
@note Care must be taken when using HAL_Delay(), this function provides accurate delay (in milliseconds)
      based on variable incremented in SysTick ISR. This implies that if HAL_Delay() is called from
      a peripheral ISR process, then the SysTick interrupt must have higher priority (numerically lower)
      than the peripheral interrupt. Otherwise the caller ISR process will be blocked.
      To change the SysTick interrupt priority you have to use HAL_NVIC_SetPriority() function.
      
@note The application needs to ensure that the SysTick time base is always set to 1 millisecond
      to have correct HAL operation.

-  LED3 is ON when the uSD disk I/O driver is not Linked;


@par Directory contents

    - Display/LTDC_AnimatedPictureFromSDCard/Inc/main.h                 Main configuration file
    - Display/LTDC_AnimatedPictureFromSDCard/Inc/stm32f4xx_it.h         Interrupt handlers header file
    - Display/LTDC_AnimatedPictureFromSDCard/Inc/stm32f4xx_hal_conf.h   HAL Configuration file 
    - Display/LTDC_AnimatedPictureFromSDCard/Inc/ffconf.h               FAT file system module configuration file
    - Display/LTDC_AnimatedPictureFromSDCard/Inc/fatfs_storage.h        Header for fatfs_storage.c
    - Display/LTDC_AnimatedPictureFromSDCard/Src/main.c                 Main program 
    - Display/LTDC_AnimatedPictureFromSDCard/Src/fatfs_storage.c        Storage (FatFs) driver
    - Display/LTDC_AnimatedPictureFromSDCard/Src/stm32f4xx_it.c         Interrupt handlers
    - Display/LTDC_AnimatedPictureFromSDCard/Src/system_stm32f4xx.c     STM32F4xx system clock configuration file


@par Hardware and Software environment  

  - This application runs on and STM32F429xx/439xx devices.
    
  - This application has been tested with STMicroelectronics STM324x9I-EVAL RevB
    evaluation boards and can be easily tailored to any other supported device 
    and development board.
 
 
@par How to use it ?

In order to make the program work, you must do the following :
 - The two directories "BACK" and "TOP" under "/Utilities/Pictures/Animated" 
   folder must be copied at the micro SD card root.
 - Open your preferred toolchain 
 - Rebuild all files and load your image into target memory
 - Run the application
  
 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
                                   