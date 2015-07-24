/**
  @page Camera_To_USBDisk Camera to USB Disk application
  
  @verbatim
  ******************** (C) COPYRIGHT 2015 STMicroelectronics *******************
  * @file    Camera/Camera_To_USBDisk/readme.txt 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    13-March-2015
  * @brief   Description of the Camera to USB Disk application.
  ******************************************************************************
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
   @endverbatim

@par Application Description

This application provides a short description of how to use the DCMI to interface with
camera module and display in continuous mode the picture on LCD and to save a picture 
in USB device.

At the beginning of the main program the HAL_Init() function is called to reset 
all the peripherals, initialize the Flash interface and the systick.
Then the SystemClock_Config() function is used to configure the system
clock (SYSCLK) to run at 180 MHz.

The Digital camera interface is configured to receive the capture from
the camera module mounted on STM32446E-EVAL evaluation board.
DMA2 Stream1 channel1 is configured to transfer the picture from DCMI peripheral
to an external RAM used by the LCD as a frame buffer.   

The camera module is configured to generate QVGA image resolution
and the LCD is configured to display QVGA image resolution

When the tamper button is pressed an image is saved under USBdisk.
  - LED1 is on to indicate the end of saving operation and a message is displayed 
    on LCD to indicate the beginning and the end of the saving operation.
  - LED3 is ON when any error occurs.

@note In case of camera fail, verify that camera sensor (U866 3M JS1434) connector 
      located at the bottom side of MB1183 RevB Camera module is well connected.
        
@note Care must be taken when using HAL_Delay(), this function provides accurate delay (in milliseconds)
      based on variable incremented in SysTick ISR. This implies that if HAL_Delay() is called from
      a peripheral ISR process, then the SysTick interrupt must have higher priority (numerically lower)
      than the peripheral interrupt. Otherwise the caller ISR process will be blocked.
      To change the SysTick interrupt priority you have to use HAL_NVIC_SetPriority() function.
      
@note The application needs to ensure that the SysTick time base is always set to 1 millisecond
      to have correct HAL operation.


@par Directory contents

    - Camera/Camera_To_USBDisk/Inc/stm32f4xx_hal_conf.h    HAL configuration file
    - Camera/Camera_To_USBDisk/Inc/main.h                  Main program header file
    - Camera/Camera_To_USBDisk/Inc/stm32f4xx_it.h          Interrupt handlers header file
    - Camera/Camera_To_USBDisk/Inc/ffconf.h                FAT file system module configuration file 
    - Camera/Camera_To_USBDisk/Inc/fatfs_storage.h         Header for fatfs_storage.c   
    - Camera/Camera_To_USBDisk/Inc/usbh_conf.h             Header for usbh_conf.c
    - Camera/Camera_To_USBDisk/Src/main.c                  Main program  
    - Camera/Camera_To_USBDisk/Src/stm32f4xx_it.c          Interrupt handlers   
    - Camera/Camera_To_USBDisk/Src/system_stm32f4xx.c      STM32F4xx system clock configuration file
    - Camera/Camera_To_USBDisk/Src/usbh_conf.c             USB configuration file.
    - Camera/Camera_To_USBDisk/Src/fatfs_storage.c         Storage (FatFs) driver


@par Hardware and Software environment
  
  - This application runs on STM32F446xx devices.
  
  - This application has been tested with STMicroelectronics STM32446E-EVAL 
    evaluation boards 

  - STM32446E-EVAL Set-up
    - Plug the USB key into the STM32446E-EVAL board through 'USB micro A-Male 
      to A-Female' cable to the connector CN9.
	@note Make sure that :
	- jumper JP4 is on FS position (2-3)
	- jumper JP7 is on FS position (2-3)     
    
@par How to use it ?

In order to make the program work, you must do the following :
 - Open your preferred toolchain 
 - Rebuild all files and load your image into target memory
 - Run the application
  
 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
                                   