/**
  @page MTP_Standalone USB Host Media Transfer Protocol (MTP) application
  
  @verbatim
  ******************** (C) COPYRIGHT 2015 STMicroelectronics *******************
  * @file    USB_Host/MTP_Standalone/readme.txt 
  * @author  MCD Application Team
  * @version V1.2.1
  * @date    13-March-2015
  * @brief   Description of the USB Host MTP application.
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

This application is a part of the USB Host Library package using STM32Cube firmware. It describes how to use
USB host application based on the Media Transfer Protocol (MTP) on the STM32F4xx devices.

MTP is used to transfer data between computer/Host and devices connected to it. This protocol was 
developed for intelligent storage devices for downloading photographs from digital cameras music files
on digital audio players and media files on portable media players.

At the beginning of the main program the HAL_Init() function is called to reset all the peripherals,
initialize the Flash interface and the systick. The user is provided with the SystemClock_Config()
function to configure the system clock (SYSCLK) to run at 168 MHz. The Full Speed (FS) USB module uses
internally a 48-MHz clock, which is generated from an integrated PLL. In the High Speed (HS) mode the
USB clock (60 MHz) is driven by the ULPI.

It's worth noting that the system clock (SYSCLK) can be configured, depending on the used USB Core:
 - SYSCLK is set to 168 MHz: for FS Core (FS or HS-IN-FS), because used embedded PHY
                             requires 48 MHz clock, achieved only when system clock
                             is set to 168 MHz.
 - SYSCLK is set to 180 MHz: for only HS Core, since no embedded PHY is used.
 
When the application is started, the connected device (Android devices: Smartphone, Tablet...) 
is detected in MTP mode and gets initialized.
The STM32 MCU behaves as a MTP Host, it enumerates the device and extracts VID, PID, manufacturer name,
Serial no and product name information and displays it on the LCD screen. 
Then the Host detects the number of storage devices and displays on the LCD screen. The user will have 
to copy the .wav files on the root of the default storage. Only the files present on this space are visible 
by the application.

A menu is displayed and the user can select any operation from the menu using the Joystick buttons:
 - "Explore audio file" operation searches for all MTP objects, here are .Wav files and displays them 
    on the LCD screen.
 - "Start audio Player" operation starts playing the song from selected storage device.
   This application uses the I2S interface to stream audio data from USB Host to the audio codec implemented 
   on the evaluation board. Then audio downstream is driven to headphone or speaker.
   The USBH_AUDIO_ChangeOutBuffer() API is called while playing songs and the bytes are stored in a 
   circular buffer. Sampling frequency, Channels number and File size (duration) are detected. 
   The audio.c file contains a set of APIs for Audio Out playback, for example when the WAV file is 
   playing, the USBH_AUDIO_SetVolume() API is called to change the volume settings. 
 - "Re-Enumerate" operation performs a new Enumeration of the device.

@note Care must be taken when using HAL_Delay(), this function provides accurate delay (in milliseconds)
      based on variable incremented in SysTick ISR. This implies that if HAL_Delay() is called from
      a peripheral ISR process, then the SysTick interrupt must have higher priority (numerically lower)
      than the peripheral interrupt. Otherwise the caller ISR process will be blocked.
      To change the SysTick interrupt priority you have to use HAL_NVIC_SetPriority() function.
      
@note The application needs to ensure that the SysTick time base is always set to 1 millisecond
      to have correct HAL operation.

For more details about the STM32Cube USB Host library, please refer to UM1720  
"STM32Cube USB Host library".

@note The application is compliant with the MTP implementation for the Android 4.0.x/ 4.1.x/4.2.x and 4.3.x
      with USB debug mode disabled on the device.

@par USB Library Configuration

To select the appropriate USB Core to work with, user must add the following macro defines within the
compiler preprocessor (already done in the preconfigured projects provided with this application):
      - "USE_USB_HS" when using USB High Speed (HS) Core
      - "USE_USB_FS" when using USB Full Speed (FS) Core 
      - "USE_USB_HS" and "USE_USB_HS_IN_FS" when using USB High Speed (HS) Core in FS mode

It is possible to fine tune needed USB Host features by modifying defines values in USBH configuration
file �usbh_conf.h� available under the project includes directory, in a way to fit the application
requirements, such as:
- Level of debug: USBH_DEBUG_LEVEL
                  0: No debug messages
                  1: Only User messages are shown
                  2: User and Error messages are shown
                  3: All messages and internal debug messages are shown
   By default debug messages are displayed on the debugger IO terminal; to redirect the Library
   messages on the LCD screen, lcd_log.c driver need to be added to the application sources.
   

@par Directory contents

  - USB_Host/MTP_Standalone/Src/main.c                  Main program
  - USB_Host/MTP_Standalone/Src/system_stm32f4xx.c      STM32F4xx system clock configuration file
  - USB_Host/MTP_Standalone/Src/stm32f4xx_it.c          Interrupt handlers
  - USB_Host/MTP_Standalone/Src/menu.c                  MTP State Machine
  - USB_Host/MTP_Standalone/Src/usbh_conf.c             General low level driver configuration
  - USB_Host/MTP_Standalone/Src/audio.c                 Audio Out (playback) interface API
  - USB_Host/MTP_Standalone/Src/mtp.c                   Explore the MTP storage objects
  - USB_Host/MTP_Standalone/Inc/main.h                  Main program header file
  - USB_Host/MTP_Standalone/Inc/stm32f4xx_it.h          Interrupt handlers header file
  - USB_Host/MTP_Standalone/Inc/lcd_log_conf.h          LCD log configuration file
  - USB_Host/MTP_Standalone/Inc/stm32f4xx_hal_conf.h    HAL configuration file
  - USB_Host/MTP_Standalone/Inc/usbh_conf.h             USB Host driver Configuration file


@par Hardware and Software environment

  - This application runs on STM32F429xx/STM32F439xx devices.
    
  - This application has been tested with STMicroelectronics STM324x9I-EVAL RevB 
    evaluation boards and can be easily tailored to any other supported device 
    and development board.

  - STM324x9I-EVAL RevB Set-up
    - Plug the MTP device into the STM324x9I-EVAL board through 'USB micro A-Male 
      to A-Female' cable to the connector:
      - CN9 : to use USB High Speed (HS) 
      - CN14: to use USB Full Speed (FS) with embedded PHY(U7)
              Please ensure that jumper JP16 is not fitted.
      - CN15: to use USB HS-IN-FS.
              Note that some FS signals are shared with the HS ULPI bus, so some PCB rework is needed.
              For more details, refer to section 2.8 USB OTG2 HS & FS in UM1667
	- Supply the board with an external power (connector CN18)


@par How to use it ?

In order to make the program work, you must do the following :
 - Open your preferred toolchain 
 - Rebuild all files and load your image into target memory
 - In the workspace toolbar select the project configuration:
   - STM324x9I-EVAL_USBH-HS: to configure the project for STM32F4xx devices using USB OTG HS peripheral
   - STM324x9I-EVAL_USBH-FS: to configure the project for STM32F4xx devices using USB OTG FS peripheral
   - STM324x9I-EVAL_USBH-HS-IN-FS: to configure the project for STM32F4xx devices and use USB OTG HS 
                                   peripheral In FS (using embedded PHY).
 - Run the application
 
 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */