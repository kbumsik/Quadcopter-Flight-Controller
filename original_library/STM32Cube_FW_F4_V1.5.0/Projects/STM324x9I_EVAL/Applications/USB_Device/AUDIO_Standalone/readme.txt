/**
  @page AUDIO_Standalone USB Device AUDIO application
  
  @verbatim
  ******************** (C) COPYRIGHT 2015 STMicroelectronics *******************
  * @file    USB_Device/AUDIO_Standalone/readme.txt 
  * @author  MCD Application Team
  * @version V1.2.1
  * @date    13-March-2015
  * @brief   Description of the USB Device AUDIO application.
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

This application is a part of the USB Device Library package using STM32Cube firmware. It describes how to 
use USB device application based on the AUDIO Class implementation of an audio streaming 
(Out: Speaker/Headset) capability on the STM32F4xx devices.

It follows the "Universal Serial Bus Device Class Definition for Audio Devices Release 1.0 March 18, 
1998" defined by the USB Implementers Forum for reprogramming an application through USB-FS-Device. 
Following this specification, it is possible to manage only Full Speed USB mode (High Speed is not supported).
This class is natively supported by most Operating Systems: no need for specific driver setup.

This is a typical application on how to use the STM32F4xx USB OTG Device peripheral and SAI peripheral to 
stream audio data from USB Host to the audio codec implemented on the STM324x9I-EVAL board.

At the beginning of the main program the HAL_Init() function is called to reset all the peripherals,
initialize the Flash interface and the systick. The user is provided with the SystemClock_Config()
function to configure the system clock (SYSCLK) to run at 168 MHz. The Full Speed (FS) USB module uses
internally a 48-MHz clock, which is generated from an integrated PLL. In the High Speed (HS) mode the
USB clock (60 MHz) is driven by the ULPI.

It's worth noting that the system clock (SYSCLK) can be configured, depending on the used USB Core:
 - SYSCLK is set to 168 MHz: for FS Core (FS or HS-IN-FS), because used embedded PHY
                             requires 48 MHz clock, achieved only when system clock
                             is set to 168 MHz.
 
The device supports the following audio features:
  - Pulse Coded Modulation (PCM) format
  - sampling rate: 48KHz. 
  - Bit resolution: 16
  - Number of channels: 2
  - No volume control
  - Mute/Unmute capability
  - Asynchronous Endpoints

In order to overcome the difference between USB clock domain and STM32 clock domain,
the Add-Remove mechanism is implemented at class driver level.
This is a basic solution that doesn't require external components. It is based
on adding/removing one sample at a periodic basis to speedup or slowdown
the audio output process. This allows to resynchronize it with the input flow.

@note Care must be taken when using HAL_Delay(), this function provides accurate delay (in milliseconds)
      based on variable incremented in SysTick ISR. This implies that if HAL_Delay() is called from
      a peripheral ISR process, then the SysTick interrupt must have higher priority (numerically lower)
      than the peripheral interrupt. Otherwise the caller ISR process will be blocked.
      To change the SysTick interrupt priority you have to use HAL_NVIC_SetPriority() function.
      
@note The application needs to ensure that the SysTick time base is always set to 1 millisecond
      to have correct HAL operation.
      
For more details about the STM32Cube USB Device library, please refer to UM1734 
"STM32Cube USB Device library".
      

@par USB Library Configuration

To select the appropriate USB Core to work with, user must add the following macro defines within the
compiler preprocessor (already done in the preconfigured projects provided with this application):
      - "USE_USB_FS" when using USB Full Speed (FS) Core 
      - "USE_USB_HS" and "USE_USB_HS_IN_FS" when using USB High Speed (HS) Core in FS mode

It is possible to fine tune needed USB Device features by modifying defines values in USBD configuration
file �usbd_conf.h� available under the project includes directory, in a way to fit the application
requirements, such as:      
 - USBD_AUDIO_FREQ, specifying the sampling rate conversion from original audio file sampling rate to the
   sampling rate supported by the device.   
    
         
@par Directory contents

  - USB_Device/AUDIO_Standalone/Src/main.c                  Main program
  - USB_Device/AUDIO_Standalone/Src/system_stm32f4xx.c      STM32F4xx system clock configuration file
  - USB_Device/AUDIO_Standalone/Src/stm32f4xx_it.c          Interrupt handlers
  - USB_Device/AUDIO_Standalone/Src/usbd_audio_if.c         USBD Audio interface
  - USB_Device/AUDIO_Standalone/Src/usbd_conf.c             General low level driver configuration
  - USB_Device/AUDIO_Standalone/Src/usbd_desc.c             USB device AUDIO descriptor                                    
  - USB_Device/AUDIO_Standalone/Inc/main.h                  Main program header file
  - USB_Device/AUDIO_Standalone/Inc/stm32f4xx_it.h          Interrupt handlers header file
  - USB_Device/AUDIO_Standalone/Inc/stm32f4xx_hal_conf.h    HAL configuration file
  - USB_Device/AUDIO_Standalone/Inc/usbd_conf.h             USB device driver Configuration file
  - USB_Device/AUDIO_Standalone/Inc/usbd_desc.h             USB device AUDIO descriptor header file
  - USB_Device/AUDIO_Standalone/Inc/usbd_audio_if.h         USBD Audio interface header file  


@par Hardware and Software environment

  - This application runs on STM32F429xx/STM32F439xx devices.
    
  - This application has been tested with STMicroelectronics STM324x9I-EVAL RevB 
    evaluation boards and can be easily tailored to any other supported device 
    and development board.

  - STM324x9I-EVAL RevB Set-up
    - Connect the STM324x9I-EVAL board to the PC for audio streaming through 
     'USB micro A-Male to A-Male' cable to the connector:
      - CN14: to use USB Full Speed (FS)
              Please ensure that jumper JP16 is not fitted.
      - CN15: to use USB HS-IN-FS.
              Note that some FS signals are shared with the HS ULPI bus, so some PCB rework is needed.
              For more details, refer to section 2.8 USB OTG2 HS & FS in UM1667        
    - Use CN23 connector to connect the board to external headset


@par How to use it ? 

In order to make the program work, you must do the following :
 - Open your preferred toolchain 
 - Rebuild all files and load your image into target memory
 - In the workspace toolbar select the project configuration:
   - STM324x9I-EVAL_USBH-FS: to configure the project for STM32F4xx devices using USB OTG FS peripheral
   - STM324x9I-EVAL_USBH-HS-IN-FS: to configure the project for STM32F4xx devices and use USB OTG HS 
                                   peripheral In FS (using embedded PHY).
 - Run the application
 - Open an audio player application (Windows Media Player) and play music on the PC host

 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
