/**
  @page CustomHID_Standalone USB Device Custom HID application
  
  @verbatim
  ******************** (C) COPYRIGHT 2015 STMicroelectronics *******************
  * @file    USB_Device/CustomHID_Standalone/readme.txt 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    13-March-2015
  * @brief   Description of the USB Custom HID application.
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
use USB device application based on the Custom HID Class on the STM32F446xx devices.

This is a typical application on how to use the STM32F446xx USB OTG Device peripheral, where the STM32 MCU is 
enumerated as a HID compliant device using the native PC Host HID driver to which the STM32446E-EVAL
board is connected.

At the beginning of the main program the HAL_Init() function is called to reset all the peripherals,
initialize the Flash interface and the systick. The user is provided with the SystemClock_Config()
function to configure the system clock (SYSCLK) to run at 180 MHz. The Full Speed (FS) USB module uses
internally a 48-MHz clock which is coming from a specific output of two PLLs PLL or PLL SAI.
In the High Speed (HS) mode the USB clock (60 MHz) is driven by the ULPI.

The 48 MHz clock for the USB FS can be derived from one of the two following sources:
  � PLL clock (clocked by the HSE): If the USB uses the PLL as clock source, the PLL VCO clock must be programmed
    to output 336 MHz frequency (USBCLK = PLLVCO/PLLQ).
  � PLLSAI clock (clocked by the HSE): If the USB uses the PLLSAI as clock source, the PLLSAI VCO clock must be programmed
    to output 384 MHz frequency (USBCLK = PLLSAIVCO/PLLSAIP).

To test the application, user must proceed as follows:
 - Start the "USB HID Demonstrator" PC applet, available for download from www.st.com, 
   and connect STM32 USB to PC
 - The device should be detected and shown in the USB HID target box
 - Press Graphic View button
 - Select "use SET_FEATURE" or "use SET_REPORT" in order to use SET_REPORT request to send HID Report
   for LED control
 - Use the potentiometer of the STM32446E-EVAL board to transfer the result of the converted voltage
  (via the ADC) the to the PC host (these values are sent to the PC using the endpoint1 IN). To use
  potentiometer, SB94 must be closed.   
 - Make sure that following report ID are configured: LED1 ID (0x1), LED2 ID(0x2), LED3 ID(0x3),
   LED4 ID(0x4), BUTTON1_ID(0x6) and Potentiometer_ID(0x7).
 - Select LEDs to switch on/off on the STM32446E-EVAL board: a SET_REPORT request will be sent

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
      - "USE_USB_HS" when using USB High Speed (HS) Core
      - "USE_USB_FS" when using USB Full Speed (FS) Core 


@par Directory contents

  - USB_Device/CustomHID_Standalone/Src/main.c                  Main program
  - USB_Device/CustomHID_Standalone/Src/system_stm32f4xx.c      STM32F4xx system clock configuration file
  - USB_Device/CustomHID_Standalone/Src/stm32f4xx_it.c          Interrupt handlers
  - USB_Device/CustomHID_Standalone/Src/stm32f4xx_hal_msp.c     HAL MSP module
  - USB_Device/CCustomHID_Standalone/Src/usbd_conf.c            General low level driver configuration
  - USB_Device/CustomHID_Standalone/Src/usbd_desc.c             USB device descriptor
  - USB_Device/CustomHID_Standalone/Inc/main.h                  Main program header file
  - USB_Device/CustomHID_Standalone/Inc/stm32f4xx_it.h          Interrupt handlers header file
  - USB_Device/CustomHID_Standalone/Inc/stm32f4xx_hal_conf.h    HAL configuration file
  - USB_Device/CustomHID_Standalone/Inc/usbd_conf.h             USB device driver Configuration file
  - USB_Device/CustomHID_Standalone/Inc/usbd_desc.h             USB device descriptor header file

	
@par Hardware and Software environment

  - This application runs on STM32F446xx devices.
    
  - This application has been tested with STMicroelectronics STM32446E-EVAL 
    evaluation boards and can be easily tailored to any other supported device 
    and development board.

  - STM32446E-EVAL RevB Set-up
    - Connect the STM32446E-EVAL board to the PC through 'USB micro A-Male 
      to A-Male' cable to the connector:
      - CN8: to use USB High Speed (HS) 
      - CN9: to use USB Full Speed (FS)
	@note Make sure that :
	- jumper JP4 is on FS position (2-3)
	- jumper JP7 is on FS position (2-3)
	- SB94 must be closed to use the potentiometer.	  
    
@par How to use it ?

In order to make the program work, you must do the following :
 - Open your preferred toolchain 
 - Rebuild all files and load your image into target memory
 - In the workspace toolbar select the project configuration:
   - STM32446E-EVAL_USBD-HS: to configure the project for STM32F446xx devices using USB OTG HS peripheral
   - STM32446E-EVAL_USBD-FS: to configure the project for STM32F446xx devices using USB OTG FS peripheral
 - Run the application
 
 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
 