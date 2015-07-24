/**
  @page Demo   Demo STM324xG-EVAL
 
  @verbatim
  ******************** (C) COPYRIGHT 2015 STMicroelectronics *******************
  * @file    Demonstrations/readme.txt 
  * @author  MCD Application Team
  * @version V1.2.1
  * @date    13-March-2015
  * @brief   Description of STM324xG-EVAL Demonstration
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

@par Demo Description

The STM32Cube Demonstration platform comes on top of the STM32CubeTM as a firmware
package that offers a full set of software components based on a modules architecture
allowing re-using them separately in standalone applications. All these modules are
managed by the STM32Cube Demonstration kernel allowing to dynamically adding new
modules and access to common resources (storage, graphical components and widgets,
memory management, Real-Time operating system)

The STM32Cube Demonstration platform is built around the powerful graphical library
STemWin and the FreeRTOS real time operating system and uses almost the whole STM32
capability to offer a large scope of usage based on the STM32Cube HAL BSP and several
middleware components.
  
Below you find an overview of the different offered module in the demonstration:

 + System
 --------
 The system module provides three control tabs: 
  - system information 
  - general settings
  - clock settings 
 To set the global demonstration settings. The system module retrieves
 demonstration information from internal kernel settings data structures and acts 
 on the several kernel services to changes settings.
 
 + File browser
 --------------
 The File browser module is a system module that allows to explore the connected
 storage unit(s), to delete or to open a selected file. The file list structure 
 is built during the media connection and updated after a connection status change of one
 of the used media.
 
 + Game
 -------
 The game coming in the STM32Cube demonstration is based on the Reversi game. It is a
 strategy board game for two players, played on an 8�8 board. The goal of the game is to
 have the majority of disks turned to display your color when the last playable empty square
 is filled.

 + Benchmark
 -----------
 The Benchmark module is a system module that allows measure the graphical performance
 by measuring the time needed to draw several colored rectangles in random position with
 random size during a specific period. The result is given in pixel per second.
 
 + Audio
 -------
 The audio player module provides a complete audio solution based on the STM32F4xx and
 delivers a high-quality music experience. It supports playing music in WAV format but may
 be extended to support other compressed formats such as MP3 and WMA audio formats.
 You can use the *.wav audio provided under "Utilities/Media/Audio" or any other ones.
  
 + Video
 -------
 The video player module provides a video solution based on the STM32F4xx and STemWin
 movie API. It supports playing movie in emf format.
 You can use the *.emf video file provided under "Utilities/Media/Video".
 
 + USB Mass storage Device
 -------------------------
 The USB device module includes mass storage device application using the MicroSD
 memory. It uses the USB OTG FS peripheral as the USB OTG HS is used for the USB disk
 Flash storage unit.
 

 + Camera
 --------
 The camera application allows to directly and permanently display on the LCD the image
 captured using the camera module. It is also possible to take a snapshot and save it to a
 customizable location in the storage unit.
 In addition to brightness and contrast which are adjustable, several effects can be applied to
 the output image: black and white, negative, antique...etc. Note that all these effects can be
 applied in runtime.
 
 + Image viewer
 --------------
 The Image viewer module allows displaying bmp and jpg pictures. It is possible to load the
 full images list from a folder or to add the images manually to the playlist. Once the playlist is
 created, navigation between pictures can be done either via Next and previous buttons or by
 enabling the slide show mode. The slide show timer can be changed on the fly (there is no
 need to restart the module).
 You can use the *.jpg image files provided under "Utilities/Media/Pictures/JPG" or any other ones.
 
 For more details about the demonstration modules please refers to  STM32CubeF4 demonstration (UM1743)

@note Care must be taken when using HAL_Delay(), this function provides accurate delay (in milliseconds)
      based on variable incremented in SysTick ISR. This implies that if HAL_Delay() is called from
      a peripheral ISR process, then the SysTick interrupt must have higher priority (numerically lower)
      than the peripheral interrupt. Otherwise the caller ISR process will be blocked.
      To change the SysTick interrupt priority you have to use HAL_NVIC_SetPriority() function.
      
@note The application needs to ensure that the SysTick time base is always set to 1 millisecond
      to have correct HAL operation.

@par Hardware and Software environment

  - This demonstration runs on STM32F407xx/417xx devices.
    
  - This demonstration has been tested with STM324xG-EVAL RevC evaluation board and can be
    easily tailored to any other supported device and development board. 

@par How to use it ? 

In order to make the program work, you must do the following :
 - Open your preferred toolchain 
 - Rebuild all files and load your image into target memory
 - Run the demonstration

 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
 
