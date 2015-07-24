/**
  @page STemWin_Simulation Readme file
 
  @verbatim
  ******************** (C) COPYRIGHT 2014 STMicroelectronics *******************
  * @file    WIN32/STemWin_Simulation/readme.txt 
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    26-December-2014  
  * @brief   Description of the STemWin Simulation project. 
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

@par Description

This STemWin simulation project allows you to compile the same C source on your Windows
PC using a native (typically Microsoft) compiler and create an executable for your own
application. Doing so allows the following:
 - Design of the user interface on your PC (no hardware required!).
 - Debugging of the user interface program.
 - Creation of demos of your application in windows executable file .


The STemWin simulation requires Microsoft Visual C++ (version 6.00 or higher) and the
integrated development environment (IDE) which comes with it. You will see a simulation
of your LCD on your PC screen, which has the same resolution in X and Y and can display
the exact same colors as your LCD once it has been properly configured.
The entire graphic library API and Window Manager API of the simulation are identical
to those on your target system; all functions will behave in the very same way as
on the target hardware since the simulation uses the same C source code as the target
system. The difference lies only in the lower level of the software: the display
driver. Instead of using the actual display driver, the PC simulation uses a simulation
driver which writes into a bitmap. The bitmap is then displayed on your screen using
a second thread of the simulation. This second thread is invisible to the application;
it behaves just as if the LCD routines were writing directly to the display.

The STemWinxxx_WIN32.lib file contains a full library which allows you to evaluate all
available features of STemWin. You will not be able to view the source code of STemWin or 
the simulation, but you will still be able to become familiar with what STemWin can do.

The project allows also to run the different Segger samples that can be downloaded
from here: http://www.segger.com/emwin-samples.html
To do this, user has only to replace the file "MainTask.c" into the project workspace
by the downloaded one.


@par Software environment 
  - This Simulation project runs on VS Express for Desktop on windows platform
  - This project has been tested with Microsoft Visual Studio Express 2012 for Windows Desktop on Windows 7


@par How to use it ? 

In order to use the simulation project :
  - Open the Simulation.vcxproj file 
  - Rebuild all files
  - Start Debugging (F5)
  - A "hello world" message will be shown in the Simulation display
  - the MainTask.c could be overwritten by user code to run his own code
 
 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
