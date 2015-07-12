/**
  @page QSPI_ExecuteInPlace example
  
  @verbatim
  ******************** (C) COPYRIGHT 2015 STMicroelectronics *******************
  * @file    QSPI/QSPI_ExecuteInPlace/readme.txt 
  * @author  MCD Application Team
  * @version V1.5.0
  * @date    06-March-2015
  * @brief   Description of the QSPI Execute In Place example.
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

@par Example Description

This example describes how to use the QSPI firmware library to execution of a part of the code from 
QSPI FLASH device.

In this example,the QSPI is interfacing with SPANSION S25FL512S FLASH or MICRON MT25QL512AB FLASH memory
and up to user to select the right flash connected on his STM32469I-EVAL board 
by uncommenting the required line in main.h:
- QSPI_FLASH_SPANSION: QSPI FLASH Spansion
- QSPI_FLASH_MICRON: QSPI FLASH Micron 

At the startup, the QSPI memory is erased, then the data are copied from the initialization
section of the flash to the QSPI memory using DMA.Then the QSPI is configured in memory-mapped 
mode to read and execute the code in a forever loop.

The code performs a GPIO toggle(PC13),it can be displayed using an oscilloscope.

@note
The QSPI Clock is configured to reach maximum frequency at 90 MHZ.

@note
You have to replace linker file by the one stored in project folder :
- EWARM-> stm32f4xx_flash.icf
- MDK-ARM-> STM32F446xx -> STM32F446xx.sct 
    Modify in Project options  :
     -	Linker -> “Use Memory Layout from Target Dialog” option disabled.
     -	Linker -> Scatter file -> .\STM32F446xx\STM32F446xx.sct selected.
- TrueSTUDIO-> STM32F446xx  -> STM32F446ZE_FLASH.ld

@note
Change in project options,C/C++ Compiler-> Language Conformance to standard with IAR extensions.

@par Directory contents 

  - QSPI/QSPI_ExecuteInPlace/system_stm32f4xx.c   STM32F4xx system clock configuration file
  - QSPI/QSPI_ExecuteInPlace/stm32f4xx_conf.h HAL Library configuration file
  - QSPI/QSPI_ExecuteInPlace/stm32f4xx_it.c       Interrupt handlers
  - QSPI/QSPI_ExecuteInPlace/stm32f4xx_it.h       Interrupt handlers header file
  - QSPI/QSPI_ExecuteInPlace/main.c               Main program
  - QSPI/QSPI_ExecuteInPlace/main.h               Header for main.c module  

@@par Hardware and Software environment 
 
  - This example runs on and STM32F446xx devices.
    
  - This example has been tested with STMicroelectronics STM32446E-EVAL 
    (STM32F446xx Devices) evaluation boards and can be easily 
    tailored to any other supported device and development board.

  - This example has been tested with STMSTM32446E-EVAL RevB board which includes
    the MB758 LCD board. 
     

  - STM32F446-EVAL Set-up
   Connect the following pins to an oscilloscope to monitor its waveform :
        - PC.13 (in CN13 connector)	

@par How to use it ?

In order to make the program work, you must do the following:
 - Copy all source files from this example folder to the template folder under
   Project\STM32F4xx_StdPeriph_Templates
 - Open your preferred toolchain
 - Rebuild all files and load your image into target memory
 - Run the example
  
 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
