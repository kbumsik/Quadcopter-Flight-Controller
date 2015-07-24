/**
  @page BSP  Example on how to use the BSP drivers

  @verbatim
  ******************** (C) COPYRIGHT 2015 STMicroelectronics *******************
  * @file    BSP/readme.txt 
  * @author  MCD Application Team
  * @version V1.2.1
  * @date    13-March-2015
  * @brief   Description of the BSP example.
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

This example provides a description of how to use the different BSP drivers. 

At the beginning of the main program the HAL_Init() function is called to reset 
all the peripherals.
Then the SystemClock_Config() function is used to configure the systemclock (SYSCLK) 
to run at 84 MHz.

This example shows how to use the different functionalities of Gyroscope L3GD20 
and Accelerometer LSM303DLHC, Audio device CS43L22 and ST MEMS microphone (MP45DT02)
by switching between all tests using USER button. 

Firstly, push the User button to start first Test.  
4 LEDs will blink between each test.Press user key to start another test:

     1) ACCELEROMETER_MEMS_Test.    Device: "LSM303DLHC"
        LEDs 3,4,5 and 6 show board orientation. (X and Y axis)       
        
     2) GYROSCOPE_MEMS_Test.        Device: "L3GD20"
        LEDs 3,4,5 and 6 show board movement. (X and Y axis)   
     
     3) AudioPlay_Test(Need headphone). Device: "CS43L22"
        Plug a headphone to ear a 48K sound  /!\ Take care of yours ears.
        Default volume is 50%.
        A click on the board will pause the audio file play (LED6 & LED4 ON). 
		    Another click resumes audio file play (only LED6 on)
        @Note: Copy file "/Utilities/Media/Audio/art_of_gard_128K.bin" directly 
        in the STM32 flash at @0x08080000 

     3) AudioRecord_Test. Device: "MP45DT02"
        Record your voice (2 or 3 secs)
        When LED6 turns on, Plug a headphone to ear a 16K sound  /!\ Take care of yours ears.
        Default volume is 70%.

@note Care must be taken when using HAL_Delay(), this function provides accurate delay (in milliseconds)
      based on variable incremented in SysTick ISR. This implies that if HAL_Delay() is called from
      a peripheral ISR process, then the SysTick interrupt must have higher priority (numerically lower)
      than the peripheral interrupt. Otherwise the caller ISR process will be blocked.
      To change the SysTick interrupt priority you have to use HAL_NVIC_SetPriority() function.
      
@note The application needs to ensure that the SysTick time base is always set to 1 millisecond
      to have correct HAL operation.


@par Directory contents 

  - BSP/Src/system_stm32f4xx.c      STM32F4xx system clock configuration file
  - BSP/Src/main.c                  Main program
  - BSP/Src/mems.c                  Mems example and configuration file
  - BSP/Src/audio_play.c            Audio play file
  - BSP/Src/audio_record.c          Audio record file
  - BSP/Src/stm32f4xx_it.c          Interrupt handlers
  - BSP/Inc/main.h                  Main program header file
  - BSP/Inc/mems.h                  Mems example and configuration header file
  - BSP/Inc/audio_play.h            Audio play header file
  - BSP/Inc/audio_record.h          Audio record header file
  - BSP/Inc/stm32f4xx_hal_conf.h    HAL configuration file
  - BSP/Inc/stm32f4xx_it.h          Interrupt handlers header file
 
      
@par Hardware and Software environment  

  - This example runs on and STM32F401xCx devices.
    
  - This example has been tested with STMicroelectronics STM32F401-Discovery RevB 
    board and can be easily tailored to any other supported device and development board.

  - Use STLink utility, available on www.st.com or any other in system programming
    tool to load "/Utilities/Media/Audio/art_of_gard_448K.bin" file to the STM32 
    internal flash at the address 0x08080000.


@par How to use it ? 

In order to make the program work, you must do the following :
 - Open your preferred toolchain 
 - Rebuild all files and load your image into target memory
 - Run the example

 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
