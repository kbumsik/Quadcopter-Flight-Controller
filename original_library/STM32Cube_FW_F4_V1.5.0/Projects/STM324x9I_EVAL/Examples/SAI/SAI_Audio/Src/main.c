/**
  ******************************************************************************
  * @file    SAI/SAI_Audio/Src/main.c   
  * @author  MCD Application Team
  * @version V1.2.1
  * @date    13-March-2015
  * @brief   Main program body
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
  */

/* Includes ------------------------------------------------------------------*/
#include "audio_if.h"

/** @addtogroup STM32F4xx_HAL_Examples
  * @{
  */

/** @addtogroup SAI_Audio
  * @{
  */ 

/* Private define ------------------------------------------------------------*/
#define MESSAGE1   "    STM32F429xx     "
#define MESSAGE2   " Device running on  " 
#define MESSAGE3   "   STM324x9I-EVAL    "

   
/* Private typedef -----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
__IO uint32_t uwVolume = AUDIO_DEFAULT_VOLUME;

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /* STM32F4xx HAL library initialization:
       - Configure the Flash prefetch, instruction and Data caches
       - Configure the Systick to generate an interrupt each 1 msec
       - Set NVIC Group Priority to 4
       - Global MSP (MCU Support Package) initialization
     */
  HAL_Init();
  
  /* Configure the system clock to 180 MHz */
  SystemClock_Config();
  
  /* Configure LED1, LED2, LED3 and LED4 */
  BSP_LED_Init(LED1);
  BSP_LED_Init(LED2);
  BSP_LED_Init(LED3);
  BSP_LED_Init(LED4);
  
  /* Initialize the Push buttons */
  /* Wakeup button used for Volume Low */    
  BSP_PB_Init(BUTTON_WAKEUP, BUTTON_MODE_GPIO); 
  /* Tamper button used for Volume High */ 
  BSP_PB_Init(BUTTON_TAMPER, BUTTON_MODE_GPIO);  
  
  /* Initialize the LCD */
  BSP_LCD_Init();
  
  BSP_LCD_LayerDefaultInit(1, 0xC0130000);
  
  BSP_LCD_SelectLayer(1);
  
  /* Display message on EVAL LCD **********************************************/
  /* Clear the LCD */ 
  BSP_LCD_Clear(LCD_COLOR_BLUE);  
  
  /* Set the LCD Back Color */
  BSP_LCD_SetBackColor(LCD_COLOR_BLUE);
  
  /* Set the LCD Text Color */
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_DisplayStringAt(0, LINE(0), (uint8_t *)MESSAGE1, CENTER_MODE);
  BSP_LCD_DisplayStringAt(0, LINE(1), (uint8_t *)MESSAGE2, CENTER_MODE);
  BSP_LCD_DisplayStringAt(0, LINE(2), (uint8_t *)MESSAGE3, CENTER_MODE);
  
  /* Turn on LEDs available on EVAL *******************************************/
  BSP_LED_On(LED1);
  BSP_LED_On(LED2);
  BSP_LED_On(LED3);
  BSP_LED_On(LED4);
  
  /* Initialize the Audio codec and all related peripherals (SAI, I2C, IOs...) */  
  if(AUDIO_Init() == AUDIO_ERROR_NONE)
  {
    BSP_LCD_DisplayStringAt(0, LINE(4), (uint8_t *)"====================", CENTER_MODE);
    BSP_LCD_DisplayStringAt(0, LINE(5), (uint8_t *)"Tamper: Vol+        ", CENTER_MODE);
    BSP_LCD_DisplayStringAt(0, LINE(6), (uint8_t *)"Wakeup: Vol-        ", CENTER_MODE);
    BSP_LCD_DisplayStringAt(0, LINE(7), (uint8_t *)"====================", CENTER_MODE);
    BSP_LCD_DisplayStringAt(0, LINE(8), (uint8_t *)"  AUDIO CODEC   OK  ", CENTER_MODE);    
  }
  else
  {
    BSP_LCD_DisplayStringAt(0, LINE(4), (uint8_t *)"  AUDIO CODEC  FAIL ", CENTER_MODE);
    BSP_LCD_DisplayStringAt(0, LINE(5), (uint8_t *)" Try to reset board ", CENTER_MODE);
  }
  
  /* 
  Start playing the file from a circular buffer, once the DMA is enabled, it is 
  always in running state. Application has to fill the buffer with the audio data 
  using Transfer complete and/or half transfer complete interrupts callbacks 
  (EVAL_AUDIO_TransferComplete_CallBack() or EVAL_AUDIO_HalfTransfer_CallBack()...
  */
  AUDIO_Start();
  
  /* Display the state on the screen */
  BSP_LCD_DisplayStringAt(0, LINE(9), (uint8_t *)"       PLAYING...     ", CENTER_MODE);

   
  /* IMPORTANT:
     AUDIO_Process() is called by the SysTick Handler, as it should be called 
     within a periodic process */
   
  /* Infinite loop */
  while(1)
  {      
    /* Check on the Volume high button */
    if (BSP_PB_GetState(BUTTON_WAKEUP) != RESET)
    {
      /* Wait to avoid rebound */
      while (BSP_PB_GetState(BUTTON_WAKEUP) != RESET);
      
      /* Decrease volume by 5% */
      if (uwVolume > 5)
        uwVolume -= 5; 
      else
        uwVolume = 0; 
      
      /* Apply the new volume to the codec */
      BSP_AUDIO_OUT_SetVolume(uwVolume);
      BSP_LCD_DisplayStringAt(0, LINE(10), (uint8_t *)"       VOL:   -     ", CENTER_MODE); 
    }    
    
    /* Check on the Volume high button */
    if (BSP_PB_GetState(BUTTON_TAMPER) == RESET)
    {
      /* Wait to avoid rebound */
      while (BSP_PB_GetState(BUTTON_TAMPER) == RESET);
      
      /* Increase volume by 5% */
      if (uwVolume < 95)
        uwVolume += 5; 
      else
        uwVolume = 100; 
      
      /* Apply the new volume to the codec */
      BSP_AUDIO_OUT_SetVolume(uwVolume);
      BSP_LCD_DisplayStringAt(0, LINE(10), (uint8_t *)"       VOL:   +     ", CENTER_MODE);
    }  
    
    /* Toggle LED3 */
    BSP_LED_Toggle(LED3);
    
    /* Insert 100 ms delay */
    HAL_Delay(100);
    
    /* Toggle LED2 */
    BSP_LED_Toggle(LED2);
    
    /* Insert 100 ms delay */
    HAL_Delay(100);
  } 
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 180000000
  *            HCLK(Hz)                       = 180000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 25000000
  *            PLL_M                          = 25
  *            PLL_N                          = 360
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __PWR_CLK_ENABLE();

  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);
  
  /* Activate over drive */
  HAL_PWREx_ActivateOverDrive();
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
