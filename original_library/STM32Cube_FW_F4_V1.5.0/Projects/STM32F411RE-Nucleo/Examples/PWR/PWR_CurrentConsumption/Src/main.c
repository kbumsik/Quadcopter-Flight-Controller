/**
  ******************************************************************************
  * @file    PWR/PWR_CurrentConsumption/Src/main.c 
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    26-December-2014
  * @brief   This sample code shows how to use STM32F4xx PWR HAL API to enter
  *          and exit the stop mode.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
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
#include "main.h"

/** @addtogroup STM32F4xx_HAL_Examples
  * @{
  */

/** @addtogroup PWR_CurrentConsumption
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
__IO uint32_t uwCounter = 0;

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
  
  /* Configure LED2 */
  BSP_LED_Init(LED2);

  /* Configure the system clock to 100 MHz */
  SystemClock_Config();
  
  /* Enable Power Clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* Check and handle if the system was resumed from Standby mode */ 
  if(__HAL_PWR_GET_FLAG(PWR_FLAG_SB) != RESET)
  {
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);
  
    /* Infinite loop */
    while (1)
    {
      /* Toggle LED2 */
      BSP_LED_Toggle(LED2);
   
      /* Insert a 100ms delay */
      HAL_Delay(100);
    }
  }
  
  /* Configure USER Button */
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_GPIO);

  /* Wait until USER button is pressed to enter the Low Power mode */
  while(BSP_PB_GetState(BUTTON_KEY) != RESET)
  {
    /* Toggle LED2 */
    BSP_LED_Toggle(LED2);
   
    /* Inserted Delay */
    HAL_Delay(100);
  }
  /* Loop while USER Button is maintained pressed */
  while(BSP_PB_GetState(BUTTON_KEY) == RESET)
  {
  }

#if defined (SLEEP_MODE)
  /* Sleep Mode Entry 
      - System Running at PLL (168MHz)
      - Flash 5 wait state
      - Instruction and Data caches ON
      - Prefetch ON
      - Code running from Internal FLASH
      - All peripherals disabled.
      - Wake-up using EXTI Line (User Button)
   */
  SleepMode_Measure();
#elif defined (STOP_MODE)
  /* STOP Mode Entry 
      - RTC Clocked by LSI
      - Regulator in LP mode
      - HSI, HSE OFF and LSI OFF if not used as RTC Clock source  
      - No IWDG
      - FLASH in deep power down mode
      - Automatic Wake-up using RTC clocked by LSI (after ~20s)
   */
  StopMode_Measure();
#elif defined (STANDBY_MODE)
  /* STANDBY Mode Entry 
      - Backup SRAM and RTC OFF
      - IWDG and LSI OFF
      - Wake-up using WakeUp Pin (PA.00)
   */
  StandbyMode_Measure();

#elif defined (STANDBY_RTC_MODE)
  /* STANDBY Mode with RTC on LSI Entry 
      - RTC Clocked by LSI
      - IWDG OFF and LSI OFF if not used as RTC Clock source
      - Backup SRAM OFF
      - Automatic Wake-up using RTC clocked by LSI (after ~20s)
   */
  StandbyRTCMode_Measure();

#elif defined (STANDBY_RTC_BKPSRAM_MODE)
  /* STANDBY Mode with RTC on LSI Entry 
      - RTC Clocked by LSI
      - Backup SRAM ON
      - IWDG OFF
      - Automatic Wake-up using RTC clocked by LSI (after ~20s)
  */
  StandbyRTCBKPSRAMMode_Measure();
#endif

  if(uwCounter != 0)
  {
    BSP_LED_Init(LED2);
  }
  
  /* Infinite loop */
  while (1)
  {
    /* Toggle LED2 */
    BSP_LED_Toggle(LED2);
   
    /* Inserted Delay */
    HAL_Delay(100);
  }
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSI)
  *            SYSCLK(Hz)                     = 100000000
  *            HCLK(Hz)                       = 100000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            HSI Frequency(Hz)              = 16000000
  *            PLL_M                          = 16
  *            PLL_N                          = 400
  *            PLL_P                          = 4
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale2 mode
  *            Flash Latency(WS)              = 3
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  
  /* Enable HSI Oscillator and activate PLL with HSI as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 0x10;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* Turn LED2 on */
  BSP_LED_On(LED2);
  while(1)
  {
  }
}
    
/**
  * @brief SYSTICK callback
  * @param None
  * @retval None
  */
void HAL_SYSTICK_Callback(void)
{
  HAL_IncTick();
}

/**
  * @brief  Wake Up Timer callback
  * @param  hrtc : hrtc handle
  * @retval None
  */
void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc)
{
  /* NOTE : add the specific code to handle the RTC wake up interrupt */
  uwCounter = 1;
}

/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* NOTE : add the specific code to handle the wake up button interrupt */
  if(GPIO_Pin == KEY_BUTTON_PIN)
  { 
    uwCounter = 2;
  }
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
