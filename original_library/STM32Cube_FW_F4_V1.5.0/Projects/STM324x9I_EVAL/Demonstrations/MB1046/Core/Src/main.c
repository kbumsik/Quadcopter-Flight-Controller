/**
  ******************************************************************************
  * @file    main.c 
  * @author  MCD Application Team
  * @version V1.2.1
  * @date    13-March-2015
  * @brief   This file provides main program functions
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
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
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h" 
#include "k_mem.h" 
#include "k_bsp.h" 
#include "k_log.h"    
#include "k_rtc.h"    
#include "k_calibration.h"    
#include "k_storage.h"   

/** @addtogroup CORE
  * @{
  */

/** @defgroup MAIN
  * @brief main file
  * @{
  */ 

/** @defgroup MAIN_Private_TypesDefinitions
  * @{
  */ 
/**
  * @}
  */ 

/** @defgroup MAIN_Private_Defines
  * @{
  */ 
/**
  * @}
  */ 

/** @defgroup MAIN_Private_Macros
  * @{
  */ 
/**
  * @}
  */ 

/** @defgroup MAIN_Private_Variables
  * @{
  */
/**
  * @}
  */ 

/** @defgroup MAIN_Private_FunctionPrototypes
  * @{
  */ 
static void SystemClock_Config(void);
static void GUIThread(void const * argument);
static void TimerCallback(void const *n);

extern K_ModuleItem_Typedef  system_info;
extern K_ModuleItem_Typedef  video_player;
extern K_ModuleItem_Typedef  game_board;
extern K_ModuleItem_Typedef  image_browser;
extern K_ModuleItem_Typedef  cpu_bench;
extern K_ModuleItem_Typedef  file_browser;
extern K_ModuleItem_Typedef  usb_device;
extern K_ModuleItem_Typedef  audio_player;
extern K_ModuleItem_Typedef  camera_capture;


uint32_t GUI_FreeMem = 0;


/**
  * @}
  */ 

/** @defgroup MAIN_Private_Functions
  * @{
  */ 

/**
  * @brief  Main program
  * @param  None
  * @retval int
  */
int main(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  uint32_t FLatency;
  SystemSettingsTypeDef setting;    
  osTimerId lcd_timer;  
  
  /* STM32F4xx HAL library initialization:
       - Configure the Flash prefetch, instruction and Data caches
       - Configure the Systick to generate an interrupt each 1 msec
       - Set NVIC Group Priority to 4
       - Global MSP (MCU Support Package) initialization
  */
  HAL_Init();
 
  /* Configure the system clock to 168 MHz */
  SystemClock_Config();
  
  /* Initialize Joystick, Touch screen and LEDs */
  k_BspInit();
  k_LogInit(); 
  
  /* Initialize audio Interface */
  k_BspAudioInit();  
  
  /* Initialize RTC */
  k_CalendarBkupInit();  

  /* Add Modules */
  k_ModuleInit();  

  /* Initialize memory pools */
  k_MemInit();
  
  /* Create GUI task */
  osThreadDef(GUI_Thread, GUIThread, osPriorityHigh, 0, 20 * configMINIMAL_STACK_SIZE);
  osThreadCreate (osThread(GUI_Thread), NULL); 
  
  k_ModuleAdd(&video_player);
  k_ModuleOpenLink(&video_player, "emf");
  k_ModuleOpenLink(&video_player, "EMF");
  k_ModuleAdd(&image_browser);  
  k_ModuleOpenLink(&image_browser, "jpg"); 
  k_ModuleOpenLink(&image_browser, "JPG");
  k_ModuleOpenLink(&image_browser, "bmp"); 
  k_ModuleOpenLink(&image_browser, "BMP");
  k_ModuleAdd(&audio_player);  
  k_ModuleOpenLink(&audio_player, "wav"); 
  k_ModuleOpenLink(&audio_player, "WAV"); 
  k_ModuleAdd(&camera_capture);    
  k_ModuleAdd(&system_info);
  k_ModuleAdd(&file_browser);  
  k_ModuleAdd(&cpu_bench);  
  k_ModuleAdd(&game_board);  
  k_ModuleAdd(&usb_device);   
  
  /* Initialize GUI */
  GUI_Init();
  WM_MULTIBUF_Enable(1);
  GUI_SelectLayer(1);  
  
  /* Set General Graphical proprieties */
  k_SetGuiProfile();  

  /* Get General settings */
  setting.d32 = k_BkupRestoreParameter(CALIBRATION_GENERAL_SETTINGS_BKP);
    
  if(setting.b.use_180Mhz)
  {
    HAL_RCC_GetClockConfig(&RCC_ClkInitStruct, &FLatency);
    /* Select HSE as system clock source */
    RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK);
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

    HAL_RCC_GetOscConfig(&RCC_OscInitStruct);  
    RCC_OscInitStruct.PLL.PLLM = 25;
    RCC_OscInitStruct.PLL.PLLN = 360;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);
    
    HAL_PWREx_EnableOverDrive();
    
    /* Select PLL as system clock source */
    RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK);
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
  }  
  
  /* Create Touch screen Timer */
  osTimerDef(TS_Timer, TimerCallback);
  lcd_timer =  osTimerCreate(osTimer(TS_Timer), osTimerPeriodic, (void *)0);

  /* Start the TS Timer */
  osTimerStart(lcd_timer, 100);
    
  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  for( ;; );
}


/**
  * @brief This function provides accurate delay (in milliseconds) based 
  *        on SysTick counter flag.
  * @note This function is declared as __weak to be overwritten in case of other
  *       implementations in user file.
  * @param Delay: specifies the delay time length, in milliseconds.
  * @retval None
  */

void HAL_Delay (__IO uint32_t Delay)
{
  while(Delay) 
  {
    if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) 
    {
      Delay--;
    }
  }
}

/**
  * @brief  Start task
  * @param  argument: pointer that is passed to the thread function as start argument.
  * @retval None
  */
static void GUIThread(void const * argument)
{ 
  /* Initialize Storage Units */
  k_StorageInit();  
  
  /* Check for calibration */
  if(k_CalibrationIsDone() == 0)
  {
    GUI_SelectLayer(1);
    k_CalibrationInit();
  }  
  
  /* Demo Startup */
  k_StartUp();    
  
  GUI_SelectLayer(0);  
  
  /* Show the main menu */
  k_InitMenu();  
    
  /* Gui background Task */
  while(1) {
    GUI_Exec(); /* Do the background work ... Update windows etc.) */
    /* Toggle LED1 .. LED4 */
    BSP_LED_Toggle(LED1);
    BSP_LED_Toggle(LED2);
    BSP_LED_Toggle(LED3);
    BSP_LED_Toggle(LED4);      
    osDelay(250); /* Nothing left to do for the moment ... Idle processing */
    GUI_FreeMem = GUI_ALLOC_GetNumFreeBytes();

  }
}

/**
  * @brief  Timer callbacsk (40 ms)
  * @param  n: Timer index 
  * @retval None
  */
static void TimerCallback(void const *n)
{  
  k_TouchUpdate();
}

/** @brief  EXTI line detection callbacks
  * @param  GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == GPIO_PIN_8)
  {   
    if (SD_DETECT_PIN)
    {
      BSP_SD_DetectIT();
    } 
  }
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 168000000
  *            HCLK(Hz)                       = 168000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 25000000
  *            PLL_M                          = 25
  *            PLL_N                          = 336
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
  __HAL_RCC_PWR_CLK_ENABLE();

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
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig (&RCC_OscInitStruct);
  
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

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
