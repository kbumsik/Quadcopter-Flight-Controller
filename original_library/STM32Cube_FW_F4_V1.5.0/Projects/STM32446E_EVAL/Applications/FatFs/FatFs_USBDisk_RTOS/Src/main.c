/**
  ******************************************************************************
  * @file    FatFs/FatFs_USBDisk_RTOS/Src/main.c 
  * @author  MCD Application Team
   * @version V1.0.0
  * @date    13-March-2015
  * @brief   Main program body
  *          This sample code shows how to use FatFs with USB disk drive in RTOS
  *          mode.
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

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
FATFS USBDISKFatFs;           /* File system object for USB disk logical drive */
FIL MyFile;                   /* File object */
char USBDISKPath[4];          /* USB Host logical drive path */
USBH_HandleTypeDef hUSB_Host; /* USB Host handle */

typedef enum {
  DISCONNECTION_EVENT = 1,  
  CONNECTION_EVENT,
}MSC_ApplicationTypeDef;

osMessageQId AppliEvent;

/* Private function prototypes -----------------------------------------------*/ 
static void SystemClock_Config(void);
static void Error_Handler(void);
static void USBH_UserProcess(USBH_HandleTypeDef *phost, uint8_t id);
static void MSC_Application(void);
static void StartThread(void const *argument);

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
  
  /*Initialize the IO module*/
  BSP_IO_Init ();
  
  /* Configure LED1 and LED3 */
  BSP_LED_Init(LED1);
  BSP_LED_Init(LED3);  
  
  /*##-1- Start task #########################################################*/
  osThreadDef(USER_Thread, StartThread, osPriorityNormal, 0, 8 * configMINIMAL_STACK_SIZE);
  osThreadCreate(osThread(USER_Thread), NULL);
  
  /*##-2- Create Application Queue ###########################################*/
  osMessageQDef(osqueue, 1, uint16_t);
  AppliEvent = osMessageCreate(osMessageQ(osqueue), NULL);
  
  /*##-3- Start scheduler ####################################################*/
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  for( ;; );
}

/**
  * @brief  Start task
  * @param  pvParameters not used
  * @retval None
  */
static void StartThread(void const *argument)
{
  osEvent event;
  
  /* Link the USB Host disk I/O driver */
  if(FATFS_LinkDriver(&USBH_Driver, USBDISKPath) == 0)
  {
    /* Init Host Library */
    USBH_Init(&hUSB_Host, USBH_UserProcess, 0);
    
    /* Add Supported Class */
    USBH_RegisterClass(&hUSB_Host, USBH_MSC_CLASS);
    
    /* Start Host Process */
    USBH_Start(&hUSB_Host);
    
    for( ;; )
    {
      event = osMessageGet(AppliEvent, osWaitForever);
      
      if(event.status == osEventMessage)
      {
        switch(event.value.v)
        {
        case CONNECTION_EVENT:
          MSC_Application();
          break;
          
        case DISCONNECTION_EVENT:
          f_mount(NULL, (TCHAR const*)"", 0);
          break; 
          
        default:
          break;
        }
      }
    }
  }
}

/**
  * @brief  Main routine for Mass Storage Class.
  * @param  None
  * @retval None
  */
static void MSC_Application(void)
{
  FRESULT res;                                          /* FatFs function common result code */
  uint32_t byteswritten, bytesread;                     /* File write/read counts */
  uint8_t wtext[] = "This is STM32 working with FatFs"; /* File write buffer */
  uint8_t rtext[100];                                   /* File read buffer */
  
  /* Register the file system object to the FatFs module */
  if(f_mount(&USBDISKFatFs, (TCHAR const*)USBDISKPath, 0) != FR_OK)
  {
    /* FatFs Initialization Error */
    Error_Handler();
  }
  else
  {  
    /* Create and Open a new text file object with write access */
    if(f_open(&MyFile, "STM32.TXT", FA_CREATE_ALWAYS | FA_WRITE) != FR_OK) 
    {
      /* 'STM32.TXT' file Open for write Error */
      Error_Handler();
    }
    else
    {
      /* Write data to the text file */
      res = f_write(&MyFile, wtext, sizeof(wtext), (void *)&byteswritten);
      
      if((byteswritten == 0) || (res != FR_OK))
      {
        /* 'STM32.TXT' file Write or EOF Error */
        Error_Handler();
      }
      else
      {
        /* Close the open text file */
        f_close(&MyFile);
        
        /* Open the text file object with read access */
        if(f_open(&MyFile, "STM32.TXT", FA_READ) != FR_OK)
        {
          /* 'STM32.TXT' file Open for read Error */
          Error_Handler();
        }
        else
        {
          /* Read data from the text file */
          res = f_read(&MyFile, rtext, sizeof(rtext), (void *)&bytesread);
          
          if((bytesread == 0) || (res != FR_OK))
          {
            /* 'STM32.TXT' file Read or EOF Error */
            Error_Handler();
          }
          else
          {
            /* Close the open text file */
            f_close(&MyFile);
            
            /* Compare read data with the expected data */
            if((bytesread != byteswritten))
            {                
              /* Read data is different from the expected data */
              Error_Handler();
            }
            else
            {
              /* Success of the demo: no error occurrence */
              BSP_LED_On(LED1);
            }     
          }
        }
      }
    }
  }
  
  /* Unlink the USB disk I/O driver */
  FATFS_UnLinkDriver(USBDISKPath);
}

/**
  * @brief  User Process
  * @param  phost: Host handle
  * @param  id: Host Library user message ID
  * @retval None
  */
static void USBH_UserProcess(USBH_HandleTypeDef *phost, uint8_t id)
{  
  switch(id)
  { 
  case HOST_USER_SELECT_CONFIGURATION:
    break;
    
  case HOST_USER_DISCONNECTION:
    osMessagePut(AppliEvent, DISCONNECTION_EVENT, 0);
    BSP_LED_Off(LED1); 
    BSP_LED_Off(LED3);  
    break;
    
  case HOST_USER_CLASS_ACTIVE:
    osMessagePut(AppliEvent, CONNECTION_EVENT, 0);
    break;
    
  default:
    break;
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
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 8
  *            PLL_N                          = 360
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  *         The USB clock configuration from PLLSAI:
  *            PLLSAIM                        = 8
  *            PLLSAIN                        = 384
  *            PLLSAIP                        = 8
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 2;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);
  
  /* Activate the OverDrive to reach the 180 MHz Frequency */  
  HAL_PWREx_EnableOverDrive();
  
  /* Select PLLSAI output as USB clock source */
  PeriphClkInitStruct.PLLSAI.PLLSAIM = 8;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 384;
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV8;
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CK48;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CK48CLKSOURCE_PLLSAIP;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* Turn LED3 on */
  BSP_LED_On(LED3);
  while(1)
  {
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
