/**
  ******************************************************************************
  * @file    USB_Device/DualCore_Standalone/Src/main.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    13-March-2015
  * @brief   USB device DualCore demo main file
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
#include "stm32446e_eval_io.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
USBD_HandleTypeDef USBD_Device_HS;
USBD_HandleTypeDef USBD_Device_FS;
__IO uint8_t HID_SendReport = 0, JoyButtonInitialized = 0;
uint8_t HID_Buffer[4];

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Toggle_Leds(void);
static void GetPointerData(uint8_t *pbuf);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /* STM32F446xx HAL library initialization */
  HAL_Init();
  
  /* Configure the system clock to 180 MHz */
  SystemClock_Config();
  
  /* Configure LED1 and LED3 */
  BSP_LED_Init(LED1);
  BSP_LED_Init(LED3);
  
  /* Initialize IO expander */
  BSP_IO_Init();

  /* Initialize Joystick */
  if (BSP_JOY_Init(JOY_MODE_GPIO) == 0)
  {
    JoyButtonInitialized = 1;
  }
  
  /* Init CDC Application */
  USBD_Init(&USBD_Device_HS, &VCP_Desc, 1);
  
  /* Init HID Application */
  USBD_Init(&USBD_Device_FS, &HID_Desc, 0);
  
  /* Add Supported Classes */
  USBD_RegisterClass(&USBD_Device_HS, &USBD_CDC);
  USBD_RegisterClass(&USBD_Device_FS, &USBD_HID);
  
    /* Add CDC Interface Class */
  USBD_CDC_RegisterInterface(&USBD_Device_HS, &USBD_CDC_fops);
  
  /* Start Device Process */
  USBD_Start(&USBD_Device_FS);
  USBD_Start(&USBD_Device_HS);
  
  while (1)
  {
    Toggle_Leds();
    if(HID_SendReport == 1)
    {
      HID_SendReport = 0;
      GetPointerData(HID_Buffer);
      
      /* send data though IN endpoint*/
      if((HID_Buffer[1] != 0) || (HID_Buffer[2] != 0))
      {
        USBD_HID_SendReport(&USBD_Device_FS, HID_Buffer, 4);
      }
    }
  }
}

/**
  * @brief  Gets Pointer Data.
  * @param  pbuf: Pointer to report
  * @retval None
  */
static void GetPointerData(uint8_t *pbuf)
{
  int8_t  x = 0, y = 0 ;
  
  if(JoyButtonInitialized == 1)
  {
    switch(BSP_JOY_GetState())
    {
    case JOY_LEFT:
      x -= CURSOR_STEP;
      break;  
      
    case JOY_RIGHT:
      x += CURSOR_STEP;
      break;
      
    case JOY_UP:
      y -= CURSOR_STEP;
      break;
      
    case JOY_DOWN:
      y += CURSOR_STEP;
      break;
      
    default:
      break;
    }
  }
  pbuf[0] = 0;
  pbuf[1] = x;
  pbuf[2] = y;
  pbuf[3] = 0;
}

/**
  * @brief  Toggles LEDs to show user input state.
  * @param  None
  * @retval None
  */
static void Toggle_Leds(void)
{
  static uint32_t ticks;
  
  if(ticks++ == 0xfffff)
  {
    BSP_LED_Toggle(LED1);
    BSP_LED_Toggle(LED3);
    ticks = 0;
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
void SystemClock_Config(void)
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
