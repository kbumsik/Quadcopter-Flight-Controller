/**
  ******************************************************************************
  * @file    Camera/Camera_To_USBDisk/Src/main.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    13-March-2015
  * @brief   This application describes how to configure the camera in continuous mode
             and save picture under USBDisk.
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

/** @addtogroup STM32F4xx_HAL_Applications
  * @{
  */

/** @addtogroup Camera_To_USBDisk
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define PRESSED_FIRST    0x00


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t ubPressedButton = PRESSED_FIRST;

FATFS MSC_FatFs;  /* File system object for USB disk logical drive */
FIL MyFile;       /* File object */
char MSC_Path[4]; /* USB Host logical drive path */
USBH_HandleTypeDef  hUSB_Host;

/* Image header */  
const uint32_t aBMPHeader[14]=
{0xB0364D42, 0x00000004, 0x00360000, 0x00280000, 0x01400000, 0x00F00000, 0x00010000, 
 0x00000020, 0xF5400000, 0x00000006, 0x00000000, 0x00000000, 0x00000000, 0x0000};

typedef enum {
  STORAGE_IDLE = 0,  
  STORAGE_READY,    
}MSC_ApplicationTypeDef;

MSC_ApplicationTypeDef Appli_state = STORAGE_IDLE;

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void PicturePrepare(void);
static void Error_Handler(void);
static void SavePicture(void);
static void CAMERA_Capture(void);
static void USBH_UserProcess(USBH_HandleTypeDef *phost, uint8_t id);

#define RGB565_TO_R(pixel)   (((pixel & 0x1F) << 3) | ((((pixel & 0x1F) << 3) & 0xE0) >> 5));
#define RGB565_TO_G(pixel)   (((pixel & 0x7E0) >> 3) | ((((pixel & 0x7E0) >> 3) & 0xC0) >> 6));
#define RGB565_TO_B(pixel)   (((pixel & 0xF800) >> 8) | ((((pixel & 0xF800) >> 8) & 0xE0) >> 5));
  
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /* STM32F4xx HAL library initialization:
       - Configure the Flash prefetch
       - Systick timer is configured by default as source of time base, but user 
         can eventually implement his proper time base source (a general purpose 
         timer for example or other time source), keeping in mind that Time base 
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
         handled in milliseconds basis.
       - Set NVIC Group Priority to 4
       - Low Level Initialization
     */
  HAL_Init();
  
  /* Configure the system clock to 180 MHz */
  SystemClock_Config();
  
  /* Initialize IO expander */
  BSP_IO_Init();

  /* Configure LED1 and LED3 */
  BSP_LED_Init(LED1);
  BSP_LED_Init(LED3);
  
  /*##-1- Init Host Library ##################################################*/
  USBH_Init(&hUSB_Host, USBH_UserProcess, 0);
  
  /* Add Supported Class */
  USBH_RegisterClass(&hUSB_Host, USBH_MSC_CLASS);
  
  /* Start Host Process */
  USBH_Start(&hUSB_Host);
  
  /*##-2- Configure User button ############################################*/
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_GPIO);
  
  /*##-3- Link the USB Host disk I/O driver ##################################*/
  FATFS_LinkDriver(&USBH_Driver, MSC_Path);

  /*##-4- Initialize the SDRAM and LCD ########################################*/ 
  BSP_LCD_Init(); 
  BSP_SDRAM_Init();

  /*##-5- Camera Initialization and start capture ############################*/
  /* Initialize the Camera */
  BSP_CAMERA_Init(RESOLUTION_R320x240);
  
  /* Start the Camera Capture */
  BSP_CAMERA_ContinuousStart((uint8_t *)CAMERA_FRAME_BUFFER);
   
  /*##-6- Run Application ####################################################*/
  while (1)
  { 
    /* USB Host Background task */
    USBH_Process(&hUSB_Host);

    switch(Appli_state)
    {
    case STORAGE_READY:
      CAMERA_Capture();
      break;
        
    case STORAGE_IDLE:
    default:
      break;      
    } 
  }
}

/**
  * @brief  Frame Event callback.
  * @param  None
  * @retval None
*/
void BSP_CAMERA_FrameEventCallback(void)
{
  /* Display on LCD */
  BSP_LCD_DrawRGBImage(0, 0, 320, 240, (uint8_t *)CAMERA_FRAME_BUFFER);
}

/**
  * @brief  Main routine for Mass Storage Class
  * @param  None
  * @retval None
  */
static void SavePicture(void)
{
  FRESULT res1, res2;     /* FatFs function common result code */
  uint32_t byteswritten;  /* File write count */
  
  static uint32_t counter = 0;
  uint8_t str[30];
  
  /* Suspend the camera capture */
  BSP_CAMERA_Suspend();
  
  /* Prepare the image to be saved */
  PicturePrepare();
  
  /* Format the string */
  sprintf((char *)str, "image_%lu.bmp", counter);
  
  /* Create and Open a new file object with write access */
  if(f_open(&MyFile, (const char*)str, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK) 
  {
    Error_Handler();
  }
  else
  {
    /* Write data to the BMP file */
    res1 = f_write(&MyFile, (uint32_t *)aBMPHeader, 54, (void *)&byteswritten);
    res2 = f_write(&MyFile, (uint16_t *)CAMERA_FRAME_BUFFER, (BSP_LCD_GetYSize()*BSP_LCD_GetXSize()*sizeof(uint32_t)), (void *)&byteswritten);
    
    if((res1 != FR_OK) || (res2 != FR_OK) || (byteswritten == 0))
    {
      Error_Handler();
    }
    else
    {
      /* Close the open BMP file */
      f_close(&MyFile);
      
      /* Success of the demo: no error occurrence */
      BSP_LED_On(LED1);
      BSP_LCD_SetTextColor(LCD_COLOR_DARKGREEN);
      BSP_LCD_DisplayStringAt(20, (BSP_LCD_GetYSize()-24), (uint8_t *)"   Saved  ", RIGHT_MODE);

      /* Wait for 2s */
      HAL_Delay(2000);

      counter++;
      BSP_LED_Off(LED1);
    }
  }
}

/**
  * @brief  Main routine for Camera capture
  * @param  None
  * @retval None
  */
static void CAMERA_Capture(void)
{
  while(1)
  {
    /* If user button is pressed and released save the picture */ 
    if(BSP_PB_GetState(BUTTON_KEY) != GPIO_PIN_RESET) 
    {
      if(BSP_PB_GetState(BUTTON_KEY) != GPIO_PIN_SET) 
      {
        SavePicture();
        BSP_CAMERA_Resume();
      }
    }
  }
}  

/**
  * @brief  Prepares the picture to be Saved in USB.
  * @param  None
  * @retval None
  */
static void PicturePrepare(void) 
{
  uint32_t address = CAMERA_FRAME_BUFFER;
  uint16_t x = 0;
  uint16_t y = 0;
  uint16_t tmp = 0;
  uint8_t aRGB[4];
  
  /* Go to the address of the last line of BMP file */
  address += ((BSP_LCD_GetXSize() * (BSP_LCD_GetYSize() - 1)) * 4);

  /* Read data from GRAM and swap it into SDRAM */
  for(y = 0; y < (BSP_LCD_GetYSize()); y++)
  { 
    for(x = 0; x < (BSP_LCD_GetXSize()); x++)
    {      
      /* Write data to the SDRAM memory */
      tmp  = BSP_LCD_ReadPixel(x, y); 
      
      aRGB[0] =  RGB565_TO_R(tmp);
      aRGB[1] =  RGB565_TO_G(tmp);
      aRGB[2] =  RGB565_TO_B(tmp);
      aRGB[3] =  0xFF;
      
      if(BSP_SDRAM_WriteData(address, (uint32_t *)aRGB, 1) != SDRAM_OK)
      {
        Error_Handler();
      }
      else
      {
        address += 4;
      }
    }
    address -= 8*BSP_LCD_GetXSize();
  }    
}

/**
  * @brief  User Process
  * @param  phost: Host handle
  * @param  id: Host Library user message ID
  * @retval None
  */
static void USBH_UserProcess(USBH_HandleTypeDef *phost, uint8_t id)
{  
  switch (id)
  { 
  case HOST_USER_SELECT_CONFIGURATION:
    break;
    
  case HOST_USER_DISCONNECTION:
    Appli_state = STORAGE_IDLE;
    f_mount(NULL, (TCHAR const*)"", 0);      
    break;
    
  case HOST_USER_CLASS_ACTIVE:
    /* Register the file system object to the FatFs module */
    if(f_mount(&MSC_FatFs, (TCHAR const*)MSC_Path, 0) != FR_OK)
    {
      /* FatFs Initialization Error */
      Error_Handler();
    }
    else
    {
      Appli_state = STORAGE_READY;
    }
    break;
  }
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
