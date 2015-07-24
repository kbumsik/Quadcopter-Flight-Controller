/**
  ******************************************************************************
  * @file    FLASH/FLASH_WriteProtection/Src/main.c 
  * @author  MCD Application Team
  * @version V1.2.1
  * @date    13-March-2015
  * @brief   This example provides a description of how to erase and program the 
  *			     STM32F4xx FLASH.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
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

/** @addtogroup FLASH_WriteProtection
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define MESSAGE1     "        STM32F429X         " 
#define MESSAGE2     "      Device running on    " 

#ifdef FLASH_BANK1
 #define MESSAGE3    "        FLASH BANK1        "
 #define MESSAGE7     "         Swap BFB2         "
#else
 #define MESSAGE3    "        FLASH BANK2        "
 #define MESSAGE7     "         Swap BFB1         "
#endif

#define MESSAGE5     "      PUSH Key button      "



/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
FLASH_OBProgramInitTypeDef    OBInit; 
FLASH_AdvOBProgramInitTypeDef AdvOBInit;

uint8_t Message[24];
uint32_t VAR = 0;
__IO uint32_t tmp = 0;

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
  
  /* Configure TAMPER Button */
  BSP_PB_Init(BUTTON_TAMPER, BUTTON_MODE_GPIO);
  
  /* Configure LED3 and LED4 */
  BSP_LED_Init(LED3);
  BSP_LED_Init(LED4);

  /*##-1- Initialize the LCD #################################################*/
  /* Initialize the LCD */
  BSP_LCD_Init();
  
  /* Set LCD font */
  BSP_LCD_SetFont(&Font20);
   
  /* LCD Layer Initialization */
  BSP_LCD_LayerDefaultInit(1, 0xC0130000); 
  
  BSP_LCD_SelectLayer(1);
  
  BSP_LCD_DisplayOn();
    
  /*##-2- Display messages on LCD ############################################*/ 
  
  /* Clear the LCD */ 
  BSP_LCD_Clear(LCD_COLOR_WHITE);
  
#ifdef FLASH_BANK1
  BSP_LCD_SetBackColor(LCD_COLOR_BLUE);
#else
  BSP_LCD_SetBackColor(LCD_COLOR_RED);
#endif /* BOOT_FROM_BANK1 */
  
  /* Set the LCD Text Color */
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  
  /* Display LCD messages */
  BSP_LCD_DisplayStringAtLine(0, (uint8_t *)MESSAGE1);
  BSP_LCD_DisplayStringAtLine(1, (uint8_t *)MESSAGE3);
  BSP_LCD_DisplayStringAtLine(2, (uint8_t *)MESSAGE5);
  BSP_LCD_DisplayStringAtLine(3, (uint8_t *)MESSAGE7);
  
  /* Turn on LEDs */
  BSP_LED_On(LED3);
  BSP_LED_On(LED4);
  
  /* Infinite loop */
  while (1)
  {
    /*--- If Wake-up button is pushed, Set BFB2 bit to enable boot from Bank2
    (active after next reset, w/ Boot pins set in Boot from Flash memory position ---*/
    
    /* Wait for TAMPER push-button is released */
    while (BSP_PB_GetState(BUTTON_TAMPER) != RESET)
    {
      #ifdef FLASH_BANK1        
    /* Toggle LED3 */
    BSP_LED_Toggle(LED3);
    
    /* Insert 50 ms delay */
    HAL_Delay(50);
#else
    /* Toggle LED4 */    
    BSP_LED_Toggle(LED4);
    
    /* Insert 50 ms delay */
    HAL_Delay(50);
#endif    
    }
    
    /* Set BFB2 bit to enable boot from Flash Bank2 */
    /* Allow Access to Flash control registers and user Falsh */
    HAL_FLASH_Unlock();
  
    /* Allow Access to option bytes sector */ 
    HAL_FLASH_OB_Unlock();
    
    /* Get the Dual boot configuration status */
    AdvOBInit.OptionType = OPTIONBYTE_BOOTCONFIG;
    HAL_FLASHEx_AdvOBGetConfig(&AdvOBInit);

    /* Enable/Disable dual boot feature */
    if (((AdvOBInit.BootConfig) & (FLASH_OPTCR_BFB2)) == FLASH_OPTCR_BFB2)
    {
      AdvOBInit.BootConfig = OB_DUAL_BOOT_DISABLE;
      HAL_FLASHEx_AdvOBProgram (&AdvOBInit);
    }
    else
    {
      AdvOBInit.BootConfig = OB_DUAL_BOOT_ENABLE;
      HAL_FLASHEx_AdvOBProgram (&AdvOBInit);
    }
    
    /* Start the Option Bytes programming process */  
    if (HAL_FLASH_OB_Launch() != HAL_OK)
    {
      /* User can add here some code to deal with this error */
      while (1)
      {
      }
    }
    
    /* Prevent Access to option bytes sector */ 
    HAL_FLASH_OB_Lock();
    
    /* Disable the Flash option control register access (recommended to protect 
    the option Bytes against possible unwanted operations) */
    HAL_FLASH_Lock();    
    
    /* Initiates a system reset request to reset the MCU */
    HAL_NVIC_SystemReset();  
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
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);
  
  /* Activate the Over-Drive mode */
  HAL_PWREx_EnableOverDrive();
  
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
