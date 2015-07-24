/**
  ******************************************************************************
  * @file    TIM/TIM_6Steps/Src/main.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    13-March-2015
  * @brief   This example shows how to use TIM1 peripheral to generate 6 Steps.
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
#include "main.h"

/** @addtogroup STM32F4xx_HAL_Examples
  * @{
  */

/** @addtogroup TIM_6Steps
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Step Index */
 __IO uint32_t uwStep = 0;

/* Timer handler declaration */
TIM_HandleTypeDef    TimHandle;

/* Timer Output Compare Configuration Structure declaration */
TIM_OC_InitTypeDef              sPWMConfig1, sPWMConfig2, sPWMConfig3;
/* Timer Break Configuration Structure declaration */
TIM_BreakDeadTimeConfigTypeDef sBreakConfig;

/* Authorize TIM COM event generation */
__IO uint32_t uwAuthorizeTimComEvent = 0;

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);

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

  /* Configure LED3 */
  BSP_LED_Init(LED3);

  /*##-1- Configure the TIM peripheral #######################################*/ 
  /* ---------------------------------------------------------------------------
  TIM1 input clock (TIM1CLK) is set to 2 * APB2 clock (PCLK2), since APB2
    prescaler is different from 1.
    TIM1CLK = 2 * PCLK2
    PCLK1 = HCLK / 2
    => TIM1CLK = HCLK = SystemCoreClock
  --------------------------------------------------------------------------- */

  /* Initialize TIMx peripheral as follow:
       + Prescaler = 0
       + Period = 4095
       + ClockDivision = 0
       + Counter direction = Up
  */
  TimHandle.Instance = TIM1;
  
  TimHandle.Init.Period            = 4095;
  TimHandle.Init.Prescaler         = 0;
  TimHandle.Init.ClockDivision     = 0;
  TimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
  TimHandle.Init.RepetitionCounter = 0;

  if(HAL_TIM_OC_Init(&TimHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /*##-2- Configure the output channels ######################################*/
  /* Common configuration for all channels */
  sPWMConfig1.OCMode       = TIM_OCMODE_TIMING;
  sPWMConfig1.OCPolarity   = TIM_OCPOLARITY_HIGH;
  sPWMConfig1.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
  sPWMConfig1.OCIdleState  = TIM_OCIDLESTATE_SET;
  sPWMConfig1.OCNIdleState = TIM_OCNIDLESTATE_SET;
  sPWMConfig1.OCFastMode   = TIM_OCFAST_DISABLE;

  /* Set the pulse value for channel 1 */
  sPWMConfig1.Pulse = 2047;
  if(HAL_TIM_OC_ConfigChannel(&TimHandle, &sPWMConfig1, TIM_CHANNEL_1) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }
  
  /* Set the pulse value for channel 2 */
  sPWMConfig2 = sPWMConfig1;
  sPWMConfig2.Pulse = 1023;
  if(HAL_TIM_OC_ConfigChannel(&TimHandle, &sPWMConfig2, TIM_CHANNEL_2) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }
  
  /* Set the pulse value for channel 3 */
  sPWMConfig3 = sPWMConfig1;
  sPWMConfig3.Pulse = 511;
  if(HAL_TIM_OC_ConfigChannel(&TimHandle, &sPWMConfig3, TIM_CHANNEL_3) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }

  /*##-3- Configure the Break stage ##########################################*/
  sBreakConfig.OffStateRunMode  = TIM_OSSR_ENABLE;
  sBreakConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
  sBreakConfig.LockLevel        = TIM_LOCKLEVEL_OFF;
  sBreakConfig.BreakState       = TIM_BREAK_ENABLE;
  sBreakConfig.BreakPolarity    = TIM_BREAKPOLARITY_HIGH;
  sBreakConfig.AutomaticOutput  = TIM_AUTOMATICOUTPUT_ENABLE;
  sBreakConfig.DeadTime         = 1;
  
  if(HAL_TIMEx_ConfigBreakDeadTime(&TimHandle, &sBreakConfig) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }

  /*##-4- Configure the commutation event: software event ####################*/
  HAL_TIMEx_ConfigCommutationEvent_IT(&TimHandle, TIM_TS_NONE, TIM_COMMUTATION_SOFTWARE);

  /*##-5- Start signals generation ###########################################*/
  /*--------------------------------------------------------------------------*/
  /* Start channel 1 */
  if(HAL_TIM_OC_Start(&TimHandle, TIM_CHANNEL_1) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }
  /* Start channel 1N */
  if(HAL_TIMEx_OCN_Start(&TimHandle, TIM_CHANNEL_1) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }
  /*--------------------------------------------------------------------------*/

  
  /*--------------------------------------------------------------------------*/
  /* Start channel 2 */
  if(HAL_TIM_OC_Start(&TimHandle, TIM_CHANNEL_2) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }
  /* Start channel 2N */
  if(HAL_TIMEx_OCN_Start(&TimHandle, TIM_CHANNEL_2) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }
  /*--------------------------------------------------------------------------*/

  
  /*--------------------------------------------------------------------------*/
  /* Start channel 3 */
  if(HAL_TIM_OC_Start(&TimHandle, TIM_CHANNEL_3) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }
  /* Start channel 3N */
  if(HAL_TIMEx_OCN_Start(&TimHandle, TIM_CHANNEL_3) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }

  /* Authorize TIM COM event generation */
  uwAuthorizeTimComEvent = 1;

  while (1)
  {
  }
}

/**
  * @brief  Commutation event callback in non blocking mode
  * @param  htim : Timer handle
  * @retval None
  */
void HAL_TIMEx_CommutationCallback(TIM_HandleTypeDef *htim)
{
  /* Entry state */
  if (uwStep == 0)
  {
    /* Next step: Step 1 Configuration -------------------------------------- */
    sPWMConfig1.OCMode     = TIM_OCMODE_PWM1;
    HAL_TIM_PWM_ConfigChannel(&TimHandle, &sPWMConfig1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_1);
    HAL_TIMEx_OCN_Stop(&TimHandle, TIM_CHANNEL_1);

    /*  Channel3 configuration */
    sPWMConfig3.OCMode     = TIM_OCMODE_PWM1;
    HAL_TIM_PWM_ConfigChannel(&TimHandle, &sPWMConfig3, TIM_CHANNEL_3);
    HAL_TIMEx_OCN_Start(&TimHandle, TIM_CHANNEL_3);
    HAL_TIM_PWM_Stop(&TimHandle, TIM_CHANNEL_3);

    /*  Channel2 configuration */
    HAL_TIM_OC_Stop(&TimHandle, TIM_CHANNEL_2);
    HAL_TIMEx_OCN_Stop(&TimHandle, TIM_CHANNEL_2);
    uwStep = 1;
  }

  if (uwStep == 1)
  {
    /* Next step: Step 2 Configuration -------------------------------------- */
    /*  Channel1 configuration */
    /* Same configuration as the previous step */

    /*  Channel2 configuration */
    sPWMConfig2.OCMode     = TIM_OCMODE_PWM1;
    HAL_TIM_PWM_ConfigChannel(&TimHandle, &sPWMConfig2, TIM_CHANNEL_2);
    HAL_TIMEx_OCN_Start(&TimHandle, TIM_CHANNEL_2);

    /*  Channel3 configuration */
    HAL_TIMEx_OCN_Stop(&TimHandle, TIM_CHANNEL_3);

    uwStep++;
  }

  else if (uwStep == 2)
  {
    /* Next step: Step 3 Configuration -------------------------------------- */
    /*  Channel2 configuration */
    /* Same configuration as the previous step */

    /*  Channel3 configuration */
    sPWMConfig3.OCMode     = TIM_OCMODE_PWM1;
    HAL_TIM_PWM_ConfigChannel(&TimHandle, &sPWMConfig3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_3);

    /*  Channel1 configuration */
    HAL_TIM_OC_Stop(&TimHandle, TIM_CHANNEL_1);

    uwStep++;
  }

  else if (uwStep == 3)
  {
    /* Next step: Step 4 Configuration -------------------------------------- */
    /*  Channel3 configuration */
    /* Same configuration as the previous step */

    /*  Channel2 configuration */
    HAL_TIMEx_OCN_Stop(&TimHandle, TIM_CHANNEL_2);

    /*  Channel1 configuration */
    sPWMConfig1.OCMode     = TIM_OCMODE_PWM1;
    HAL_TIM_PWM_ConfigChannel(&TimHandle, &sPWMConfig1, TIM_CHANNEL_1);
    HAL_TIMEx_OCN_Start(&TimHandle, TIM_CHANNEL_1);

    uwStep++;
  }
  else if (uwStep == 4)
  {
    /* Next step: Step 5 Configuration -------------------------------------- */
    /*  Channel3 configuration */
    HAL_TIM_OC_Stop(&TimHandle, TIM_CHANNEL_3);

    /*  Channel1 configuration */
    /* Same configuration as the previous step */

    /*  Channel2 configuration */
    sPWMConfig2.OCMode     = TIM_OCMODE_PWM1;
    HAL_TIM_PWM_ConfigChannel(&TimHandle, &sPWMConfig2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_2);

    uwStep++;
  }

  else if (uwStep == 5)
  {
    /* Next step: Step 6 Configuration -------------------------------------- */
    /*  Channel3 configuration */
    sPWMConfig3.OCMode     = TIM_OCMODE_PWM1;
    HAL_TIM_PWM_ConfigChannel(&TimHandle, &sPWMConfig3, TIM_CHANNEL_3);
    HAL_TIMEx_OCN_Start(&TimHandle, TIM_CHANNEL_3);

    /*  Channel1 configuration */
    HAL_TIMEx_OCN_Stop(&TimHandle, TIM_CHANNEL_1);

    /*  Channel2 configuration */
    /* Same configuration as the previous step */

    uwStep++;
  }

  else
  {
    /* Next step: Step 1 Configuration -------------------------------------- */
    /*  Channel1 configuration */
    sPWMConfig1.OCMode     = TIM_OCMODE_PWM1;
    HAL_TIM_PWM_ConfigChannel(&TimHandle, &sPWMConfig1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_1);

    /*  Channel3 configuration */
    /* Same configuration as the previous step */

    /*  Channel2 configuration */
    HAL_TIM_OC_Stop(&TimHandle, TIM_CHANNEL_2);

    uwStep = 1;
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
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  HAL_StatusTypeDef ret = HAL_OK;

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
  
  ret = HAL_RCC_OscConfig(&RCC_OscInitStruct);
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }
  
  /* Activate the OverDrive to reach the 180 MHz Frequency */  
  ret = HAL_PWREx_EnableOverDrive();
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
  
  ret = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
  if(ret != HAL_OK)
  {
    while(1) { ; }
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
  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
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
