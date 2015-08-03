/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/


/* Project config */
#include "drone_config.h"

/* Include core modules */
#include "stm32f4xx.h"
#include "stm32f4xx_nucleo.h"

/* Include my libraries here */
#include "tm_stm32_delay.h"
#include "tm_stm32_usart.h"

#include "kb_stm32f4_mpu9150.h"
#include "kb_stm32f4_functions.h"

#include <stdio.h>

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);

int main(void) {
	KB_MPU9150_t MPU9150_Data;
	char str[120];

	char print_buffer_1[10];
	char print_buffer_2[10];
	char print_buffer_3[10];

	/* Init HAL layer */
	HAL_Init();

	/* Configure the system clock to 100 MHz */
	SystemClock_Config();

	/* Initialize delay */
	TM_DELAY_Init();

	/* Initialize USART, TX: PA9 RX: PA10 */
	TM_USART_Init(USART1, TM_USART_PinsPack_1, 115200);

	sprintf(str, "test started\r\n");
	TM_USART_Puts(USART1, str);

	/* Initialize MPU9150 sensor, SDA: PC9 SCL: PA8 */
	if (KB_MPU9150_Init(&MPU9150_Data, KB_MPU9150_Accelerometer_8G, KB_MPU9150_Gyroscope_2000s) != KB_MPU9150_Result_Ok) {
		/* Display error to user */
		TM_USART_Puts(USART1, "MPU9150 Error\n");

		/* Infinite loop */
		while (1);
	}

	while (1) {
		/* Every 0.5 second read sensors */
		if (TM_DELAY_Time() >= 500) {
			TM_DELAY_SetTime(0);
			/* Read all data from sensor */
			KB_MPU9150_ReadAll(&MPU9150_Data);
	
			/* Format data */
			conv_FloatToString(MPU9150_Data.Accelerometer_X, print_buffer_1);
			conv_FloatToString(MPU9150_Data.Accelerometer_Y, print_buffer_2);
			conv_FloatToString(MPU9150_Data.Accelerometer_Z, print_buffer_3);
	
			sprintf(str, "Accelerometer\n- X: %s, Y: %s, Z: %s,",
				print_buffer_1, print_buffer_2, print_buffer_3
			);
	
			/* Show to usart */
			TM_USART_Puts(USART1, str);
	
			/* Format data */
			conv_FloatToString(MPU9150_Data.Gyroscope_X, print_buffer_1);
			conv_FloatToString(MPU9150_Data.Gyroscope_Y, print_buffer_2);
			conv_FloatToString(MPU9150_Data.Gyroscope_Z, print_buffer_3);
	
	
			sprintf(str, " Gyroscope\n- X: %s, Y: %s, Z: %s,",
				print_buffer_1, print_buffer_2, print_buffer_3
			);
	
			/* Show to usart */
			TM_USART_Puts(USART1, str);
	
			/* Format data */
			conv_FloatToString(MPU9150_Data.Temperature, print_buffer_1);
	
			sprintf(str, " Temperature: %s,",
				print_buffer_1
			);
	
			/* Show to usart */
			TM_USART_Puts(USART1, str);
	
			/* Format data */
			conv_FloatToString(MPU9150_Data.Magnetometer_X, print_buffer_1);
			conv_FloatToString(MPU9150_Data.Magnetometer_Y, print_buffer_2);
			conv_FloatToString(MPU9150_Data.Magnetometer_Z, print_buffer_3);
	
			sprintf(str, " Magnetometer\n- X: %s, Y: %s, Z: %s\r\n",
				print_buffer_1, print_buffer_2, print_buffer_3
			);
	
			/* Show to usart */
			TM_USART_Puts(USART1, str);
		}
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
static void Error_Handler(void)
{
  while(1)
  {
  }
}
