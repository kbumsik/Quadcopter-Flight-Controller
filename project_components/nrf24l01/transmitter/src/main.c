/**
 * Keil project example for NRF24L01+
 *
 * Transmitter code
 *
 * Before you start, select your target, on the right of the "Load" button
 *
 * @author    Tilen Majerle
 * @email     tilen@majerle.eu
 * @website   http://stm32f4-discovery.com
 * @ide       Keil uVision 5
 * @conf      PLL parameters are set in "Options for Target" -> "C/C++" -> "Defines"
 * @packs     STM32F4xx/STM32F7xx Keil packs are requred with HAL driver support
 * @stdperiph STM32F4xx/STM32F7xx HAL drivers required
 */

#include "quadcopter_config.h"
/* Include core modules */
#include "stm32f4xx.h"
/* Include my libraries here */
#include "tm_stm32_nrf24l01.h"
#include "tm_stm32_usart.h"
#include <stdio.h>

/* My address */
uint8_t MyAddress[] = {
	0xE7,
	0xE7,
	0xE7,
	0xE7,
	0xE7
};
/* Receiver address */
uint8_t TxAddress[] = {
	0x7E,
	0x7E,
	0x7E,
	0x7E,
	0x7E
};

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void GPIO_Init(void);

/* Data received and data for send */
uint8_t dataOut[32], dataIn[32];

/* NRF transmission status */
TM_NRF24L01_Transmit_Status_t transmissionStatus;

/* Buffer for strings */
char str[40];

/* Timer variable */
uint32_t timer;

int main(void) {
	
	/* Init HAL layer */
	HAL_Init();
	
	/* Configure the system clock */
  	SystemClock_Config();

	/* Init GPIO */
	GPIO_Init();
	
	/* Initialize USART, TX: PA9, RX: PA10 */
	TM_USART_Init(USART1, TM_USART_PinsPack_1, 115200);
	
	/* Initialize NRF24L01+ on channel 15 and 32bytes of payload */
	/* By default 2Mbps data rate and 0dBm output power */
	/* NRF24L01 goes to RX mode by default */
	TM_NRF24L01_Init(15, 32);
	
	/* Set 2MBps data rate and -18dBm output power */
	TM_NRF24L01_SetRF(TM_NRF24L01_DataRate_2M, TM_NRF24L01_OutputPower_M18dBm);
	
	/* Set my address, 5 bytes */
	TM_NRF24L01_SetMyAddress(MyAddress);
	
	/* Set TX address, 5 bytes */
	TM_NRF24L01_SetTxAddress(TxAddress);

	while (1) {
		/* Fill data with something */
		sprintf((char *)dataOut, "abcdefghijklmnoszxABCDEFCBDA");

		/* Display on USART */
		TM_USART_Puts(USART1, "pinging: ");


		/* Reset time, start counting microseconds */
		timer = HAL_GetTick();

		/* Transmit data, goes automatically to TX mode */
		TM_NRF24L01_Transmit(dataOut);

		/* Turn on led to indicate sending */
		LED_ON();

		/* Wait for data to be sent */
		do {
			/* Get transmission status */
			transmissionStatus = TM_NRF24L01_GetTransmissionStatus();
		} while (transmissionStatus == TM_NRF24L01_Transmit_Status_Sending);

		/* Turn off led */
		LED_OFF();

		/* Go back to RX mode */
		TM_NRF24L01_PowerUpRx();

		/* Wait received data, wait max 100ms, if time is larger, then data were probably lost */
		while (!TM_NRF24L01_DataReady() && (HAL_GetTick()-timer) < 100);

		/* FIXME: sprintf does not work with type conversion from uint32_t to unsinged int */
		/* Format time */
		//sprintf(str, "%d ms", HAL_GetTick()-timer);

		/* Show ping time */
		//TM_USART_Puts(USART1, str);

		/* Get data from NRF2L01+ */
		TM_NRF24L01_GetData(dataIn);

		/* Check transmit status */
		if (transmissionStatus == TM_NRF24L01_Transmit_Status_Ok) {
			/* Transmit went OK */
			TM_USART_Puts(USART1, ": OK\n");
		} else if (transmissionStatus == TM_NRF24L01_Transmit_Status_Lost) {
			/* Message was LOST */
			TM_USART_Puts(USART1, ": LOST\n");
		} else {
			/* This should never happen */
			TM_USART_Puts(USART1, ": SENDING\n");
		}

		/* Do it every two seconds */
		HAL_Delay(2000);
	}
}

void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 200;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PA2   ------> USART2_TX
     PA3   ------> USART2_RX
*/
void GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOC_CLK_ENABLE();
  __GPIOH_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure LED for NUCLEO Board */
  GPIO_InitStruct.Pin = LED_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);
}


#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif
