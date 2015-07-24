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

int main(void) {
	KB_MPU9150_t MPU9150_Data;
	char str[120];

	char print_buffer_1[10];
	char print_buffer_2[10];
	char print_buffer_3[10];

	/* Initialize system */
	SystemInit();

	/* Initialize delay */
	TM_DELAY_Init();

	/* Initialize USART, TX: PB6 */
	TM_USART_Init(USART1, TM_USART_PinsPack_2, 115200);

	sprintf(str, "test started\r\n");
	TM_USART_Puts(USART1, str);

	/* Initialize MPU9150 sensor */
	if (KB_MPU9150_Init(&MPU9150_Data, KB_MPU9150_Accelerometer_8G, KB_MPU9150_Gyroscope_2000s) != KB_MPU9150_Result_Ok) {
		/* Display error to user */
		TM_USART_Puts(USART1, "MPU9150 Error\n");

		/* Infinite loop */
		while (1);
	}

	while (1) {
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
			print_buffer_1, print_buffer_2, print_buffer_3
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

		/* Little delay */
		Delayms(500);
	}
}
