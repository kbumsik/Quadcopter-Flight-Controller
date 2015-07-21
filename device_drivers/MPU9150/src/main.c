/**
 *	Keil project example for MPU9150 6-axes
 *
 *	Before you start, select your target, on the right of the "Load" button
 *
 *	@author		Tilen Majerle
 *	@email		tilen@majerle.eu
 *	@website	http://stm32f4-discovery.com
 *	@ide		Keil uVision 5
 *	@packs		STM32F4xx Keil packs version 2.2.0 or greater required
 *	@stdperiph	STM32F4xx Standard peripheral drivers version 1.4.0 or greater required
 */
/* Include core modules */
#include "stm32f4xx.h"
/* Include my libraries here */
#include "defines.h"
#include "tm_stm32f4_delay.h"
#include "tm_stm32f4_usart.h"
#include "kb_stm32f4_mpu9150.h"

#include <stdio.h>

int main(void) {
	KB_MPU9150_t MPU9150_Data;
	char str[120];

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
		sprintf(str, "Accelerometer\n- X:%d\n- Y:%d\n- Z:%d\n",
			MPU9150_Data.Accelerometer_X,
			MPU9150_Data.Accelerometer_Y,
			MPU9150_Data.Accelerometer_Z
		);

		/* Show to usart */
		TM_USART_Puts(USART1, str);

		/* Format data */
		sprintf(str, " Gyroscope\n- X:%d\n- Y:%d\n- Z:%d",
			MPU9150_Data.Gyroscope_X,
			MPU9150_Data.Gyroscope_Y,
			MPU9150_Data.Gyroscope_Z
		);

		/* Show to usart */
		TM_USART_Puts(USART1, str);

		/* Format data */
		sprintf(str, " Temperature\n- %3.2f",
			MPU9150_Data.Temperature
		);

		/* Show to usart */
		TM_USART_Puts(USART1, str);

		/* Format data */
		sprintf(str, " Magnetometer\n- X:%d\n- Y:%d\n- Z:%d\r\n",
			MPU9150_Data.Magnetometer_X,
			MPU9150_Data.Magnetometer_Y,
			MPU9150_Data.Magnetometer_Z
		);

		/* Show to usart */
		TM_USART_Puts(USART1, str);

		/* Little delay */
		Delayms(500);
	}
}
