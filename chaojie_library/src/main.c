/**
 *	Keil project example for MPU6050 6-axes
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
#include "cj_state.h"
#include "cj_helper.h"

#include <stdio.h>
#include <math.h>


int main(void) {

    /* Initialize system */
    SystemInit();

    /* Initialize delay */
    TM_DELAY_Init();

    /* Initialize USART, TX: PB6 */
    TM_USART_Init(USART1, TM_USART_PinsPack_2, 115200);

    if (!cj_state_init()) {
	printf("state init failed\r\n");
	while (1) {}
    }
    
    /* Printing buffers */
    char print_buffer_1[10];
    char print_buffer_2[10];
    char print_buffer_3[10];
    char str[120];
    /*state*/
    struct Cj_helper_float3 anlges;
    struct Cj_helper_float3 anlges_v;
    while (1) {
	cj_state_update();

	cj_state_get_angles(&angles);
	cj_state_get_angular_velocity(&angles_v);
	
	/* format data */
	conv_FloatToString(angles.a, print_buffer_1);
	conv_FloatToString(angles.b, print_buffer_2);
	conv_FloatToString(angles.c, print_buffer_3);

	//print the data
	sprintf(str, "angle:(%s, %s, %s); ",
		print_buffer_1, print_buffer_2, print_buffer_3);

	/* Show to usart */
	TM_USART_Puts(USART1, str);

	/* format data */
	conv_FloatToString(angles_v.a, print_buffer_1);
	conv_FloatToString(angles_v.b, print_buffer_2);
	conv_FloatToString(angles_v.c, print_buffer_3);
	
	//print the data
	sprintf(str, "the angular velocity: (%s, %s, %s)\r\n",
		print_buffer_1, print_buffer_2, print_buffer_3);

	/* Show to usart */
	TM_USART_Puts(USART1, str);

	/* Little delay */
	Delayms(10);
    }
}
