/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
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

#include "quadcopter_config.h"
#include "stm32f4xx.h"
#include "kb_stm32_hal_motor.h"

/* Private function prototypes -----------------------------------------------*/

static void update_Speed(int speed);

char buffer_input[30];
int	input_speed;

int main(void)
{
	/* Init system */
	quadcopter_Init();

  /* Update the speed of motor */
  update_Speed(1500);

  /* Then start the pwm signal */
  KB_STM32_Motor_Start();
  
  /* Display on USART */
  printf("Motor Started Wait...\n");
  /* Initial speed */
  input_speed = 1500;

  /* Infinite loop */
  while (1)
  {
	  /* update the speed */
	  if(scanf("%d", &input_speed) != 1)
	  {
		  printf("wrong input!\n");
		  /* consume the rest of stdin stream */
		  scanf("%s",buffer_input);
		  continue;
	  }
	  LED_TOGGLE();
	  printf("%d\n", input_speed);
	  update_Speed(input_speed);
	  KB_STM32_Motor_Start();
  }
}



static void update_Speed(int speed)
{
  KB_STM32_Motor_SetSpeed(speed, KB_STM32_Motor_Channel_1);
  KB_STM32_Motor_SetSpeed(speed, KB_STM32_Motor_Channel_2);
  KB_STM32_Motor_SetSpeed(speed, KB_STM32_Motor_Channel_3);
  KB_STM32_Motor_SetSpeed(speed, KB_STM32_Motor_Channel_4);
}



/**
  * @}
  */

/**
  * @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
