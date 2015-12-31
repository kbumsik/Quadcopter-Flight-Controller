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
#include "config.h"
#include <stdio.h>
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Global variables ----------------------------------------------------------*/
TIM_HandleTypeDef xTIM1Handle;          /* Located in PWMInput.h */
TIM_HandleTypeDef xTIM2Handle;          /* Located in PWMInput.h */
TIM_HandleTypeDef xTIM3Handle;          /* Located in PWMInput.h */
TIM_HandleTypeDef xTIM5Handle;          /* Located in PWMInput.h */

/* Private variables ---------------------------------------------------------*/
xTaskHandle xBlinkyHandle;
xTaskHandle xScanInputHandle;
xTaskHandle xRemoteReceiverHandle;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void vBlinkyTask(void *pvParameters);
void vScanInputTask(void *pvParameters);
void vRemoteReceiverTask(void *pvParameters);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals,Initializes the Flash interface and the Systick.*/
  quadcopter_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */

  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  /* TODO: change it to the FreeRTOS functions */
  xTaskCreate(vBlinkyTask,			        /* Pointer to the function that implements the task */
		  	  "Blinky",						/* Text name for the task. This is to facilitate debugging only. It is not used in the scheduler */
		  	  configMINIMAL_STACK_SIZE,		/* Stack depth in words */
		  	  NULL,							/* Pointer to a task parameters */
		  	  configMAX_PRIORITIES-2,		/* The task priority */
		  	  &xBlinkyHandle);                        /* Pointer of its task handler, if you don't want to use, you can leave it NULL */
  /*
  xTaskCreate(vScanInputTask,
              "Scan",
              configMINIMAL_STACK_SIZE+200,
              NULL,
              configMAX_PRIORITIES-2,
              &xScanInputHandle);
              */
  xTaskCreate(vRemoteReceiverTask,
                "RemoteReceiver",
                configMINIMAL_STACK_SIZE+500,
                NULL,
                configMAX_PRIORITIES-1,
                &xRemoteReceiverHandle);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  vTaskStartScheduler();
  /* NOTE: We should never get here as control is now taken by the scheduler */
  while (1)
    {
    }
}

/* vBlinkyTask function */
void vBlinkyTask(void *pvParameters)
{
  uint32_t ulCount = 0;

  portTickType xLastWakeTime;
  /* Initialize xLastWakeTime for vTaskDelayUntil */
  /* This variable is updated every vTaskDelayUntil is called */
  xLastWakeTime = xTaskGetTickCount();
  for(;;)
    {
      vLED_0_Toggle();
      /* Call this Task explicitly every 50ms ,NOT Delay for 50ms */
      vTaskDelayUntil(&xLastWakeTime, (50/portTICK_RATE_MS));
    }

  /* It never goes here, but the task should be deleted when it reached here */
  vTaskDelete(NULL);
}

/* vScanInputTask Task function */
void vScanInputTask(void *pvParameters)
{
  unsigned portBASE_TYPE uxPriority;
  uint32_t  ulCount = 1;
  int32_t   lCheck = 0;
  int       lInput = 0;
  char pcStr[30];

  /* Get the priority of this task */
  uxPriority = uxTaskPriorityGet(NULL);

  /* Infinite loop */
  for(;;)
    {
      printf("ready\r\n");
      /* Set Blinky task less than this task, meaning stop blinking */
      //vTaskPrioritySet(xBlinkyHandle, uxPriority-1);

      lCheck = scanf("%d",&lInput);
      if(lCheck!=1)
        {
          printf("Wrong Input!: ");
          // flush the buffer
          scanf("%s",pcStr);
          printf("%s\r\n",pcStr);
        }
      else
        {
          printf("%d\r\n",lInput);
          /* Print the Idle count */
          printf("count=%d\r\n",ulCount++);
        }

      /* Set Blinky back */
      //vTaskPrioritySet(xBlinkyHandle, uxPriority);

      /* Set the task into blocking mode for 1000ms */
      /* The task becomes ready state after 1000ms */
      vTaskDelay(1000 / portTICK_RATE_MS);
    }

  /* It never goes here, but the task should be deleted when it reached here */
  vTaskDelete(NULL);
}

/* Remote Receiver Task */
void vRemoteReceiverTask(void *pvParameters)
{
  /* Infinite loop */
  while (1)
  {
    printf("Raw data of the duty cycle: ");
    printf("%d ", ulPWMInputDutyCycle(&xTIM1Handle));
    printf("%d ", ulPWMInputDutyCycle(&xTIM2Handle));
    printf("%d ", ulPWMInputDutyCycle(&xTIM3Handle));
    printf("%d\r\n", ulPWMInputDutyCycle(&xTIM5Handle));
    printf("Raw data of the period :");
    printf("%d ", ulPWMInputPeriod(&xTIM1Handle));
    printf("%d ", ulPWMInputPeriod(&xTIM2Handle));
    printf("%d ", ulPWMInputPeriod(&xTIM3Handle));
    printf("%d\r\n", ulPWMInputPeriod(&xTIM5Handle));
    printf("----------------------\r\n");
    vTaskDelay(2000);
  }
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
  printf("Wrong parameters value: file %s on line %d\r\n", file, line);
  /* USER CODE END 6 */
}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
