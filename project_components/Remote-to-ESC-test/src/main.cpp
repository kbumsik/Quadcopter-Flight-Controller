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
#include "main.h"
#include "board_init.h"

/* Global Variables ----------------------------------------------------------*/
TIM_HandleTypeDef xMotorHandle; /* located in Motor.h */

/* Private variables ---------------------------------------------------------*/
/* Task Handlers */
TaskHandle_t xBlinkyHandle;
TaskHandle_t xScanInputHandle;
TaskHandle_t xRemoteScanHandle;

/* Queue Handlers */
QueueHandle_t quUARTReceive;

/* Mutex Handlers */
SemaphoreHandle_t muRemote;

/* Global function prototypes ------------------------------------------------*/
void Error_Handler(void);

/* Private function prototypes -----------------------------------------------*/
void vBlinkyTask(void *pvParameters);
void vScanInputTask(void *pvParameters);
void vRemoteScanTask(void *pvParameters);

int main(void)
{
  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface
   * and the Systick. */
  quadcopter_Init();

  /* USER CODE BEGIN RTOS_MUTEX */
  muRemote = xSemaphoreCreateMutex();
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  xTaskCreate(vBlinkyTask,			        /* Pointer to the function that implements the task */
		  	  "Blinky",						/* Text name for the task. This is to facilitate debugging only. It is not used in the scheduler */
		  	  configMINIMAL_STACK_SIZE,		/* Stack depth in words */
		  	  NULL,							/* Pointer to a task parameters */
		  	  1,		                    /* The task priority */
		  	  &xBlinkyHandle);                        /* Pointer of its task handler, if you don't want to use, you can leave it NULL */
  xTaskCreate(vScanInputTask,
              "Scan",
              configMINIMAL_STACK_SIZE+2000,
              NULL,
              configMAX_PRIORITIES-3,
              &xScanInputHandle);
  xTaskCreate(vRemoteScanTask,
              "Remote",
              configMINIMAL_STACK_SIZE+1000,
              NULL,
              configMAX_PRIORITIES-1,
              &xRemoteScanHandle);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* definition and creation of xQueueUARTReceive */
  quUARTReceive = xQueueCreate(confUART_RECEIVE_QUEUE_LENGTH, /* length of queue */
                              sizeof(uint8_t)*confUART_RECEIVE_BUFFER_SIZE); /* size in byte of each item */
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
  portTickType xLastWakeTime;
  /* Initialize xLastWakeTime for vTaskDelayUntil */
  /* This variable is updated every vTaskDelayUntil is called */
  xLastWakeTime = xTaskGetTickCount();

  for(;;)
    {
      vLED_0_Toggle();
      /* Call this Task explicitly every 50ms ,NOT Delay for 50ms */
      vTaskDelayUntil(&xLastWakeTime, (200/portTICK_RATE_MS));
    }

  /* It never goes here, but the task should be deleted when it reached here */
  vTaskDelete(NULL);
}

/* vScanInputTask Task function */
void vScanInputTask(void *pvParameters)
{
  int i = 0;
  portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  eMotorChannel_t eMotorChannel = eMOTOR_CHANNEL_1;
  /* Update the speed of motor */
  swMotorSetSpeed(&xMotorHandle, 1500, eMOTOR_CHANNEL_ALL);

  /* Then start the pwm signal */
  eMotorStart(&xMotorHandle, eMOTOR_CHANNEL_ALL);

  /* Infinite loop */
  for (;;)
  {
    /* Start updating the speed of motor */
    /* get remote value */
    xSemaphoreTake(muRemote, portMAX_DELAY); /* Take Mutext */
    for (i = 0; i < confREMOTE_NUMBER_OF_CHANNEL; i++)
    {
      pfMotorSpeed[i] = xControlPlusAndScaleReversed(pfRemote[i],
          motorSPEED_MID, pfRemoteToMotorScale[i]);
    }
    xSemaphoreGive(muRemote); /* Release mutex */

    /* Limit the speed (Don't really needed here though */
    for (i = 0; i < confREMOTE_NUMBER_OF_CHANNEL; i++)
    {
      pfMotorSpeed[i] = xControlLimitter(pfMotorSpeed[i], motorSPEED_MIN,
          motorSPEED_MAX);
    }
    /* Finally update the motor speed */
    eMotorChannel = eMOTOR_CHANNEL_1;
    for (i = 0; i < confREMOTE_NUMBER_OF_CHANNEL; i++)
    {
      swMotorSetSpeed(&xMotorHandle, (int32_t) pfMotorSpeed[i], eMotorChannel);
      eMotorChannel = (eMotorChannel_t)(eMotorChannel + 1);
    }
    eMotorStart(&xMotorHandle, eMOTOR_CHANNEL_ALL); /* Start PWM */
    /* End updating the speed of motor */
    /* Do this every 200ms */
    vTaskDelayUntil(&xLastWakeTime, 200 / portTICK_RATE_MS);
  }

  /* It never goes here, but the task should be deleted when it reached here */
  vTaskDelete(NULL);
}

/* vScanInputTask Task function */
void vRemoteScanTask(void *pvParameters)
{
  portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  int i = 0;
  while (1)
  {

    /* Start Update Remote controller */
    /* get Remote value and scale */
    xSemaphoreTake(muRemote, portMAX_DELAY); /* Take Mutext */
    for (i = 0; i < confREMOTE_NUMBER_OF_CHANNEL; i++)
    {
      pfRemote[i] = xControlLimitter(uwPWMInputDutyCycle(pxRemoteHandle[i]),
          ppuwRemoteRange[i][confREMOTE_MIN], ppuwRemoteRange[i][confREMOTE_MAX]);
    }
    for (i = 0; i < confREMOTE_NUMBER_OF_CHANNEL; i++)
    {
      pfRemote[i] = (float32_t)xControlMinusAndScale(pfRemote[i],
                     ppuwRemoteRange[i][confREMOTE_CENTER], pfRemoteScale[i]);
    }
    /* End Update remote controller */
    xSemaphoreGive(muRemote); /* Release mutex */
    /* Do this every 100ms */
    vTaskDelayUntil(&xLastWakeTime, 100 / portTICK_RATE_MS);
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
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  printf("Wrong parameters value: file %s on line %d\r\n", file, line);
  while(1)
  {

  }
}

#endif

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  while(1)
  {
  }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
