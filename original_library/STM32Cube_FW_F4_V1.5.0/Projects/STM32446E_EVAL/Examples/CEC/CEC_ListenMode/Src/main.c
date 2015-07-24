/**
  ******************************************************************************
  * @file    CEC/CEC_ListenMode/Src/main.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    13-March-2015
  * @brief   This example describes how to configure and use the CEC through 
  *          the STM32F4xx HAL API.
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

/** @addtogroup CEC_ListenMode
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t  Tab_Rx[CEC_MAX_PAYLOAD];   /* Received data buffer. Max size = 16 bytes
                                     * header + opcode followed by up to 14 operands */
uint8_t  Tab_Tx[CEC_MAX_PAYLOAD-1]; /* Transmitted data buffer. 
                                     * header is not included in Tab_Tx.
                                     *  Max size = 15 bytes.                                   
                                     *  one opcode followed by up to 14 operands.
                                     *  When payload size = 0, only the header is sent
                                     *  (ping operation) */                                   
uint8_t ReceivedFrame       = 0x0;  /* Set when a reception occurs */
uint16_t NbOfReceivedBytes  = 0x0;  /* Number of received bytes in addition to the header. 
                                     * when a ping message has been received (header 
                                     * only), NbOfReceivedBytes = 0 */
uint8_t StartSending        = 0x0;  /* Set when a transmission is triggered by the user */
uint32_t TxSize             = 0x0;  /* Number of bytes to transmit in addition to the header. 
                                     * In case of ping operation (only the header sent),
                                     * TxSize = 0 */
uint8_t DestinationAddress;         /* Destination logical address */                            
uint8_t LogicalAddress;             /* CEC IP Initiator logical address */              
CEC_HandleTypeDef hcec;             /* CEC IP handle */ 
/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
#if 0
static void Error_Handler(void);
#endif
static void CEC_Config(CEC_HandleTypeDef *hcec);
static void CEC_FlushRxBuffer(void);
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
 /* This sample code shows how to use STM32F4xx CEC HAL API to transmit and 
  * receive data. The device is set in waiting to receive mode and sends
  * messages when the evaluation board buttons are pushed by the user */
  
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

  /* -1- Initialize LEDs mounted on EVAL board */
  /* Configure LED1, LED2, LED3 and LED4 */
  BSP_LED_Init(LED1);
  BSP_LED_Init(LED2);
  BSP_LED_Init(LED3);
  BSP_LED_Init(LED4);

  /* -2- Configure User push-button in Interrupt mode */
  /* button-triggered interruptions initialization */
  BSP_PB_Init(BUTTON_TAMPER,BUTTON_MODE_EXTI);
  
  /* -3- Configure Joystick Selection push-button in Interrupt mode */
  BSP_JOY_Init(JOY_MODE_EXTI);

  
  /* CEC device initialization */
  
#if defined (DEVICE_1)
  LogicalAddress = DEVICE_ADDRESS_1;
  DestinationAddress = DEVICE_ADDRESS_2; /* follower address */
#elif defined (DEVICE_2)
  LogicalAddress = DEVICE_ADDRESS_2;
  DestinationAddress = DEVICE_ADDRESS_1; /* follower address */
#elif defined (DEVICE_3)
  LogicalAddress = DEVICE_ADDRESS_3;
  DestinationAddress = DEVICE_ADDRESS_1; /* follower address */
#endif  
    
  /* -4- CEC configuration (transfer will take place in Interrupt mode) */
  hcec.Instance = CEC;
  /* Deinitialize CEC to reinitialize from scratch */
  HAL_CEC_DeInit(&hcec);
  /* IP configuration */ 
  CEC_Config(&hcec);
  

  /* -5- CEC transfer general variables initialization */
  ReceivedFrame = 0;
  StartSending = 0;
  NbOfReceivedBytes = 0;
  CEC_FlushRxBuffer();
  
  
  /* Test start */
    
  /* Enter infinite reception loop: the CEC device is set in
   * waiting to receive mode. 
   * The CEC "background" state is HAL_CEC_STATE_STANDBY_RX.
   * Upon any message reception or transmission, the CEC 
   * comes back to that state.
   * It is up to the user to define exit conditions in modifying
   * accordingly the RX, TX or Error callback functions. */
  HAL_CEC_Receive_IT(&hcec, (uint8_t *)&Tab_Rx); 
  while (HAL_CEC_GetState(&hcec) != HAL_CEC_STATE_READY)
  {
    
    /* if no reception has occurred and no error has been detected,
     * transmit a message if the user has pushed a button */
    if( (StartSending == 1) && (ReceivedFrame == 0))
    { 
      HAL_CEC_Transmit_IT(&hcec, DestinationAddress, (uint8_t *)&Tab_Tx, TxSize);
      /* loop until TX ends or TX error reported */
        while (HAL_CEC_GetState(&hcec) != HAL_CEC_STATE_STANDBY_RX);
      StartSending = 0;
    }  
    
    /* if a frame has been received */
    if (ReceivedFrame == 1)
    { 
      if (Tab_Rx[1] == 0x44) /* Test on the opcode value */
      {
        /* Receive command is equal to Volume Up(Button Up) */
        if (Tab_Rx[2] == 0x41) /* Test on the operand value */
        {       
          BSP_LED_On(LED1);
          BSP_LED_On(LED2);
          BSP_LED_On(LED3);
          BSP_LED_On(LED4);
        }
        else if (Tab_Rx[2] == 0x42) /* Receive command is equal to Volume Down(Button Down) */
        {
          BSP_LED_Off(LED1);
          BSP_LED_Off(LED2);
          BSP_LED_Off(LED3);
          BSP_LED_Off(LED4);
        }
      }
      else if (Tab_Rx[1] == 0x46) /* Test on the opcode value */
      {
        BSP_LED_On(LED1);
        BSP_LED_On(LED2);
        BSP_LED_Off(LED3);
        BSP_LED_Off(LED4);
      }
      else if (Tab_Rx[1] == 0x9F) /* Test on the opcode value */
      {
        BSP_LED_Off(LED1);
        BSP_LED_Off(LED2);
        BSP_LED_On(LED3);
        BSP_LED_On(LED4);
      }
      ReceivedFrame = 0;
    }
    else if (ReceivedFrame == 2) /* means CEC error detected */
    { 
      /* Turn on LED3 */
      BSP_LED_On(LED3);
      ReceivedFrame = 0;
    }
  }  /*  while (HAL_CEC_GetState(&hcec) != HAL_CEC_STATE_READY) */
  return 0;
}

/**
  * @brief  Configures the CEC peripheral.
  * @param  None
  * @retval None
  */
static void CEC_Config(CEC_HandleTypeDef *hcec)
{
  /* CEC configuration parameters */    
  
  hcec->Init.InitiatorAddress          = LogicalAddress;

  hcec->Init.SignalFreeTime            = CEC_DEFAULT_SFT;
  hcec->Init.Tolerance                 = CEC_STANDARD_TOLERANCE;
  hcec->Init.BRERxStop                 = CEC_NO_RX_STOP_ON_BRE;
  hcec->Init.BREErrorBitGen            = CEC_BRE_ERRORBIT_NO_GENERATION;
  hcec->Init.LBPEErrorBitGen           = CEC_LBPE_ERRORBIT_NO_GENERATION;
  hcec->Init.BroadcastMsgNoErrorBitGen = CEC_BROADCASTERROR_NO_ERRORBIT_GENERATION;
  hcec->Init.SignalFreeTimeOption      = CEC_SFT_START_ON_TXSOM;
  hcec->Init.OwnAddress                = (0x1 <<  hcec->Init.InitiatorAddress); 
  
#if defined (DEVICE_1)
  hcec->Init.ListenMode                =  CEC_REDUCED_LISTENING_MODE;
#elif defined (DEVICE_2)
  hcec->Init.ListenMode                =  CEC_REDUCED_LISTENING_MODE;
#elif defined (DEVICE_3)
  hcec->Init.ListenMode                =  CEC_FULL_LISTENING_MODE;
#endif   

  
  HAL_CEC_Init(hcec);
}

/**
  * @brief Tx Transfer completed callback
  * @param hcec: CEC handle
  * @retval None
  */
void HAL_CEC_TxCpltCallback(CEC_HandleTypeDef *hcec)
{
  /* after transmission, return to stand-by mode */
  hcec->State = HAL_CEC_STATE_STANDBY_RX;
}

/**
  * @brief Rx Transfer completed callback
  * @param hcec: CEC handle
  * @retval None
  */
void HAL_CEC_RxCpltCallback(CEC_HandleTypeDef *hcec)
{
    ReceivedFrame = 1;
    /* Reminder: hcec->RxXferSize is the sum of opcodes + operands 
     * (0 to 14 operands max).
     * If only a header is received, hcec->RxXferSize = 0 */   
    NbOfReceivedBytes = hcec->RxXferSize;
    hcec->RxXferSize = 0;
    hcec->pRxBuffPtr = Tab_Rx;
    hcec->ErrorCode = HAL_CEC_ERROR_NONE;
    /* return to stand-by mode */
    hcec->State = HAL_CEC_STATE_STANDBY_RX;
}

/**
  * @brief CEC error callbacks
  * @param hcec: CEC handle
  * @retval None
  */
void HAL_CEC_ErrorCallback(CEC_HandleTypeDef *hcec)
{
  ReceivedFrame = 2;

  hcec->RxXferSize = 0;
  hcec->pRxBuffPtr = Tab_Rx;
  hcec->ErrorCode = HAL_CEC_ERROR_NONE;
  hcec->State = HAL_CEC_STATE_STANDBY_RX;
}

/**
  * @brief  Reset CEC reception buffer
  * @param  None
  * @retval None
  */
static void CEC_FlushRxBuffer(void)
{
  uint32_t cpt;
  
  for (cpt = CEC_MAX_PAYLOAD; cpt > 0; cpt--)
  {
    Tab_Rx[cpt-1] = 0;
  }
}

/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  uint32_t ITstatus = 0;
  JOYState_TypeDef JoyState = JOY_NONE;

  if(GPIO_Pin == TAMPER_BUTTON_PIN)
  {
    /* Toggle GREEN LED1 */
    BSP_LED_Toggle(LED1);

    Tab_Tx[0] = 0x44;
    Tab_Tx[1] = 0x42;
    TxSize    = 0x02;
    StartSending = 1;
  }

  if(GPIO_Pin == MFX_IRQOUT_PIN)  /* Interrupt received from MFX */
  {
    /* The different functionalities of MFX (TS, Joystick, SD detection, etc. )  
    can be configured in exti mode to generate an IRQ on given events.
    The MFX IRQ_OUT pin is unique and common to all functionalities, so if several 
    functionalities are configured in exit mode, the MCU has to enquire MFX about  
    the IRQ source (see BSP_IO_ITGetStatus). Communication with Mfx is done by I2C. 
    Often the sw requires ISRs (irq service routines) to be quick while communication 
    with I2C can be considered relatively long (hundreds of usec depending on I2C clk). 
    Considering that the features for human interaction like TS, Joystick, SD detection 
    don�t need immediate reaction, it is suggested to use POLLING instead of EXTI mode, 
    in order to avoid "blocking I2C communication" on interrupt service routines */
	
    ITstatus = BSP_IO_ITGetStatus(JOY_ALL_PINS);
    if (ITstatus)                 /* Checks if interrupt comes from joystick */
    {
      /* Get the Joystick State */
      JoyState = BSP_JOY_GetState();

      if(JoyState == JOY_DOWN)
      {
        /* Toggle BLUE LED4 */
        BSP_LED_Toggle(LED4);
    
        Tab_Tx[0] = 0x9F;
        TxSize    = 0x01;
        StartSending = 1;
      }

      if(JoyState == JOY_SEL)
      {
        /* Toggle ORANGE LED2 */
        BSP_LED_Toggle(LED2);
    
        Tab_Tx[0] = 0x46;
        TxSize    = 0x01;
        StartSending = 1;
      }
  
      if(JoyState == JOY_UP)
      {
        /* Toggle RED LED3 */
        BSP_LED_Toggle(LED3);
    
        Tab_Tx[0] = 0x44;
        Tab_Tx[1] = 0x41;
        TxSize    = 0x02;
        StartSending = 1;    
      }

    }
    BSP_IO_ITClear();
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
  *            PLL_R                          = 2
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
#if 0
static void Error_Handler(void)
{
  /* Turn LED3 on */
  BSP_LED_On(LED3);

  while (1)
  {
  }
}
#endif

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
