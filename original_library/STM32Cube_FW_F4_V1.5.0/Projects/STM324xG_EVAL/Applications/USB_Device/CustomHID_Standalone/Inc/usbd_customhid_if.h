/**
  ******************************************************************************
  * @file    USB_Device/CustomHID_Standalone/Inc/usbd_customhid_if.h
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    13-March-2015
  * @brief   Header for usbd_customhid_if.c file.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USBD_CUSTOMHID_IF_H
#define __USBD_CUSTOMHID_IF_H

/* Includes ------------------------------------------------------------------*/
#include "usbd_customhid.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define LED1_REPORT_ID           0x01
#define LED1_REPORT_COUNT        0x01

#define LED2_REPORT_ID           0x02
#define LED2_REPORT_COUNT        0x01

#define LED3_REPORT_ID           0x03
#define LED3_REPORT_COUNT        0x01

#define LED4_REPORT_ID           0x04
#define LED4_REPORT_COUNT        0x01

#define KEY_REPORT_ID            0x05
#define TAMPER_REPORT_ID         0x06
#define ADC_REPORT_ID            0x07

/* User can use this section to tailor ADCx instance used and associated 
   resources */
/* Definition for ADCx clock resources */
#define ADCx                            ADC3
#define ADCx_CLK_ENABLE()               __HAL_RCC_ADC3_CLK_ENABLE()
#define DMAx_CLK_ENABLE()               __HAL_RCC_DMA2_CLK_ENABLE()     
#define ADCx_CHANNEL_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOF_CLK_ENABLE()
     
#define ADCx_FORCE_RESET()              __HAL_RCC_ADC_FORCE_RESET()
#define ADCx_RELEASE_RESET()            __HAL_RCC_ADC_RELEASE_RESET()

/* Definition for ADCx Channel Pin */
#define ADCx_CHANNEL_PIN                GPIO_PIN_9
#define ADCx_CHANNEL_GPIO_PORT          GPIOF 

/* Definition for ADCx's Channel */
#define ADCx_CHANNEL                    ADC_CHANNEL_7

/* Definition for ADCx's DMA */
#define ADCx_DMA_CHANNEL                DMA_CHANNEL_2
#define ADCx_DMA_STREAM                 DMA2_Stream0         

/* Definition for ADCx's NVIC */
#define ADCx_DMA_IRQn                   DMA2_Stream0_IRQn
#define ADCx_DMA_IRQHandler             DMA2_Stream0_IRQHandler

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
extern USBD_CUSTOM_HID_ItfTypeDef USBD_CustomHID_fops;

#endif /* __USBD_CUSTOMHID_IF_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
