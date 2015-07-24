/**
  ******************************************************************************
  * @file    USB_Host/MTP_Standalone/Src/mtp.c 
  * @author  MCD Application Team
  * @version V1.2.1
  * @date    13-March-2015
  * @brief   This file provides APIs to explore MTP Storage Objects
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
MTP_ObjectHandlesTypedef WavHandles;
uint32_t NumObs = 0;

/* Private function prototypes -----------------------------------------------*/
static uint8_t MTP_GetWavObjectHandles(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  AUDIO_Start 
  *         Start Audio streaming
  * @param  None
  * @retval None
  */
uint8_t MTP_Init(void)
{
  static uint8_t is_initialized = 0;
  uint8_t ret = 1;
  
  if(is_initialized == 0)
  {
    if(USBH_MTP_IsReady(&hUSBHost) > 0)
    { 
      if(USBH_MTP_GetNumObjects(&hUSBHost, 0, PTP_OFC_WAV, PTP_AT_Undefined, &NumObs) == USBH_OK)
      {
        /* Get objects handlers */
        if(MTP_GetWavObjectHandles() == 0)
        {
          is_initialized = 1;
          ret = 0;   
        }
      }
    }
  }
  else
  {
   ret = 0;  
  }
  return ret;
}

/**
  * @brief  Explores Wav Files. 
  * @param  None
  * @retval Returns 0 if OK, otherwise 1.
  */
uint8_t MTP_ExploreWavFile(void)
{
  uint8_t ret = 1;
  uint32_t index;
  MTP_ObjectInfoTypedef objectinfo;
  
  MTP_Init();
  
  if(USBH_MTP_IsReady(&hUSBHost) > 0)
  {
    LCD_UsrLog("\nAvailable wav files:\n");
 
    /* Get Available WAV files number */
    if((NumObs = MTP_GetWavObjectNumber()) > 0)
    {
      /* Get objects handlers */
      if(MTP_GetWavObjectHandles() == 0)
      {
        ret = 0; 
        
        for (index = 0; index < NumObs; index ++)
        {
          if( USBH_MTP_GetObjectInfo (&hUSBHost, 
                                      WavHandles.Handler[index], 
                                      &objectinfo) == USBH_OK)
            
          {
            LCD_DbgLog(" %lu- %s\n", index, objectinfo.Filename);
          }
          else
          {
            ret = 1; 
          }
        }
      }
    }
  }
  else
  {
    LCD_ErrLog("MTP Device Not yet ready...\n");
  }
  
  return ret;
}

/**
  * @brief  Gets Data from MTP Device.
  * @param  file_idx: File index
  * @param  offset: Offset
  * @param  maxbytes: Max bytes
  * @param  object: Pointer to the file object
  * @param  len: Pointer to the file length        
  * @retval Returns Status 0 if OK, otherwise 1.
  */
uint8_t MTP_GetData(uint32_t file_idx, uint32_t offset, uint32_t maxbytes, uint8_t *object, uint32_t *len)
{ 
  USBH_MTP_GetPartialObject(&hUSBHost, 
                            WavHandles.Handler[file_idx], 
                            offset,
                            maxbytes, 
                            object,
                            len);    
  return 0;
}

/**
  * @brief  Gets Wav Object Number.
  * @param  None
  * @retval Number of Wav object files
  */
uint16_t MTP_GetWavObjectNumber(void)
{
  return NumObs;
}

/**
  * @brief  Gets Wav Object Names.
  * @param  object_index: Object index
  * @param  filename: Pointer to the file name  
  * @retval Returns Status 0 if OK, otherwise 1.
  */
uint8_t MTP_GetWavObjectName(uint16_t object_index, uint8_t *filename)
{
  uint8_t ret = 1;
  MTP_ObjectInfoTypedef objectinfo;
  
  if(USBH_MTP_GetObjectInfo(&hUSBHost, WavHandles.Handler[object_index], &objectinfo) == USBH_OK)
  {
    strcpy((char *)filename, (char *)objectinfo.Filename);
    ret = 0;
  }
  
  return ret;
}

/**
  * @brief  Gets Wav Object Handles.
  * @param  None
  * @retval Returns Status 0 if OK, otherwise 1.
  */
static uint8_t MTP_GetWavObjectHandles(void)
{ 
  /* Get objects handlers */
  if(USBH_MTP_GetObjectHandles(&hUSBHost, 0, PTP_OFC_WAV, PTP_AT_Undefined, &WavHandles) == USBH_OK)
  {
    return 0;
  }
  return 1;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
