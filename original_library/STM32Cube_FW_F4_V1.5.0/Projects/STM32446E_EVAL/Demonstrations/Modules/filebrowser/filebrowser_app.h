/**
  ******************************************************************************
  * @file    filebrowser_app.h
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    13-March-2015
  * @brief   Header for filebrowser_app.c file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
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
#ifndef __FILEBROWSER_APP_H
#define __FILEBROWSER_APP_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "k_storage.h"
   
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void     FILEMGR_GetParentDir (char *dir);
void     FILEMGR_GetFileOnly (char *file, char *path);
uint8_t  FILEMGR_ParseDisks (char *path, FILELIST_FileTypeDef *list);
#ifdef __cplusplus
}
#endif

#endif /* __FILE_BROWSER_APP_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
