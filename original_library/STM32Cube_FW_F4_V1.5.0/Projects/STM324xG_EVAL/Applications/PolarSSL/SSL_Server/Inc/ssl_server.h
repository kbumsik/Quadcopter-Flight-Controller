/**
  ******************************************************************************
  * @file    PolarSSL/SSL_Server/Inc/ssl_server.h 
  * @author  MCD Application Team
  * @version V1.2.1
  * @date    13-March-2015
  * @brief   This file contains all the functions prototypes for the ssl_server.c 
  *          file.
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
#ifndef __SSL_SERVER_H
#define __SSL_SERVER_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "polarssl/ssl.h"

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void ssl_server(void const * argument);
void ssl_DynPage(ssl_context *ssl);
void ssl_sendframes(ssl_context *ssl, char *data, int datalen);
int RandVal(void* arg, unsigned char *data, size_t size);

#ifdef __cplusplus
}
#endif

#endif /* __SSL_SERVER_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
