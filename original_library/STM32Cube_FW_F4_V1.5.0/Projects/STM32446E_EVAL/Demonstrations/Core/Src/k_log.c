/**
  ******************************************************************************
  * @file    k_log.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    13-March-2015   
  * @brief   This file provides the kernel log functions    
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
#include "k_log.h"
#include "k_module.h"  
#include "k_mem.h" 
#include  <stdio.h>

/** @addtogroup CORE
  * @{
  */

/** @defgroup KERNEL_LOG
  * @brief Kernel Log routines
  * @{
  */

/* External variables --------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
#define LOG_DEPTH    (4 * 512)
/* Private macros ------------------------------------------------------------*/
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
/* Private variables ---------------------------------------------------------*/

uint8_t  *pLOG_CacheBuffer;  
uint32_t LOG_IN_ptr = 0;

/**
  * @brief  Init Kernel Log
  * @param  None
  * @retval None
  */
void k_LogInit(void)
{
  pLOG_CacheBuffer = (uint8_t *)k_malloc(LOG_DEPTH);
  memset (pLOG_CacheBuffer, 0, LOG_DEPTH);
  LOG_IN_ptr = 0; 
}

/**
  * @brief  Redirect printf
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
   uint32_t cnt = 0;
   
   if( LOG_IN_ptr++ >= LOG_DEPTH)
   {
     
    for (cnt = 1; cnt <  LOG_DEPTH; cnt ++)
    {
      pLOG_CacheBuffer[cnt -1 ] = pLOG_CacheBuffer[cnt];
    }
    LOG_IN_ptr = LOG_DEPTH;
   }
   pLOG_CacheBuffer [LOG_IN_ptr - 1] = ch;
   
   if(ch == '\n')
   {
     k_UpdateLog ((char *)pLOG_CacheBuffer);
   }
   
   return ch;
}
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
