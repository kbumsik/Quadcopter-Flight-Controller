/**
  ******************************************************************************
  * @file    k_mem.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    13-March-2015   
  * @brief   Utilities for file handling
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
#include "k_mem.h"

/** @addtogroup CORE
  * @{
  */

/** @defgroup KERNEL_MEMORY
  * @brief Kernel memory routines
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define MEM_BASE                     0xC0000000
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
#if (defined ( __CC_ARM ))
mem_TypeDef memory_pool __attribute__((at(MEM_BASE)));
#elif (defined (__ICCARM__))
#pragma location = MEM_BASE
__no_init mem_TypeDef memory_pool;
#elif defined ( __GNUC__ )
mem_TypeDef memory_pool __attribute__((section(".ExtRAMData")));
#endif


/* Private functions ---------------------------------------------------------*/
/**
  * @brief  This function Initialize a memory pool for private allocator.
  * @param  None
  * @retval None
  */
void k_MemInit(void)
{
  memset(&memory_pool, 0, sizeof(mem_TypeDef));
  memory_pool.mallocBase = MEM_BASE + sizeof(mem_TypeDef);
}

/**
  * @brief  This function implement a simple memory management algorithm (First fit).
  * @param  size : Requested memory size.
  * @retval Pointer to the allocated region.
  */
void * k_malloc(size_t size)
{
  __IO uint8_t index = 0, counter = 0, start = 0;
  uint8_t PageCount = 0, NewStart = 0;
  if (size > 0)
  {
    /* look for the first contiguous memory that meet requested size. */
    while ( index < (MAX_PAGE_NUMBER))
    {
      if (memory_pool.PageTable[index++] == 0)
      {
        counter++;
        if (size <= (counter * SIZE_OF_PAGE))
        {
          PageCount = counter - 1;
          NewStart = start;
          /* Set memory region state to allocated */
          for (index = 0; index <= PageCount;index++)
          {
            memory_pool.PageTable[NewStart + index] = 1;
          }
          /* Save size */
          memory_pool.size[NewStart] = counter;
          /* return pointer to allocated region. */
          return (void *)((memory_pool.mallocBase + (start  << 10)) + 1 * sizeof(uint32_t));
        }
      }
      else
      {
        start = index;
        counter = 0;
      }
    }
  }
  return NULL;
}

/**
  * @brief  Free previously allocated memory
  * @param  Pointer to the allocated region.
  * @retval None
  */
void k_free(void * p)
{
  __IO uint8_t index = 0;
  uint8_t counter = 0, start = 0;
  /* Handle NULL pointers */
  if (p == NULL)
    return;

  /* Find start Index */
  start = ((((uint32_t)p - sizeof(uint32_t)) - memory_pool.mallocBase) >> 10);
  /* Retrieve segment size */
  counter = memory_pool.size[start];
  /* Set size to zero */
  memory_pool.size[start] = 0;
  /* Deallocate memory region */
  for (index = 0; index < counter; index++)
  {
    memory_pool.PageTable[start + index] = 0;
  }

  return;
}

/**
  * @}
  */

/**
  * @}
  */
  
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
