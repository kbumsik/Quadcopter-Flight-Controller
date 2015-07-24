/**
  ******************************************************************************
  * @file    USB_Host/FWupgrade_Standalone/Src/flash_if.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    13-March-2015
  * @brief   This file provides all the flash layer functions.
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

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint32_t FirstSector = 0;
uint32_t NbOfSectors = 0;
uint32_t SectorError = 0;
uint32_t OB_RDP_LEVEL;

/* Private function prototypes -----------------------------------------------*/
static uint32_t FLASH_If_GetSectorNumber(uint32_t Address);
static FLASH_OBProgramInitTypeDef FLASH_OBProgramInitStruct;
static FLASH_EraseInitTypeDef FLASH_EraseInitStruct;

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Unlocks the Flash to enable the flash control register access.
  * @param  None
  * @retval None
  */
void FLASH_If_FlashUnlock(void)
{
  HAL_FLASH_Unlock();
}

/**
  * @brief  Gets Flash readout protection status.
  * @param  None
  * @retval ReadOut protection status
  */
FlagStatus FLASH_If_ReadOutProtectionStatus(void)
{
  FlagStatus readoutstatus = RESET;
  
  FLASH_OBProgramInitStruct.RDPLevel = OB_RDP_LEVEL;
  
  HAL_FLASHEx_OBGetConfig(&FLASH_OBProgramInitStruct);
  
  if(OB_RDP_LEVEL == SET)
  {
    readoutstatus = SET;
  }
  else
  {
    readoutstatus = RESET;
  }
  
  return readoutstatus;
}

/**
  * @brief  Erases the required FLASH Sectors.
  * @param  Address: Start address for erasing data
  * @retval 0: Erase sectors done with success
  *         1: Erase error
  */
uint32_t FLASH_If_EraseSectors(uint32_t Address)
{
  /* Erase the user Flash area
    (area defined by APPLICATION_ADDRESS and USER_FLASH_LAST_PAGE_ADDRESS) ****/
  
  if(Address <= (uint32_t) USER_FLASH_LAST_PAGE_ADDRESS)
  {
    /* Get the 1st sector to erase */
    FirstSector = FLASH_If_GetSectorNumber(Address);
    /* Get the number of sector to erase from 1st sector */
    NbOfSectors = FLASH_If_GetSectorNumber(USER_FLASH_LAST_PAGE_ADDRESS) - FirstSector + 1;
    
    FLASH_EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
    FLASH_EraseInitStruct.Sector = FirstSector;
    FLASH_EraseInitStruct.NbSectors = NbOfSectors;
    FLASH_EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    
    if(HAL_FLASHEx_Erase(&FLASH_EraseInitStruct, &SectorError) != HAL_OK)
      return (1);
  }
  else
  {
    return (1);
  }
  
  return (0);
}

/**
  * @brief  Writes a data buffer in flash (data are 32-bit aligned).
  * @note   After writing data buffer, the flash content is checked.
  * @param  Address: Start address for writing data buffer
  * @param  Data: Pointer on data buffer  
  * @retval 0: Data successfully written to Flash memory
  *         1: Error occurred while writing data in Flash memory
  */
uint32_t FLASH_If_Write(uint32_t Address, uint32_t Data)
{
  /* Program the user Flash area word by word
    (area defined by FLASH_USER_START_ADDR and APPLICATION_ADDRESS) ***********/
  
  if(Address <= (uint32_t) USER_FLASH_LAST_PAGE_ADDRESS)
  {
    if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, Data)!= HAL_OK)
      return (1);  
  }
  else
  {
    return (1);
  }
  
  return (0);
}

/**
  * @brief  Returns the Flash sector Number of the address
  * @param  None
  * @retval The Flash sector Number of the address
  */
static uint32_t FLASH_If_GetSectorNumber(uint32_t Address)
{
  uint32_t sector = 0;
  
  if(Address < ADDR_FLASH_SECTOR_1 && Address >= ADDR_FLASH_SECTOR_0)
  {
    sector = FLASH_SECTOR_0;  
  }
  else if(Address < ADDR_FLASH_SECTOR_2 && Address >= ADDR_FLASH_SECTOR_1)
  {
    sector = FLASH_SECTOR_1;  
  }
  else if(Address < ADDR_FLASH_SECTOR_3 && Address >= ADDR_FLASH_SECTOR_2)
  {
    sector = FLASH_SECTOR_2;  
  }
  else if(Address < ADDR_FLASH_SECTOR_4 && Address >= ADDR_FLASH_SECTOR_3)
  {
    sector = FLASH_SECTOR_3;  
  }
  else if(Address < ADDR_FLASH_SECTOR_5 && Address >= ADDR_FLASH_SECTOR_4)
  {
    sector = FLASH_SECTOR_4;  
  }
  else if(Address < ADDR_FLASH_SECTOR_6 && Address >= ADDR_FLASH_SECTOR_5)
  {
    sector = FLASH_SECTOR_5;  
  }
  else if(Address < ADDR_FLASH_SECTOR_7 && Address >= ADDR_FLASH_SECTOR_6)
  {
    sector = FLASH_SECTOR_6;  
  }
  else
  {
    sector = FLASH_SECTOR_7;  
  }
  return sector;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
