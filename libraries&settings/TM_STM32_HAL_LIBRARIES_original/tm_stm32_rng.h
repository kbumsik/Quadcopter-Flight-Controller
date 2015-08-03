/**
 * @author  Tilen Majerle
 * @email   tilen@majerle.eu
 * @website http://stm32f4-discovery.com
 * @link    
 * @version v1.0
 * @ide     Keil uVision
 * @license GNU GPL v3
 * @brief   Library template 
 *	
@verbatim
   ----------------------------------------------------------------------
    Copyright (C) Tilen Majerle, 2015
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    any later version.
     
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
   ----------------------------------------------------------------------
@endverbatim
 */
#ifndef TM_RNG_H
#define TM_RNG_H 100
/**
 * @addtogroup TM_STM32Fxxx_HAL_Libraries
 * @{
 */

/**
 * @defgroup TM_RNG
 * @brief    Random number generator library for STM32Fxxx devices
 * @{
 *
 * \par Changelog
 *
@verbatim
 Version 1.0
  - First release
@endverbatim
 *
 * \par Dependencies
 *
@verbatim
 - STM32Fxxx HAL
 - defines.h
@endverbatim
 */
#include "stm32fxxx_hal.h"
#include "defines.h"

/**
 * @defgroup TM_RNG_Functions
 * @brief    Library Functions
 * @{
 */

/**
 * @brief  Initializes RNG peripheral and enables clock
 * @param  None
 * @retval None
 */
void TM_RNG_Init(void);

/**
 * @brief  De initializes RNG peripheral and disables clock
 * @param  None
 * @retval None
 */
void TM_RNG_DeInit(void);

/**
 * @brief  Gets 32-bit random number
 * @param  None
 * @retval 32-bit random number
 */
uint32_t TM_RNG_Get(void);

/**
 * @}
 */
 
/**
 * @}
 */
 
/**
 * @}
 */

#endif

