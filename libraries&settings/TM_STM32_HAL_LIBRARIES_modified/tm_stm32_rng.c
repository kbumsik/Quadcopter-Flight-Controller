/**	
 * |----------------------------------------------------------------------
 * | Copyright (C) Tilen Majerle, 2014
 * | 
 * | This program is free software: you can redistribute it and/or modify
 * | it under the terms of the GNU General Public License as published by
 * | the Free Software Foundation, either version 3 of the License, or
 * | any later version.
 * |  
 * | This program is distributed in the hope that it will be useful,
 * | but WITHOUT ANY WARRANTY; without even the implied warranty of
 * | MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * | GNU General Public License for more details.
 * | 
 * | You should have received a copy of the GNU General Public License
 * | along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * |----------------------------------------------------------------------
 */
#include "tm_stm32_rng.h"

void TM_RNG_Init(void) {
	/* Enable RNG clock source */
	__HAL_RCC_RNG_CLK_ENABLE();
	
	/* RNG Peripheral enable */
	RNG->CR |= RNG_CR_RNGEN;
}

void TM_RNG_DeInit(void) {
	/* Disable RNG peripheral */
	RNG->CR &= ~RNG_CR_RNGEN;
	
	/* Disable RNG clock source */
	__HAL_RCC_RNG_CLK_DISABLE();
}

uint32_t TM_RNG_Get(void) {
	/* Wait until one RNG number is ready */
	while (!(RNG->SR & (RNG_SR_DRDY)));

	/* Get a 32-bit Random number */
	return RNG->DR;
}
