/*
 * kb_functions.h
 *
 *  Created on: Jul 21, 2015
 *      Author: BumSik
 */

#ifndef KB_STM32F4_FUNCTIONS_H_
#define KB_STM32F4_FUNCTIONS_H_ 100

#include <stdio.h>

#define DECIMAL_TO_PRINT 1000000
#define ABS(num)	((num>0)? num : -num)

int get_decimal(float floatToGet);

void conv_FloatToString(float floatToConv, char* str);



#endif /* KB_STM32F4_FUNCTIONS_H_ */
