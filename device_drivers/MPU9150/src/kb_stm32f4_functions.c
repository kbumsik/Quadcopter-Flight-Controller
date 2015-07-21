/*
 * kb_stm32f4_functions.c
 *
 *  Created on: Jul 21, 2015
 *      Author: BumSik
 */


#include "kb_stm32f4_functions.h"

int
get_decimal(float floatToGet)
{
  int intValue = (int)floatToGet;
  float decimal = (float)floatToGet - intValue;
  int result = decimal*DECIMAL_TO_PRINT;
  return result;
}
