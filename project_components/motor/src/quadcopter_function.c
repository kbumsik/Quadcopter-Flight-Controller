/*
 * quadcopter_function.c
 *
 *  Created on: Aug 15, 2015
 *      Author: kbums
 */

#include "quadcopter_config.h"
#include "stm32f4xx_hal.h"
#include "tm_stm32_usart.h"

int
get_decimal(float floatToGet)
{
  int intValue = (int)floatToGet;
  float decimal = (float)floatToGet - intValue;
  int result = decimal*DECIMAL_TO_PRINT;
  return ABS(result);
}


void
conv_FloatToString(float floatToConv, char* str)
{
  if(floatToConv>0){
      sprintf(str, "%d.%06d",(int)ABS(floatToConv),get_decimal(floatToConv));
  }
  else{
      sprintf(str, "-%d.%06d",(int)ABS(floatToConv),get_decimal(floatToConv));
  }
}

