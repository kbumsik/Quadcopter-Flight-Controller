/**
  ******************************************************************************
  * @file    k_modules_res.h
  * @author  MCD Application Team
  * @version V1.2.1
  * @date    13-March-2015
  * @brief   Header for k_modules_res.c file
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
#ifndef __K_MODULE_RES_H
#define __K_MODULE_RES_H

#ifdef __cplusplus
 extern "C" {
#endif
 
/* Includes ------------------------------------------------------------------*/   
#include <stdlib.h>
#include "GUI.h"

#ifndef GUI_CONST_STORAGE
  #define GUI_CONST_STORAGE const
#endif

/* Exported types ------------------------------------------------------------*/   
extern GUI_CONST_STORAGE GUI_BITMAP bmplay_pressed;
extern GUI_CONST_STORAGE GUI_BITMAP bmplay_not_pressed;
extern GUI_CONST_STORAGE GUI_BITMAP bmnext_not_pressed;
extern GUI_CONST_STORAGE GUI_BITMAP bmnext_pressed;
extern GUI_CONST_STORAGE GUI_BITMAP bmprevious_not_pressed;
extern GUI_CONST_STORAGE GUI_BITMAP bmprevious_pressed;
extern GUI_CONST_STORAGE GUI_BITMAP bmstop_pressed;
extern GUI_CONST_STORAGE GUI_BITMAP bmstop_not_pressed;
extern GUI_CONST_STORAGE GUI_BITMAP bmadd_pressed;
extern GUI_CONST_STORAGE GUI_BITMAP bmadd_not_pressed;
extern GUI_CONST_STORAGE GUI_BITMAP bmplaylist_pressed;
extern GUI_CONST_STORAGE GUI_BITMAP bmplaylist_not_pressed;
extern GUI_CONST_STORAGE GUI_BITMAP bmopen_pressed;
extern GUI_CONST_STORAGE GUI_BITMAP bmopen_not_pressed;
extern GUI_CONST_STORAGE GUI_BITMAP bmclose_not_pressed;
extern GUI_CONST_STORAGE GUI_BITMAP bmclose_pressed;
extern GUI_CONST_STORAGE GUI_BITMAP bmhide_not_pressed;
extern GUI_CONST_STORAGE GUI_BITMAP bmhide_pressed;
extern GUI_CONST_STORAGE GUI_BITMAP bmpause_pressed;
extern GUI_CONST_STORAGE GUI_BITMAP bmpause_not_pressed;
extern GUI_CONST_STORAGE GUI_BITMAP bmspeaker_not_mute;
extern GUI_CONST_STORAGE GUI_BITMAP bmspeaker_mute;
extern GUI_CONST_STORAGE GUI_BITMAP bmrepeat1;
extern GUI_CONST_STORAGE GUI_BITMAP bmrepeat_all;
extern GUI_CONST_STORAGE GUI_BITMAP bmrepeat_off;
extern GUI_CONST_STORAGE GUI_BITMAP bmwindowfullscreen;
extern GUI_CONST_STORAGE GUI_BITMAP bmwindownofullscreen;
extern GUI_CONST_STORAGE GUI_BITMAP bmsettings_button_not_pressed;
extern GUI_CONST_STORAGE GUI_BITMAP bmsettings_button_pressed;
extern GUI_CONST_STORAGE GUI_BITMAP bmstop2_pressed;
extern GUI_CONST_STORAGE GUI_BITMAP bmstop2_not_pressed;
extern GUI_CONST_STORAGE GUI_BITMAP bmstart_pressed;
extern GUI_CONST_STORAGE GUI_BITMAP bmstart_not_pressed;
extern GUI_CONST_STORAGE GUI_BITMAP bmok32;
extern GUI_CONST_STORAGE GUI_BITMAP bmDelete32;

/* Exported constants --------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void DrawRect3D(int x0, int y0, int width, int height);

#ifdef __cplusplus
}
#endif

#endif /*__K_MODULE_RES_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
