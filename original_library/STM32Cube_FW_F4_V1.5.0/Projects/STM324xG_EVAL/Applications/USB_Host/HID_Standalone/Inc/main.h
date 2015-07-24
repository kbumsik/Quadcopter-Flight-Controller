/**
  ******************************************************************************
  * @file    USB_Host/HID_Standalone/Inc/main.h 
  * @author  MCD Application Team
  * @version V1.2.1
  * @date    13-March-2015
  * @brief   Header for main.c module
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
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stdio.h"
#include "usbh_core.h"
#include "stm324xg_eval_io.h"
#include "lcd_log.h"
#include "usbh_hid.h"
#include "usbh_hid_parser.h"

/* Exported types ------------------------------------------------------------*/
typedef enum {
  HID_DEMO_IDLE = 0,
  HID_DEMO_WAIT,  
  HID_DEMO_START,
  HID_DEMO_MOUSE,  
  HID_DEMO_KEYBOARD,    
  HID_DEMO_REENUMERATE,
}HID_Demo_State;

typedef enum {
  HID_MOUSE_IDLE = 0,
  HID_MOUSE_WAIT, 
  HID_MOUSE_START,       
}HID_mouse_State;

typedef enum {
  HID_KEYBOARD_IDLE = 0,
  HID_KEYBOARD_WAIT,  
  HID_KEYBOARD_START,    
}HID_keyboard_State;

typedef struct _DemoStateMachine {
  __IO HID_Demo_State     state;
  __IO HID_mouse_State    mouse_state;
  __IO HID_keyboard_State keyboard_state;   
  __IO uint8_t            select;
  __IO uint8_t            lock;
}HID_DEMO_StateMachine;

typedef enum {
  APPLICATION_IDLE = 0,
  APPLICATION_DISCONNECT,  
  APPLICATION_START,
  APPLICATION_READY,    
  APPLICATION_RUNNING,
}HID_ApplicationTypeDef;

extern USBH_HandleTypeDef hUSBHost;
extern HID_ApplicationTypeDef Appli_state;
extern HID_MOUSE_Info_TypeDef mouse_info;
extern uint8_t *DEMO_MOUSE_menu[];
extern HID_DEMO_StateMachine hid_demo;
extern uint8_t prev_select;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void Toggle_Leds(void);
void HID_SelectItem(uint8_t **menu, uint8_t item);
void HID_MenuInit(void);
void HID_MenuProcess(void);
void HID_MouseMenuProcess(void);
void HID_KeyboardMenuProcess(void);
void HID_MOUSE_ButtonReleased(uint8_t button_idx);
void HID_MOUSE_ButtonPressed(uint8_t button_idx);
void USR_MOUSE_ProcessData(HID_MOUSE_Info_TypeDef *data);
void USR_KEYBRD_ProcessData(uint8_t data);
#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
