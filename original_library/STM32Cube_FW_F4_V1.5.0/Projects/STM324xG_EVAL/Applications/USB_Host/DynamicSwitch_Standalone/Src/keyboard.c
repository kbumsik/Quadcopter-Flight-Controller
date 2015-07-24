/**
  ******************************************************************************
  * @file    USB_Host/DynamicSwitch_Standalone/Src/keyboard.c 
  * @author  MCD Application Team
  * @version V1.2.1
  * @date    13-March-2015
  * @brief   This file implements the HID keyboard functions
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
#define KYBRD_FIRST_COLUMN               (uint16_t)7
#define KYBRD_LAST_COLUMN                (uint16_t)319
#define KYBRD_FIRST_LINE                 (uint8_t) 70
#define SMALL_FONT_COLUMN_WIDTH                    8
#define SMALL_FONT_LINE_WIDTH                      15
#define KYBRD_LAST_LINE                  (uint16_t)155
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern HID_DEMO_StateMachine hid_demo;
extern uint8_t *DEMO_KEYBOARD_menu[];
extern uint8_t prev_select;
extern uint32_t hid_demo_ready;
uint8_t KeybrdCharXpos = 0;
uint16_t KeybrdCharYpos = 0;

/* Private function prototypes -----------------------------------------------*/
static void USR_KEYBRD_Init(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Manages Keyboard Menu Process.  
  * @param  None
  * @retval None
  */
void HID_KeyboardMenuProcess(void)
{
  switch(hid_demo.keyboard_state)
  {
  case HID_KEYBOARD_IDLE:
    hid_demo.keyboard_state = HID_KEYBOARD_START;
    HID_SelectItem(DEMO_KEYBOARD_menu, 0);   
    hid_demo.select = 0;
    prev_select = 0;       
    break;
    
  case HID_KEYBOARD_WAIT:
    if(hid_demo.select != prev_select)
    {
      prev_select = hid_demo.select;
      HID_SelectItem(DEMO_KEYBOARD_menu, hid_demo.select & 0x7F);
      /* Handle select item */
      if(hid_demo.select & 0x80)
      {
        hid_demo.select &= 0x7F;
        switch(hid_demo.select)
        {
        case 0: 
          hid_demo.keyboard_state = HID_KEYBOARD_START;
          break;
          
        case 1: /* Return */
          LCD_LOG_ClearTextZone();
          hid_demo.state = HID_DEMO_REENUMERATE;
          hid_demo.select = 0;
          break;
          
        default:
          break;
        }
      }
    }
    break; 
    
  case HID_KEYBOARD_START:
    USR_KEYBRD_Init();   
    hid_demo.keyboard_state = HID_KEYBOARD_WAIT;
    break;  
    
  default:
    break;
  }
}

/**
  * @brief  Init Keyboard window.     
  * @param  None
  * @retval None
  */
static void USR_KEYBRD_Init(void)
{
  LCD_LOG_ClearTextZone();
  BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
  
  BSP_LCD_DisplayStringAtLine(4, (uint8_t *)"Use Keyboard to tape characters:                                                            "); 
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  
  KeybrdCharXpos = KYBRD_FIRST_LINE;
  KeybrdCharYpos = KYBRD_FIRST_COLUMN;
}

/**
  * @brief  Processes Keyboard data.
  * @param  data: Keyboard data to be displayed
  * @retval None
  */
void USR_KEYBRD_ProcessData(uint8_t data)
{
  if(data == '\n')
  {
    KeybrdCharYpos = KYBRD_FIRST_COLUMN;
    
    /* Increment char X position */
    KeybrdCharXpos += SMALL_FONT_LINE_WIDTH;
    
    if(KeybrdCharXpos > KYBRD_LAST_LINE)
    {
      LCD_LOG_ClearTextZone();
      KeybrdCharXpos = KYBRD_FIRST_LINE;
      KeybrdCharYpos = KYBRD_FIRST_COLUMN;
    }
  }
  else if(data == '\r')
  {
    /* Manage deletion of charactter and upadte cursor location */
    if( KeybrdCharYpos == KYBRD_FIRST_COLUMN) 
    {
      /* First character of first line to be deleted */
      if(KeybrdCharXpos == KYBRD_FIRST_LINE)
      {  
        KeybrdCharYpos = KYBRD_FIRST_COLUMN; 
      }
      else
      {
        KeybrdCharXpos += SMALL_FONT_LINE_WIDTH;
        KeybrdCharYpos = (KYBRD_LAST_COLUMN + SMALL_FONT_COLUMN_WIDTH); 
      }
    }
    else
    {
      KeybrdCharYpos += SMALL_FONT_COLUMN_WIDTH;                  
    } 
    BSP_LCD_DisplayChar(KeybrdCharYpos, KeybrdCharXpos, ' '); 
  }
  else
  {
    /* Update the cursor position on LCD */
    BSP_LCD_DisplayChar(KeybrdCharYpos, KeybrdCharXpos, data);  
    
    /* Increment char Y position */
    KeybrdCharYpos += SMALL_FONT_COLUMN_WIDTH;
    
    /* Check if the Y position has reached the last column */
    if(KeybrdCharYpos == KYBRD_LAST_COLUMN)
    {
      KeybrdCharYpos = KYBRD_FIRST_COLUMN;
      
      /* Increment char X position */
      KeybrdCharXpos += SMALL_FONT_LINE_WIDTH;
    }

    if(KeybrdCharXpos > KYBRD_LAST_LINE)
    {
      LCD_LOG_ClearTextZone();
      KeybrdCharXpos = KYBRD_FIRST_LINE;
      /* Start New Display of the cursor position on LCD */
      BSP_LCD_DisplayChar(KeybrdCharYpos,KeybrdCharXpos, data);  
    }
  }
}          

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
