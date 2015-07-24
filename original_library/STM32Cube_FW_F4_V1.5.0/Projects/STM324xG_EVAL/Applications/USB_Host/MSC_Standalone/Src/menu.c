/**
  ******************************************************************************
  * @file    USB_Host/MSC_Standalone/Src/menu.c 
  * @author  MCD Application Team
  * @version V1.2.1
  * @date    13-March-2015
  * @brief   This file implements Menu Functions
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
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
MSC_DEMO_StateMachine msc_demo;
uint8_t prev_select = 0;
uint8_t *MSC_main_menu[] = 
{
  (uint8_t *)"      1 - File Operations                                                   ",
  (uint8_t *)"      2 - Explorer Disk                                                     ",
  (uint8_t *)"      3 - Re-Enumerate                                                      ",
};

/* Private function prototypes -----------------------------------------------*/
static void MSC_SelectItem(uint8_t **menu, uint8_t item);
static void MSC_DEMO_ProbeKey(JOYState_TypeDef state);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Demo state machine.
  * @param  None
  * @retval None
  */
void Menu_Init(void)
{
  BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
  BSP_LCD_DisplayStringAtLine(15, (uint8_t *)"Use [Joystick Left/Right] to scroll up/down");
  BSP_LCD_DisplayStringAtLine(16, (uint8_t *)"Use [Joystick Up/Down] to scroll MSC menu");
  msc_demo.state = MSC_DEMO_IDLE;
  MSC_SelectItem(MSC_main_menu, 0); 
}

/**
  * @brief  Manages MSC Menu Process.
  * @param  None
  * @retval None
  */
void MSC_MenuProcess(void)
{
  switch(msc_demo.state)
  {
  case MSC_DEMO_IDLE:
    MSC_SelectItem(MSC_main_menu, 0); 
    msc_demo.state = MSC_DEMO_WAIT;
    msc_demo.select = 0;
    break;    
    
  case MSC_DEMO_WAIT:
    if(msc_demo.select != prev_select)
    {
      prev_select = msc_demo.select;
      MSC_SelectItem(MSC_main_menu, msc_demo.select & 0x7F);
      
      /* Handle select item */
      if(msc_demo.select & 0x80)
      {
        switch(msc_demo.select & 0x7F)
        {
        case 0:
          msc_demo.state = MSC_DEMO_FILE_OPERATIONS;  
          break;
          
        case 1:
          msc_demo.state = MSC_DEMO_EXPLORER;  
          break;
          
        case 2:
          msc_demo.state = MSC_REENUMERATE;  
          break;
          
        default:
          break;
        }
      }
    }
    break;
    
  case MSC_DEMO_FILE_OPERATIONS:
    /* Read and Write File Here */
    if(Appli_state == APPLICATION_READY)
    {
      MSC_File_Operations();
    }
    msc_demo.state = MSC_DEMO_WAIT;
    break; 
    
  case MSC_DEMO_EXPLORER:
    /* Display disk content */
    if(Appli_state == APPLICATION_READY)
    {        
      Explore_Disk("0:/", 1);
    }
    msc_demo.state = MSC_DEMO_WAIT;
    break; 
    
  case MSC_REENUMERATE:
    /* Force MSC Device to re-enumerate */
    USBH_ReEnumerate(&hUSBHost); 
    msc_demo.state = MSC_DEMO_WAIT;
    break;
    
  default:
    break;
  }
  msc_demo.select &= 0x7F;
} 

/**
  * @brief  Manages the menu on the screen.
  * @param  menu: Menu table
  * @param  item: Selected item to be highlighted
  * @retval None
  */
static void MSC_SelectItem(uint8_t **menu, uint8_t item)
{
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  
  switch(item)
  {
  case 0: 
    BSP_LCD_SetBackColor(LCD_COLOR_MAGENTA);
    BSP_LCD_DisplayStringAtLine(17, menu[0]);
    BSP_LCD_SetBackColor(LCD_COLOR_BLUE);    
    BSP_LCD_DisplayStringAtLine(18, menu[1]);
    BSP_LCD_DisplayStringAtLine(19, menu[2]); 
    break;
    
  case 1: 
    BSP_LCD_SetBackColor(LCD_COLOR_BLUE);
    BSP_LCD_DisplayStringAtLine(17, menu[0]);
    BSP_LCD_SetBackColor(LCD_COLOR_MAGENTA);    
    BSP_LCD_DisplayStringAtLine(18, menu[1]);
    BSP_LCD_SetBackColor(LCD_COLOR_BLUE);  
    BSP_LCD_DisplayStringAtLine(19, menu[2]); 
    break;
    
  case 2: 
    BSP_LCD_SetBackColor(LCD_COLOR_BLUE);
    BSP_LCD_DisplayStringAtLine(17, menu[0]);
    BSP_LCD_SetBackColor(LCD_COLOR_BLUE);    
    BSP_LCD_DisplayStringAtLine(18, menu[1]);
    BSP_LCD_SetBackColor(LCD_COLOR_MAGENTA);  
    BSP_LCD_DisplayStringAtLine(19, menu[2]); 
    break;
  }
  BSP_LCD_SetBackColor(LCD_COLOR_BLACK); 
}

/**
  * @brief  Probes the MSC joystick state.
  * @param  state: Joystick state
  * @retval None
  */
static void MSC_DEMO_ProbeKey(JOYState_TypeDef state)
{
  /* Handle Menu inputs */
  if((state == JOY_UP) && (msc_demo.select > 0))
  {
    msc_demo.select--;
  }
  else if((state == JOY_DOWN) && (msc_demo.select < 2))
  {
    msc_demo.select++;
  }
  else if(state == JOY_SEL)
  {
    msc_demo.select |= 0x80;
  }  
}

/**
  * @brief  EXTI line detection callbacks.
  * @param  GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  static JOYState_TypeDef JoyState = JOY_NONE;
  
  if(GPIO_Pin == GPIO_PIN_2)
  {    
    /* Get the Joystick State */
    JoyState = BSP_JOY_GetState();
    
    MSC_DEMO_ProbeKey(JoyState); 
    
    switch(JoyState)
    {
    case JOY_LEFT:
      LCD_LOG_ScrollBack();
      break;
           
    case JOY_RIGHT:
      LCD_LOG_ScrollForward();
      break;          
      
    default:
      break;           
    }
    /* Clear joystick interrupt pending bits */
    BSP_IO_ITClear(JOY_ALL_PINS);
  }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
