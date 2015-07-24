/**
  ******************************************************************************
  * @file    usbd_win.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    13-March-2015   
  * @brief   USB Device functions
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
#include "dialog.h"
#include "k_module.h"
#include "usbd_res.c"
#include "k_modules_res.h"
#include "k_storage.h"
#include "usbd_app.h"

/** @addtogroup USB_DEVICE_MODULE
  * @{
  */

/** @defgroup USB_DEVICE
  * @brief usb device routines 
  * @{
  */


/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void Startup(WM_HWIN hWin, uint16_t xpos, uint16_t ypos);
/* Private typedef -----------------------------------------------------------*/
K_ModuleItem_Typedef  usb_device =
{
  10,
  "USB Device",
  &bmusbdevice,
  Startup,
  NULL,
}
;

typedef union
{
  uint32_t d32;
  struct
  {
    uint32_t sd_mounted         : 1;    
    uint32_t connection         : 1;
  }b;
}
USBDSettingsTypeDef;

/* Private defines -----------------------------------------------------------*/
#define ID_FRAMEWIN_INFO        (GUI_ID_USER + 0x01)
#define ID_USB_NOT_CONNECTED    (GUI_ID_USER + 0x02)
#define ID_MSD_NOT_CONNECTED    (GUI_ID_USER + 0x03)
#define ID_BUTTON_USB           (GUI_ID_USER + 0x04)

/* USB and MSD info */
#define ID_IMAGE1_CONNECTED      (GUI_ID_USER + 0x12)
#define ID_IMAGE1_NOT_CONNECTED  (GUI_ID_USER + 0x13)
#define ID_IMAGE2_CONNECTED      (GUI_ID_USER + 0x14)
#define ID_IMAGE2_NOT_CONNECTED  (GUI_ID_USER + 0x15)

/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Dialog resource using a WINDOW widget */
static const GUI_WIDGET_CREATE_INFO _aDialog[] = 
{
  { FRAMEWIN_CreateIndirect, "USB Mass Storage Device [mSD]",   ID_FRAMEWIN_INFO,     0,   0,   320, 215, 0, 0x64, 0 },
  { IMAGE_CreateIndirect,    "Image",        ID_USB_NOT_CONNECTED, 35,  16,  100, 100, 0, 0,    0 },
  { IMAGE_CreateIndirect,    "Image",        ID_MSD_NOT_CONNECTED, 160, 16,  100, 100, 0, 0,    0 },
  { BUTTON_CreateIndirect,   "Connect USB ", ID_BUTTON_USB,        35,  145, 225, 35,  0, 0,    0 }, 
  { IMAGE_CreateIndirect,    "",             ID_IMAGE1_CONNECTED,  91,  77,  18,  19,  0, 0,    0 },
  { IMAGE_CreateIndirect,    "",             ID_IMAGE2_CONNECTED,  216,  78,  18,  19,  0, 0,    0 },
};

USBDSettingsTypeDef         USBDSettings;

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Callback function of the CPU window
  * @param  pMsg: pointer to data structure of type WM_MESSAGE
  * @retval None
  */
static void _cbMSDConnectionStatus(WM_MESSAGE * pMsg) {
  
  static WM_HTIMER hTimerTime;
  WM_HWIN hItem;  
  static uint8_t msd_connection_Changed = 0;
  switch (pMsg->MsgId) 
  {
  case WM_CREATE:
    /* Create timer */
    hTimerTime = WM_CreateTimer(pMsg->hWin, 0, 500, 0);        
    break;
    
  case WM_TIMER:
    
    if(msd_connection_Changed != k_StorageGetStatus(MSD_DISK_UNIT))
    {
      msd_connection_Changed = k_StorageGetStatus(MSD_DISK_UNIT);
      
      if(msd_connection_Changed == 1)
      {
        hItem = WM_GetDialogItem(WM_GetParent(pMsg->hWin), ID_IMAGE2_CONNECTED);
        IMAGE_SetBitmap(hItem, &bmconnected);          
      }
      else
      {
        hItem = WM_GetDialogItem(WM_GetParent(pMsg->hWin), ID_IMAGE2_CONNECTED);
        IMAGE_SetBitmap(hItem, &bmnot_connected);  
      }
    }
       
    WM_RestartTimer(pMsg->Data.v, 500);
    break; 
    
  case WM_DELETE:
    msd_connection_Changed = 0;
    WM_DeleteTimer(hTimerTime);
    break;
    
  default:
    WM_DefaultProc(pMsg);
  }
}

/**
  * @brief  Callback routine of the dialog
  * @param  pMsg: pointer to data structure of type WM_MESSAGE
  * @retval None
  */
static void _cbDialog(WM_MESSAGE * pMsg) {
  WM_HWIN hItem;
  int Id, NCode;
  
  switch (pMsg->MsgId) {
  case WM_INIT_DIALOG:

    /* Initialization of 'System Information'  */
    hItem = pMsg->hWin;
    FRAMEWIN_AddCloseButton(hItem, FRAMEWIN_BUTTON_RIGHT, 0);     
    
    hItem = WM_GetDialogItem(pMsg->hWin, ID_USB_NOT_CONNECTED);
    IMAGE_SetBitmap(hItem, &bmUSB_not_connected); 
    hItem = WM_GetDialogItem(pMsg->hWin, ID_MSD_NOT_CONNECTED);
    IMAGE_SetBitmap(hItem, &bmsdcard_not_connected); 
    
    USBDSettings.b.connection = DISCONNECTED;
       
    WM_CreateWindowAsChild(470, 0, 10, 10, pMsg->hWin, WM_CF_SHOW | WM_CF_HASTRANS, _cbMSDConnectionStatus , 0);     
    
break;

   case WM_PAINT:

    DrawRect3D(25,  05, 250, 180);   
    
    break;
    
  case WM_DELETE:
    USBDSTOR_Stop();
    break;
    
 case WM_NOTIFY_PARENT:
   Id    = WM_GetId(pMsg->hWinSrc);    /* Id of widget */
   NCode = pMsg->Data.v;               /* Notification code */
   
   switch(Id) {
     
     /* Notifications sent by 'Connect' Button */
   case ID_BUTTON_USB: 
     if(NCode == WM_NOTIFICATION_RELEASED)
     {
       hItem = WM_GetDialogItem(pMsg->hWin, ID_BUTTON_USB);         
       if(USBDSettings.b.connection == DISCONNECTED)
       {  
         BUTTON_SetText(hItem, "Disconnect USB");
         hItem = WM_GetDialogItem(pMsg->hWin, ID_IMAGE1_CONNECTED);
         IMAGE_SetBitmap(hItem, &bmconnected);          
         USBDSettings.b.connection = CONNECTED;
         USBDSTOR_Connect();
       }
       else
       { 
         BUTTON_SetText(hItem, "Connect USB ");
         hItem = WM_GetDialogItem(pMsg->hWin, ID_IMAGE1_CONNECTED);
         IMAGE_SetBitmap(hItem, &bmnot_connected);           
         USBDSettings.b.connection = DISCONNECTED;
         USBDSTOR_Disconnect();          
       }
     }
     break;  
     
   }
   break;
   
  default:
    WM_DefaultProc(pMsg);
    break;
  }
}

/**
  * @brief  USB Device window Startup
  * @param  hWin: pointer to the parent handle.
  * @param  xpos: X position 
  * @param  ypos: Y position
  * @retval None
  */
static void Startup(WM_HWIN hWin, uint16_t xpos, uint16_t ypos)
{
  USBDSTOR_Start();
  GUI_CreateDialogBox(_aDialog, GUI_COUNTOF(_aDialog), _cbDialog, hWin, xpos, ypos);
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
