/**
  ******************************************************************************
  * @file           : USB_DEVICE  
  * @version        : v1.0_Cube
  * @brief          : This file implements the USB Device 
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  * 1. Redistributions of source code must retain the above copyright notice,
  * this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  * this list of conditions and the following disclaimer in the documentation
  * and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of its contributors
  * may be used to endorse or promote products derived from this software
  * without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/

#include "usb_device.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_msc.h"
#include "usbd_storage_if.h"

/* Return USBD_OK if the Battery Charging Detection mode (BCD) is used, else USBD_FAIL */
extern USBD_StatusTypeDef USBD_LL_BatteryCharging(USBD_HandleTypeDef *pdev);
/* USB Device Core handle declaration */
USBD_HandleTypeDef hUsbDeviceFS;

/* init function */				        
void MX_USB_DEVICE_Init(void)
{
  /* Init Device Library,Add Supported Class and Start the library*/
  USBD_Init(&hUsbDeviceFS, &FS_Desc, DEVICE_FS);
  USBD_RegisterClass(&hUsbDeviceFS, &USBD_MSC);
  USBD_MSC_RegisterStorage(&hUsbDeviceFS, &USBD_Storage_Interface_fops_FS);
  /* Verify if the Battery Charging Detection mode (BCD) is used : */
  /* If yes, the USB device is started in the HAL_PCDEx_BCD_Callback */
  /* upon reception of PCD_BCD_DISCOVERY_COMPLETED message. */
  /* If no, the USB device is started now. */
  if (USBD_LL_BatteryCharging(&hUsbDeviceFS) != USBD_OK) {
  USBD_Start(&hUsbDeviceFS);
  }
}

/**
  * @brief  HAL_PCDEx_BCD_Callback : Send BCD message to user layer
  * @param  hpcd: PCD handle
  * @param  msg: LPM message
  * @retval HAL status
  */
void HAL_PCDEx_BCD_Callback(PCD_HandleTypeDef *hpcd, PCD_BCD_MsgTypeDef msg)
{
  USBD_HandleTypeDef usbdHandle = hUsbDeviceFS;
  /* USER CODE BEGIN 7 */
  if (hpcd->battery_charging_active == ENABLE)
  {
    switch(msg)
    {    
      case PCD_BCD_CONTACT_DETECTION:
    
      break;
    
      case PCD_BCD_STD_DOWNSTREAM_PORT:
   
      break;
    
      case PCD_BCD_CHARGING_DOWNSTREAM_PORT:
   
      break;
    
      case PCD_BCD_DEDICATED_CHARGING_PORT:
   
      break;
    
      case PCD_BCD_DISCOVERY_COMPLETED:
        USBD_Start(&usbdHandle);
      break;
    
      case PCD_BCD_ERROR:
      default:
      break;
    }
  }
  /* USER CODE END 7 */
}
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
