/**
  ******************************************************************************
  * @file    FLASH/FLASH_EraseProgram/Src/main.c
  * @author  MCD Application Team modified by molnard(GitHub)
  * @version V1.4.0
  * @date    26-February-2016 modified 18-May-2016
  * @brief   This example provides a description of how to erase and program the
  *          STM32L4xx FLASH.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __EEPROM_H
#define __EEPROM_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported constants --------------------------------------------------------*/

#define ADDR_FLASH_PAGE_504   ((uint32_t)0x080fc000) /* Base @ of Page 504, 2 Kbytes */
#define ADDR_FLASH_PAGE_505   ((uint32_t)0x080fc800) /* Base @ of Page 505, 2 Kbytes */
#define ADDR_FLASH_PAGE_506   ((uint32_t)0x080fd000) /* Base @ of Page 506, 2 Kbytes */
#define ADDR_FLASH_PAGE_507   ((uint32_t)0x080fd800) /* Base @ of Page 507, 2 Kbytes */
#define ADDR_FLASH_PAGE_508   ((uint32_t)0x080fe000) /* Base @ of Page 508, 2 Kbytes */
#define ADDR_FLASH_PAGE_509   ((uint32_t)0x080fe800) /* Base @ of Page 509, 2 Kbytes */
#define ADDR_FLASH_PAGE_510   ((uint32_t)0x080ff000) /* Base @ of Page 510, 2 Kbytes */
#define ADDR_FLASH_PAGE_511   ((uint32_t)0x080ff800) /* Base @ of Page 511, 2 Kbytes */
#define DATA_32                 ((uint32_t)0x12345678)
#define DATA_64                 ((uint64_t)0x1234567812345678)

/* FLASH ld file modifications ------------------------------------------------*/
	/*full:1024K - 2K (1 PAGE) eeprom emu = 1022K*/
/* Specify the memory areas */
	/*MEMORY
	{
		FLASH (rx)      : ORIGIN = 0x8000000, LENGTH = 1022K
		RAM (xrw)      : ORIGIN = 0x20000000, LENGTH = 96K
	}*/

#define FLASH_USER_START_ADDR   ADDR_FLASH_PAGE_511   /* Start @ of user Flash area */
#define FLASH_USER_END_ADDR     ADDR_FLASH_PAGE_511 + FLASH_PAGE_SIZE - 1   /* End @ of user Flash area */
#define EEPROM_START_ADDRESS  FLASH_USER_START_ADDR /* EEPROM emulation start address:*/

/* Exported types ------------------------------------------------------------*/
typedef enum eepromAddress32_t
{
	EE_IsTouchCalibrated,
	EE_REG_TOUCH_TRANSFORM_A,
	EE_REG_TOUCH_TRANSFORM_B,
	EE_REG_TOUCH_TRANSFORM_C,
	EE_REG_TOUCH_TRANSFORM_D,
	EE_REG_TOUCH_TRANSFORM_E,
	EE_REG_TOUCH_TRANSFORM_F,
	//do not delete this or define enum under this!
	EE_LastUnusedVariable,
} EepromAddress32;


/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void EE_Format(void);
void EE_ReadVariable(EepromAddress32 virtAddress, uint32_t* data);
void EE_WriteVariable(EepromAddress32 virtAddress, uint32_t data);

#endif /* __EEPROM_H */

/******************* (C) COPYRIGHT 2007 STMicroelectronics *****END OF FILE****/
