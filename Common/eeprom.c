/******************** (C) COPYRIGHT 2007 STMicroelectronics ********************
* File Name          : eeprom.c
* Author             : MCD Application Team
* Version            : V1.0
* Date               : 10/08/2007
* Description        : This file provides all the EEPROM emulation firmware functions.
********************************************************************************
* THE PRESENT SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "eeprom.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint32_t EE_storage[EE_LastUnusedVariable];


/**
  * @brief  Gets the page of a given address
  * @param  Addr: Address of the FLASH Memory
  * @retval The page of a given address
  */
static uint32_t GetPage(uint32_t Addr)
{
  uint32_t page = 0;

  if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
  {
    /* Bank 1 */
    page = (Addr - FLASH_BASE) / FLASH_PAGE_SIZE;
  }
  else
  {
    /* Bank 2 */
    page = (Addr - (FLASH_BASE + FLASH_BANK_SIZE)) / FLASH_PAGE_SIZE;
  }

  return page;
}

/**
  * @brief  Gets the bank of a given address
  * @param  Addr: Address of the FLASH Memory
  * @retval The bank of a given address
  */
static uint32_t GetBank(uint32_t Addr)
{
  uint32_t bank = 0;

  if (READ_BIT(SYSCFG->MEMRMP, SYSCFG_MEMRMP_FB_MODE) == 0)
  {
  	/* No Bank swap */
    if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
    {
      bank = FLASH_BANK_1;
    }
    else
    {
      bank = FLASH_BANK_2;
    }
  }
  else
  {
  	/* Bank swap */
    if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
    {
      bank = FLASH_BANK_2;
    }
    else
    {
      bank = FLASH_BANK_1;
    }
  }

  return bank;
}


/*******************************************************************************
* Function Name  : EE_Format
* Description    : Erases PAGE0 and PAGE1 and writes VALID_PAGE header to PAGE0
* Input          : None
* Output         : None
* Return         : Status of the last operation (Flash write or erase) done during
*                  EEPROM formating
*******************************************************************************/
void EE_Format(void)
{
	/* Erase the user Flash area
	(area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/

	/* Clear OPTVERR bit set on virgin samples */
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);
	/* Get the 1st page to erase */
	uint32_t FirstPage = GetPage(FLASH_USER_START_ADDR);
	/* Get the number of pages to erase from 1st page */
	uint32_t NbOfPages = GetPage(FLASH_USER_END_ADDR) - FirstPage + 1;
	/* Get the bank */
	uint32_t BankNumber = GetBank(FLASH_USER_START_ADDR);
	uint32_t PAGEError;
	/* Fill EraseInit structure*/
	FLASH_EraseInitTypeDef EraseInitStruct;
	EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.Banks       = BankNumber;
	EraseInitStruct.Page        = FirstPage;
	EraseInitStruct.NbPages     = NbOfPages;

	/* Note: If an erase operation in Flash memory also concerns data in the data or instruction cache,
	 you have to make sure that these data are rewritten before they are accessed during code
	 execution. If this cannot be done safely, it is recommended to flush the caches by setting the
	 DCRST and ICRST bits in the FLASH_CR register. */
	HAL_Error_Handler(HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError));
	/*
	  IF Error occurred while page erase.
	  User can add here some code to deal with this error.
	  PAGEError will contain the faulty page and then to know the code error on this page,
	  user can call function 'HAL_FLASH_GetError()'
	*/
	uint32_t Address = FLASH_USER_START_ADDR;
	while (Address < FLASH_USER_END_ADDR)
	{
		HAL_Error_Handler(HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address, DATA_64));
		Address = Address + 8;
	}
	HAL_FLASH_Lock();

	/* Check if the programmed data is OK
	  MemoryProgramStatus = 0: data programmed correctly
	  MemoryProgramStatus != 0: number of words not programmed correctly ******/
	Address = FLASH_USER_START_ADDR;
	uint32_t MemoryProgramStatus = 0x0;
	while (Address < FLASH_USER_END_ADDR)
	{
		uint32_t data32 = *(__IO uint32_t *)Address;
		if (data32 != DATA_32)
		{
		  MemoryProgramStatus++;
		}
		Address = Address + 4;
	}
	/*Check if there is an issue to program data*/
	if (MemoryProgramStatus == 0)
	{
		/* No error detected. Switch on LED2*/
		return;
	}
	else
	{
	/* Error detected. LED2 will blink with 1s period */
		HAL_Error_Handler(HAL_ERROR);
	}
}

void EE_ReadVariable(EepromAddress32 virtAddress, uint32_t* data)
{
	uint32_t address=FLASH_USER_START_ADDR + virtAddress*4;
	uint32_t data32 = *(__IO uint32_t *)address;
	*data = data32;
}

void EE_WriteVariable(EepromAddress32 virtAddress, uint32_t data)
{
	uint16_t length=EE_LastUnusedVariable;
	uint32_t address = FLASH_USER_START_ADDR;
	for (uint16_t varIndex = 0;varIndex < length;varIndex++)
	{
		uint32_t data32 = *(__IO uint32_t *)address;
		EE_storage[varIndex] = data32;
		address = address + 4;
	}
	EE_storage[virtAddress] = data; //write the new data

	HAL_FLASH_Unlock();
	/* Get the 1st page to erase */
	uint32_t FirstPage = GetPage(FLASH_USER_START_ADDR);
	/* Get the number of pages to erase from 1st page */
	uint32_t NbOfPages = GetPage(FLASH_USER_END_ADDR) - FirstPage + 1;
	/* Get the bank */
	uint32_t BankNumber = GetBank(FLASH_USER_START_ADDR);
	uint32_t PAGEError;
	/* Fill EraseInit structure*/
	FLASH_EraseInitTypeDef EraseInitStruct;
	EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.Banks       = BankNumber;
	EraseInitStruct.Page        = FirstPage;
	EraseInitStruct.NbPages     = NbOfPages;
	HAL_Error_Handler(HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError));
	address = FLASH_USER_START_ADDR;
	for (uint16_t varIndex = 0;varIndex < length;varIndex+=2)
	{
		uint64_t toWrite=EE_storage[varIndex];
		if (varIndex+1<length)
		{
			toWrite|= ((uint64_t)EE_storage[varIndex+1])<<32;
		}
		HAL_Error_Handler(HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, toWrite));
		address = address + 8;
	}
	HAL_FLASH_Lock();

}



/******************* (C) COPYRIGHT 2007 STMicroelectronics *****END OF FILE****/
