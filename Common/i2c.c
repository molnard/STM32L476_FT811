/**
 *******************************************************************************
 * @file  i2c2.c
 * @author  Hampus Sandberg
 * @version 0.1
 * @date  2015-08-15
 * @brief
 *******************************************************************************
  Copyright (c) 2015 Hampus Sandberg.

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "i2c.h"
#include "main.h"


/* Private defines -----------------------------------------------------------*/

/* Private typedefs ----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/

/* Functions -----------------------------------------------------------------*/
/**
 * @brief	Initializes the I2C
 * @param	None
 * @retval	None
 */
void I2C_Init(I2C_Instance_t* hi2c_os, I2C_HandleTypeDef* i2c)
{
	hi2c_os->Handle = i2c;
	/* Mutex semaphore for mutual exclusion to the I2Cx device */
	hi2c_os->CompleteSemaphore = xSemaphoreCreateMutex();

	HAL_Error_Handler(hi2c_os->CompleteSemaphore == NULL);

}

static void WaitReady(I2C_Instance_t* i2c)
{
	/* -> Wait for the end of the transfer */
	/* Before starting a new communication transfer, you need to check the current
	 * state of the peripheral; if it’s busy you need to wait for the end of current
	 * transfer before starting a new one.
	 * For simplicity reasons, this example is just waiting till the end of the
	 * transfer, but application may perform other tasks while transfer operation
	 * is ongoing.
	 */
	  while (HAL_I2C_GetState(i2c->Handle) != HAL_I2C_STATE_READY)
	  {
	  }
}

/**
 * @brief	Transmits data as a master to a slave
 * @param	DevAddress: Address for the slave device
 * @param	pBuffer: Pointer to the buffer of data to send
 * @param	Size: Size of the buffer
 * @retval	None
 */
void I2C_Transmit(I2C_Instance_t* i2c,uint8_t DevAddress, uint8_t* pBuffer, uint16_t Size)
{
	/* Try to take the semaphore in case some other process is using the device */
	if (xSemaphoreTake(i2c->CompleteSemaphore, 100) == pdTRUE)
	{
		HAL_Error_Handler(HAL_I2C_Master_Transmit(i2c->Handle, (uint16_t)(DevAddress << 1), pBuffer, Size, 1000));
		WaitReady(i2c);
		xSemaphoreGive(i2c->CompleteSemaphore);
	}
}

/**
 * @brief	Transmits data as a master to a slave, used in ISR
 * @param	DevAddress: Address for the slave device
 * @param	pBuffer: Pointer to the buffer of data to send
 * @param	Size: Size of the buffer
 * @retval	None
 */
void I2C_TransmitFromISR(I2C_Instance_t* i2c,uint8_t DevAddress, uint8_t* pBuffer, uint16_t Size)
{
	/* Try to take the semaphore in case some other process is using the device */
	if (xSemaphoreTakeFromISR(i2c->CompleteSemaphore, NULL) == pdTRUE)
	{
		HAL_Error_Handler(HAL_I2C_Master_Transmit(i2c->Handle, (uint16_t)(DevAddress << 1), pBuffer, Size, 1000));
		WaitReady(i2c);
		xSemaphoreGiveFromISR(i2c->CompleteSemaphore, NULL);
	}
}

void I2C_Write(I2C_Instance_t* i2c,uint16_t slave_address, uint8_t data)
{
	uint8_t reg_data[1] = {data};
	I2C_Transmit(i2c,slave_address,reg_data,1);
}

void I2C_WriteReg(I2C_Instance_t* i2c,uint16_t slave_address, uint8_t reg, uint8_t data)
{
	uint8_t reg_data[2] = {reg,data};
	I2C_Transmit(i2c,slave_address,reg_data,2);
}

uint8_t I2C_ReadReg(I2C_Instance_t* i2c,uint16_t slave_address, uint8_t reg)
{
	uint8_t data = 0x80;
	I2C_Transmit(i2c,slave_address,(uint8_t*)&reg,1);
	I2C_Receive(i2c,slave_address,(uint8_t *) &data,1);
	return data;
}

/**
 * @brief	Transmits data as a master to a slave
 * @param	DevAddress: Address for the slave device
 * @param	Data: Pointer to a buffer where data will be stored
 * @param	Size: Size of the amount of data to receive
 * @retval	None
 */
void I2C_Receive(I2C_Instance_t* i2c,uint8_t DevAddress, uint8_t* pBuffer, uint16_t Size)
{
	/* Try to take the semaphore in case some other process is using the device */
	if (xSemaphoreTake(i2c->CompleteSemaphore, 100) == pdTRUE)
	{
		HAL_Error_Handler(HAL_I2C_Master_Receive(i2c->Handle, (uint16_t)(DevAddress << 1), pBuffer, Size, 1000));
		WaitReady(i2c);
		xSemaphoreGive(i2c->CompleteSemaphore);
	}
}

/**
 * @brief	Transmits data as a master to a slave, used in ISR
 * @param	DevAddress: Address for the slave device
 * @param	Data: Pointer to a buffer where data will be stored
 * @param	Size: Size of the amount of data to receive
 * @retval	None
 */
void I2C_ReceiveFromISR(I2C_Instance_t* i2c,uint8_t DevAddress, uint8_t* pBuffer, uint16_t Size)
{

	/* Try to take the semaphore in case some other process is using the device */
	if (xSemaphoreTakeFromISR(i2c->CompleteSemaphore, NULL) == pdTRUE)
	{
		HAL_Error_Handler(HAL_I2C_Master_Receive(i2c->Handle, (uint16_t)(DevAddress << 1), pBuffer, Size, 1000));
		WaitReady(i2c);
		xSemaphoreGiveFromISR(i2c->CompleteSemaphore, NULL);
	}
}




/* Interrupt Handlers --------------------------------------------------------*/

void i2c_probe_slaves(I2C_Instance_t* i2c)
{

	//I2C_Init(&hi2c1_os,&hi2c2_os);

	int i;
	uint8_t ch[2] = {0,0};
	HAL_StatusTypeDef i2c_status = 0xff;

	printf("Probing available I2C devices (7bit)...\r\n");
	printf("\r\n     00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F");
	printf("\r\n====================================================");
	for (i = 0; i <= 0x7F; i++)
	{
		if (!(i & 0x0F))
		{
			printf("\r\n%x  ", i >> 4);
		}
		if ((i <= 0x7) || (i > 0x7f))
		{
			printf("   ");
			continue;
		}
		ch[0] = 0;
		i2c_status = HAL_I2C_Master_Transmit(i2c->Handle, i<<1, (uint8_t*) ch,1, 100);
		if(i2c_status == HAL_OK)
		{
			printf(" %x", i);
		}
		else
		{
			printf(" --");
		}
	}
	printf("\r\n");
}




