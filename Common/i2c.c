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

#include <stdbool.h>

/* Private defines -----------------------------------------------------------*/

/* Private typedefs ----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static I2C_HandleTypeDef I2C_Handle;

static SemaphoreHandle_t xSemaphore;
static bool prvInitialized = false;

/* Private function prototypes -----------------------------------------------*/

/* Functions -----------------------------------------------------------------*/
/**
 * @brief	Initializes the I2C
 * @param	None
 * @retval	None
 */
void I2C_Init()
{
	I2C_Handle = hi2c1;
	/* Make sure we only initialize it once */
	if (!prvInitialized)
	{
		/* Mutex semaphore for mutual exclusion to the I2C2 device */
		xSemaphore = xSemaphoreCreateMutex();

		if (xSemaphoreTake(xSemaphore, 100) == pdTRUE)
		{
			//i2c bus already initialized by Cube

			//	/* NVIC Configuration */
			//	NVIC_InitTypeDef NVIC_InitStructure;
			//	/* Event interrupt */
			//	NVIC_InitStructure.NVIC_IRQChannel = I2C1_EV_IRQn;
			//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configMAX_SYSCALL_INTERRUPT_PRIORITY;
			//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
			//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
			//	NVIC_Init(&NVIC_InitStructure);
			//	/* Error interrupt */
			//	NVIC_InitStructure.NVIC_IRQChannel = I2C1_ER_IRQn;
			//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 14;
			//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
			//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
			//	NVIC_Init(&NVIC_InitStructure);

			/* I2C Init */
			//HAL_I2C_Init(&I2C_Handle);

			/* Enable the I2C1 interrupts */
			//	I2C_ITConfig(I2C_PERIPHERAL, I2C_IT_EVT, ENABLE);
			//	I2C_ITConfig(I2C_PERIPHERAL, I2C_IT_ERR, ENABLE);

			xSemaphoreGive(xSemaphore);
		}

		prvInitialized = true;
	}
}

void WaitReady()
{
	/* -> Wait for the end of the transfer */
	/* Before starting a new communication transfer, you need to check the current
	 * state of the peripheral; if it’s busy you need to wait for the end of current
	 * transfer before starting a new one.
	 * For simplicity reasons, this example is just waiting till the end of the
	 * transfer, but application may perform other tasks while transfer operation
	 * is ongoing.
	 */
	  while (HAL_I2C_GetState(&I2C_Handle) != HAL_I2C_STATE_READY)
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
void I2C_Transmit(uint8_t DevAddress, uint8_t* pBuffer, uint16_t Size)
{
	HAL_StatusTypeDef i2c_status = 0xff;
	/* Try to take the semaphore in case some other process is using the device */
	if (xSemaphoreTake(xSemaphore, 100) == pdTRUE)
	{
		i2c_status = HAL_I2C_Master_Transmit(&I2C_Handle, (uint16_t)(DevAddress << 1), pBuffer, Size, 1000);
		xSemaphoreGive(xSemaphore);
	}
	if (i2c_status != HAL_OK)
	{
#ifdef DEBUG
		fprintf(stderr,"HAL_I2C_Master_Transmit error: 0x%x - 0x%lx\r\n",i2c_status, HAL_I2C_GetError(&hi2c1));
#endif
		//HAL_Error_Handler(i2c_status);
		//return 0;
	}
	WaitReady();
}

/**
 * @brief	Transmits data as a master to a slave, used in ISR
 * @param	DevAddress: Address for the slave device
 * @param	pBuffer: Pointer to the buffer of data to send
 * @param	Size: Size of the buffer
 * @retval	None
 */
void I2C_TransmitFromISR(uint8_t DevAddress, uint8_t* pBuffer, uint16_t Size)
{
	HAL_StatusTypeDef i2c_status = 0xff;
	/* Try to take the semaphore in case some other process is using the device */
	if (xSemaphoreTakeFromISR(xSemaphore, NULL) == pdTRUE)
	{
		i2c_status = HAL_I2C_Master_Transmit(&I2C_Handle, (uint16_t)(DevAddress << 1), pBuffer, Size, 1000);
		xSemaphoreGiveFromISR(xSemaphore, NULL);
	}
	if (i2c_status != HAL_OK)
	{
#ifdef DEBUG
		fprintf(stderr,"HAL_I2C_Master_Transmit error: 0x%x - 0x%lx\r\n",i2c_status, HAL_I2C_GetError(&hi2c1));
#endif
		HAL_Error_Handler(i2c_status);
		//return 0;
	}
	WaitReady();
}

void I2C_WriteReg(uint16_t slave_address, uint8_t reg, uint8_t data)
{

	uint8_t reg_data[2] = {reg,data};
	I2C_Transmit(slave_address,reg_data,2);

}

uint8_t I2C_ReadReg(uint16_t slave_address, uint8_t reg)
{
	uint8_t data = 0x80;
	I2C_Transmit(slave_address,(uint8_t*)&reg,1);
	I2C_Receive(slave_address,(uint8_t *) &data,1);
	return data;
}

/**
 * @brief	Transmits data as a master to a slave
 * @param	DevAddress: Address for the slave device
 * @param	Data: Pointer to a buffer where data will be stored
 * @param	Size: Size of the amount of data to receive
 * @retval	None
 */
void I2C_Receive(uint8_t DevAddress, uint8_t* pBuffer, uint16_t Size)
{
	HAL_StatusTypeDef i2c_status = 0xff;
	/* Try to take the semaphore in case some other process is using the device */
	if (xSemaphoreTake(xSemaphore, 100) == pdTRUE)
	{
		i2c_status = HAL_I2C_Master_Receive(&I2C_Handle, (uint16_t)(DevAddress << 1), pBuffer, Size, 1000);
		xSemaphoreGive(xSemaphore);
	}
	if (i2c_status != HAL_OK)
	{
#ifdef DEBUG
		fprintf(stderr,"HAL_I2C_Master_Receive error: 0x%x - 0x%lx\r\n",i2c_status, HAL_I2C_GetError(&hi2c1));
#endif
		HAL_Error_Handler(i2c_status);
	}
	WaitReady();
}

/**
 * @brief	Transmits data as a master to a slave, used in ISR
 * @param	DevAddress: Address for the slave device
 * @param	Data: Pointer to a buffer where data will be stored
 * @param	Size: Size of the amount of data to receive
 * @retval	None
 */
void I2C_ReceiveFromISR(uint8_t DevAddress, uint8_t* pBuffer, uint16_t Size)
{
	HAL_StatusTypeDef i2c_status = 0xff;
	/* Try to take the semaphore in case some other process is using the device */
	if (xSemaphoreTakeFromISR(xSemaphore, NULL) == pdTRUE)
	{
		i2c_status = HAL_I2C_Master_Receive(&I2C_Handle, (uint16_t)(DevAddress << 1), pBuffer, Size, 1000);
		xSemaphoreGiveFromISR(xSemaphore, NULL);
	}
	if (i2c_status != HAL_OK)
	{
#ifdef DEBUG
		fprintf(stderr,"HAL_I2C_Master_Receive error: 0x%x - 0x%lx\r\n",i2c_status, HAL_I2C_GetError(&hi2c1));
#endif
		HAL_Error_Handler(i2c_status);
	}
	WaitReady();
}




/* Interrupt Handlers --------------------------------------------------------*/

void i2c_probe_slaves()
{
	if (!prvInitialized)
	{
		I2C_Init();
	}
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
		i2c_status = HAL_I2C_Master_Transmit(&I2C_Handle, i<<1, (uint8_t*) ch,1, 100);
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
void i2c_probe_slave(uint8_t address)
{
	if (!prvInitialized)
	{
		I2C_Init();
	}
	int i;
	printf("Reading available registers (8bit)...\r\n");
	printf("\r\n     00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F");
	printf("\r\n====================================================");
	for (i = 0; i <= 0xFF; i++)
	{
		if (!(i & 0x0F))
		{
			printf("\r\n%02X  ", i >> 4);
		}
		uint8_t result=I2C_ReadReg(address,i);
		printf(" %02X", result);

	}
	printf("\r\n");
}



