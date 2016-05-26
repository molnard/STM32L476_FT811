/**
 *******************************************************************************
 * @file  i2c2.h
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef I2C_H_
#define I2C_H_

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include "stm32l4xx.h"
#include "cmsis_os.h"

/* Defines -------------------------------------------------------------------*/
#define I2C1_ADDRESS_AMBIENTLEDDRIVER1 0x32
#define I2C1_ADDRESS_AMBIENTLEDDRIVER2 0x33
#define I2C1_ADDRESS_AMBIENTLEDDRIVER3 0x34
#define I2C1_ADDRESS_AMBIENTLEDDRIVER4 0x35

#define I2C1_ADDRESS_LCDBACKLIGHT 0x36
#define I2C1_ADDRESS_LOGOLEDDRIVER 0x38
#define I2C1_ADDRESS_LIGHTSENSOR 0x39

#define I2C2_ADDRESS_6DSENSOR 0x1E

#define I2C3_ADDRESS_TOUCHSCREEN 0x00 //NOT USED! (FT811 handles it usually)

/* Typedefs ------------------------------------------------------------------*/
typedef struct
{
	I2C_HandleTypeDef* Handle;
	/* Used to get access to and wait for completion of an I2C transaction. */
	SemaphoreHandle_t CompleteSemaphore;
} I2C_Instance_t;


/* Function prototypes -------------------------------------------------------*/
void I2C_Init(I2C_Instance_t* hi2c_os, I2C_HandleTypeDef* i2c);
void I2C_Transmit(I2C_Instance_t* i2c,uint8_t DevAddress, uint8_t* pBuffer, uint16_t Size);
void I2C_TransmitFromISR(I2C_Instance_t* i2c,uint8_t DevAddress, uint8_t* pBuffer, uint16_t Size);
void I2C_Receive(I2C_Instance_t* i2c,uint8_t DevAddress, uint8_t* pBuffer, uint16_t Size);
void I2C_ReceiveFromISR(I2C_Instance_t* i2c,uint8_t DevAddress, uint8_t* pBuffer, uint16_t Size);
void I2C_WriteReg(I2C_Instance_t* i2c,uint16_t slave_address, uint8_t reg, uint8_t data);
uint8_t I2C_ReadReg(I2C_Instance_t* i2c,uint16_t slave_address, uint8_t reg);
void I2C_Write(I2C_Instance_t* i2c,uint16_t slave_address, uint8_t data);

#endif /* I2C_H_ */
