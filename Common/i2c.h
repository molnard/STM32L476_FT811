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
#include "main.h"



/* Defines -------------------------------------------------------------------*/
#define I2C1_ADDRESS_LCDBACKLIGHT 0x36
#define I2C1_ADDRESS_TOUCHSCREEN 0x38
#define I2C1_ADDRESS_LEDDRIVER 0x32

/* Typedefs ------------------------------------------------------------------*/
/* Function prototypes -------------------------------------------------------*/
void I2C_Init();
void I2C_Transmit(uint8_t DevAddress, uint8_t *Data, uint16_t Size);
void I2C_TransmitFromISR(uint8_t DevAddress, uint8_t* pBuffer, uint16_t Size);
void I2C_Receive(uint8_t DevAddress, uint8_t *Data, uint16_t Size);
void I2C_ReceiveFromISR(uint8_t DevAddress, uint8_t* pBuffer, uint16_t Size);
uint8_t I2C_ReadReg(uint16_t slave_address, uint8_t reg);
void I2C_WriteReg(uint16_t slave_address, uint8_t reg, uint8_t data);

#endif /* I2C_H_ */
