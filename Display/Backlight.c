/*
 * Backlight.c
 *
 *  Created on: Apr 11, 2016
 *      Author: molnard
 */

#include <Backlight.h>

void backlight_enable(GPIO_PinState enable)
{
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,enable);
}



void backlight_init()
{
	backlight_enable(0);
	delay(100);
	backlight_enable(1);
	delay(100);
	I2C_WriteReg(&hi2c1_os,I2C1_ADDRESS_LCDBACKLIGHT,0x10,0b00011111);
}
