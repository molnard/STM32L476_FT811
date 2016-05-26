/*
 * main.h
 *
 *  Created on: Apr 7, 2016
 *      Author: molnard
 */

#ifndef MAIN_H_
#define MAIN_H_

#include "stm32l4xx.h"
#include "cmsis_os.h"
#include "fatfs.h"
#include "Graphics.h"
#include "i2c.h"
#include "HX8357.h"
#include "sd_diskio.h"
#include "eeprom.h"

#define delay(x) osDelay(x)

extern QSPI_HandleTypeDef hqspi;
extern SD_HandleTypeDef hsd1;
extern HAL_SD_CardInfoTypedef SDCardInfo1;
extern I2C_Instance_t hi2c1_os;
extern I2C_Instance_t hi2c2_os;
extern UART_HandleTypeDef huart2;
extern RTC_HandleTypeDef hrtc;

void HAL_Error_Handler(HAL_StatusTypeDef res);


#endif /* MAIN_H_ */
