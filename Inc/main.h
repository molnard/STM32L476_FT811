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

#define delay(x) osDelay(x)

extern QSPI_HandleTypeDef hqspi;
extern I2C_HandleTypeDef hi2c1;
extern SD_HandleTypeDef hsd1;
extern HAL_SD_CardInfoTypedef SDCardInfo1;

void HAL_Error_Handler(HAL_StatusTypeDef res);


#endif /* MAIN_H_ */
