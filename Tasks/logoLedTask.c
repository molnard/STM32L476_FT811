/*
 * logoLedTask.c
 *
 *  Created on: May 9, 2016
 *      Author: molnard
 */

#include "logoLedTask.h"
#include "Ncp5623.h"
#include "main.h"

void StartLogoLedTask(void const * argument)
{
	const uint8_t maxCurrent=31; //0-31
	while(1)
	{
	  ncp5623_setRGB(31,0,0);
	  ncp5623_setDimmingAuto(maxCurrent);
	  delay(2000);
	  ncp5623_setDimmingAuto(0);
	  ncp5623_setRGB(0,31,0);
	  ncp5623_setDimmingAuto(maxCurrent);
	  delay(2000);
	  ncp5623_setDimmingAuto(0);
	  ncp5623_setRGB(0,0,31);
	  ncp5623_setDimmingAuto(maxCurrent);
	  delay(2000);
	  ncp5623_setDimmingAuto(0);
	}
}
