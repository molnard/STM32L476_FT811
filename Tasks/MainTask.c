/*
 * MainTask.c
 *
 *  Created on: May 18, 2016
 *      Author: molnard
 */

#include <MainTask.h>

const char * pictures[] = { "autumn.jpg",  "empire.jpg", "fred2.jpg" };

void StartMainTask()
{

	//home_setup();

	uint8_t counter=0;
	uint16_t xOffset=0,yOffset =0;
	/* Infinite loop */
	for(;;)
	{
		while(1)
		{
			if (Touch_IsTouched())
			{
				Touch_Event* te= Touch_GetEvent();
				if (te->isValid)
				{
					xOffset = te->x;
					yOffset = te->y;
				}
			}

			myLoadJpeg(xOffset,yOffset,pictures[counter]);

			counter++;
			if (counter==3) counter=0;
		}




		osDelay(1000);
	}
}


