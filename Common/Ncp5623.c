/*
 * Ncp5623.c
 *
 *  Created on: Apr 19, 2016
 *      Author: molnard
 */

#include <Ncp5623.h>


static uint8_t pwm1;
static uint8_t pwm2;
static uint8_t pwm3;
static uint8_t step;
static uint8_t iled;

/*
 * ncp5623 write only IC
 */
static void ncp5623_write(uint8_t reg,uint8_t data)
{
	uint8_t out= reg | (data & 0b11111);
	I2C_Write(&hi2c1_os,I2C1_ADDRESS_LOGOLEDDRIVER,out);
	switch (reg) {
		case ncp5623_REG_PWM1:
				pwm1 = data;
			break;
		case ncp5623_REG_PWM2:
				pwm2 = data;
			break;
		case ncp5623_REG_PWM3:
				pwm3 = data;
			break;
		case ncp5623_REG_DIMSTEP:
				step = data;
			break;
		case ncp5623_REG_ILED:
				iled = data;
			break;
		default:
			break;
	}
}

void ncp5623_init()
{
	ncp5623_write(ncp5623_REG_ILED,0);
	ncp5623_write(ncp5623_REG_PWM1,0);
	ncp5623_write(ncp5623_REG_PWM2,0);
	ncp5623_write(ncp5623_REG_PWM3,0);
	ncp5623_setDimming(ncp5623_DIMDOWN,0,5);
}

void ncp5623_setRGB(uint8_t red,uint8_t green,uint8_t blue)
{
	ncp5623_write(ncp5623_REG_PWM1,blue);
	ncp5623_write(ncp5623_REG_PWM2,green);
	ncp5623_write(ncp5623_REG_PWM3,red);
}

/** Set static LED Current
 *  data - value of current draw for all LEDs range 0-31
 *  @return status of command
 */
void ncp5623_setLEDCurrent(uint8_t data)
{
	ncp5623_write(ncp5623_REG_ILED,data);
}

/** Set PWM mode for specific LED
 *  @lednum - selects LED
 *  @data - PWM value to set  range 0-31 0-100% Pulse width
 *  @return status of command
 */
void ncp5623_setPWM(ncp5623_lednum_e lednum, uint8_t data )
{
    switch (lednum) {
        case ncp5623_LED1:
        	ncp5623_write(ncp5623_REG_PWM1,data);
            break;
        case ncp5623_LED2:
        	ncp5623_write(ncp5623_REG_PWM2,data);
            break;
        case ncp5623_LED3:
        	ncp5623_write(ncp5623_REG_PWM3,data);
            break;
    }
}

/** Set Dimming mode for all LEDs
     *  @dimdir - direction of dimming
     *  @endstep - ending step of ramp up or ramp down range 0-31
     *  @steptime - time per step range 0-31 in 8 msec multiples
     *  @return status of command
     */
void ncp5623_setDimming(ncp5623_dimdirection_e dimdir, uint8_t endstep, uint8_t steptime)
{
    if (dimdir == ncp5623_DIMDOWN)
    	ncp5623_write(ncp5623_REG_DIMDOWN, endstep);
    else
    	ncp5623_write(ncp5623_REG_DIMUP, endstep);

    ncp5623_write(ncp5623_REG_DIMSTEP, steptime);
}

void ncp5623_setDimmingAuto(uint8_t endstep)
{
	if (endstep==iled) return;
	uint8_t diff=0;
	if (endstep<iled)
	{
		diff=iled-endstep;
		ncp5623_write(ncp5623_REG_DIMDOWN, endstep);
	}
	else
	{
		diff=endstep-iled;
		ncp5623_write(ncp5623_REG_DIMUP, endstep);
	}
	ncp5623_write(ncp5623_REG_DIMSTEP, step);
	uint32_t timetoWait=(uint32_t)(diff+2)*8*step; //+2 time needed until dimming end(found out during test, undocumented)
	osDelay(timetoWait);
	iled = endstep;
}



