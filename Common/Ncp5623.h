/*
 * Ncp5623.h
 *
 *  Created on: Apr 19, 2016
 *      Author: molnard
 */

#ifndef NCP5623_H_
#define NCP5623_H_

//LOGO LED DRIVER

#include "main.h"

#define ncp5623_REG_SHUTDOWN 	0b00000000
#define ncp5623_REG_ILED 		0b00100000
#define ncp5623_REG_PWM1		0b01000000
#define ncp5623_REG_PWM2		0b01100000
#define ncp5623_REG_PWM3 		0b10000000
#define ncp5623_REG_DIMUP 		0b10100000
#define ncp5623_REG_DIMDOWN		0b11000000
#define ncp5623_REG_DIMSTEP		0b11100000


typedef enum {ncp5623_LED1, ncp5623_LED2, ncp5623_LED3} ncp5623_lednum_e;
typedef enum {ncp5623_DIMUP, ncp5623_DIMDOWN} ncp5623_dimdirection_e;

#define ncp5623_TIME128		0b10000
#define ncp5623_TIME64		0b1000
#define ncp5623_TIME32		0b100
#define ncp5623_TIME16		0b10
#define ncp5623_TIME8		0b1


/*
 *
 * @red:0-31
 * @green:0-31
 * @blue:0-31
 */
void ncp5623_setRGB(uint8_t red,uint8_t green,uint8_t blue);

/** Set Dimming mode for all LEDs
 *  @dimdir - direction of dimming
 *  @endstep - ending step of ramp up or ramp down range 0-31
 *  @steptime - time per step range 0-31 in 8 msec multiples
 *  @return status of command
 */
void ncp5623_setDimming(ncp5623_dimdirection_e dimdir, uint8_t endstep, uint8_t ncp5623_TIME);

/** Set PWM mode for specific LED
 *  @lednum - selects LED
 *  @data - PWM value to set  range 0-31 0-100% Pulse width
 *  @return status of command
 */
void ncp5623_setPWM(ncp5623_lednum_e lednum, uint8_t data );

/** Set static LED Current
 *  @data - value of current draw for all LEDs range 0-31
 *  @return status of command
 */
void ncp5623_setLEDCurrent(uint8_t data);

void ncp5623_init();

/** Set Dimming mode for all LEDs, steptime is the same as set at setDimming function
     *  @endstep - ending step of ramp up or ramp down range 0-31 (0 total off, 30-31 same current)
     *  @return status of command
     */
void ncp5623_setDimmingAuto(uint8_t endstep);




#endif /* NCP5623_H_ */
