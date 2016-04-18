/*
 * HX8357.h
 *
 *  Created on: 2015.05.16.
 *      Author: A
 */

#ifndef HX8357_H_
#define HX8357_H_
#include "main.h"

#define serial_if
#undef serial_if
#define serial_if_3colorfill
#undef serial_if_3colorfill

#define display_RESET_port GPIOB
#define display_RESET_pin 	GPIO_PIN_6

#define display_SCK_port GPIOB
#define display_SCK_pin 	GPIO_PIN_5

#define display_SDA_port GPIOB
#define display_SDA_pin 	GPIO_PIN_7

#define display_CSX_port GPIOD
#define display_CSX_pin 	GPIO_PIN_7



void bitbang_write(unsigned char isCommand,unsigned char dat);
uint8_t bitbang_read8(uint8_t dummyclock);
void spi_write_com(unsigned char cmd);
void spi_write_dat(unsigned char dat);
int spi_read8(unsigned char cmd);
void bitbang_read8More(unsigned char isCommand,unsigned char dat);
void display_init(uint orientation);
void display_gpio_init(void);
void display_set_reset(uint8_t state);
void display_set_sck(uint8_t state);
void display_set_csx(uint8_t state);
void display_set_sda_dir(uint8_t dir);
void display_set_sda(uint8_t state);
uint8_t display_read_sda(void);

uint32_t HX8357_read_register_u32(uint8_t dummyclk, uint8_t reg_add);
uint32_t HX8357_read_register_n(uint8_t dummyclk, uint8_t reg_add, uint8_t count);
uint32_t HX8357_read_register2_n(uint8_t reg_add, uint8_t count);
uint8_t HX8357_regdump(void);


#endif /* HX8357_H_ */
