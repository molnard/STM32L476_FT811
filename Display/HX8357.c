/*
 * HX8357.c
 *
 *  Created on: 2015.05.16.
 *      Author: A
 */

#include "HX8357.h"
#include "main.h"

#define display_wait(x) delay(x)

#define HX8357_SWRESET			0x01
#define HX8357_GET_DISPLAYID		0x04
#define HX8357_GET_RED_CHANNEL		0x06
#define HX8357_GET_GREEN_CHANNEL	0x07
#define HX8357_GET_BLUE_CHANNEL		0x08
#define HX8357_GET_POWER_MODE		0x0a
#define HX8357_GET_MADCTL		0x0b
#define HX8357_GET_PIXEL_FORMAT		0x0c
#define HX8357_GET_DISPLAY_MODE		0x0d
#define HX8357_GET_SIGNAL_MODE		0x0e
#define HX8357_GET_DIAGNOSTIC_RESULT	0x0f
#define HX8357_ENTER_SLEEP_MODE		0x10
#define HX8357_EXIT_SLEEP_MODE		0x11
#define HX8357_ENTER_PARTIAL_MODE	0x12
#define HX8357_ENTER_NORMAL_MODE	0x13
#define HX8357_EXIT_INVERSION_MODE	0x20
#define HX8357_ENTER_INVERSION_MODE	0x21
#define HX8357_SET_DISPLAY_OFF		0x28
#define HX8357_SET_DISPLAY_ON		0x29
#define HX8357_SET_COLUMN_ADDRESS	0x2a
#define HX8357_SET_PAGE_ADDRESS		0x2b
#define HX8357_WRITE_MEMORY_START	0x2c
#define HX8357_READ_MEMORY_START	0x2e
#define HX8357_SET_PARTIAL_AREA		0x30
#define HX8357_SET_SCROLL_AREA		0x33
#define HX8357_SET_TEAR_OFF		0x34
#define HX8357_SET_TEAR_ON		0x35
#define HX8357_SET_ADDRESS_MODE		0x36
#define HX8357_SET_SCROLL_START		0x37
#define HX8357_EXIT_IDLE_MODE		0x38
#define HX8357_ENTER_IDLE_MODE		0x39
#define HX8357_SET_PIXEL_FORMAT		0x3a
#define HX8357_SET_PIXEL_FORMAT_DBI_3BIT	(0x1)
#define HX8357_SET_PIXEL_FORMAT_DBI_16BIT	(0x5)
#define HX8357_SET_PIXEL_FORMAT_DBI_18BIT	(0x6)
#define HX8357_SET_PIXEL_FORMAT_DPI_3BIT	(0x1 << 4)
#define HX8357_SET_PIXEL_FORMAT_DPI_16BIT	(0x5 << 4)
#define HX8357_SET_PIXEL_FORMAT_DPI_18BIT	(0x6 << 4)
#define HX8357_WRITE_MEMORY_CONTINUE	0x3c
#define HX8357_READ_MEMORY_CONTINUE	0x3e
#define HX8357_SET_TEAR_SCAN_LINES	0x44
#define HX8357_GET_SCAN_LINES		0x45
#define HX8357_READ_DDB_START		0xa1
#define HX8357_SET_DISPLAY_MODE		0xb4
#define HX8357_SET_DISPLAY_MODE_RGB_THROUGH	(0x3)
#define HX8357_SET_DISPLAY_MODE_RGB_INTERFACE	(1 << 4)
#define HX8357_SET_PANEL_DRIVING	0xc0
#define HX8357_SET_DISPLAY_FRAME	0xc5
#define HX8357_SET_RGB			0xc6
#define HX8357_SET_RGB_ENABLE_HIGH		(1 << 1)
#define HX8357_SET_GAMMA		0xc8
#define HX8357_SET_POWER		0xd0
#define HX8357_SET_VCOM			0xd1
#define HX8357_SET_POWER_NORMAL		0xd2
#define HX8357_SET_PANEL_RELATED	0xe9



void waitCycle(uint32_t cycle)
{
	uint32_t i=0;
	for (i = 0; i < cycle; ++i)
	{
		//IWDG_ReloadCounter();
		__NOP();
	}
}

void display_gpio_init(void)
{
	GPIO_InitTypeDef  GPIO_InitStruct;

	__GPIOA_CLK_ENABLE();
	__GPIOB_CLK_ENABLE();
	__GPIOC_CLK_ENABLE();
	/* Configure the display_RESET pin */
	GPIO_InitStruct.Pin = display_RESET_pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
	HAL_GPIO_Init(display_RESET_port, &GPIO_InitStruct);
	display_set_reset(1);

	//SCK
	GPIO_InitStruct.Pin = display_SCK_pin ;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(display_SCK_port , &GPIO_InitStruct);
	display_set_sck(0);

	//CSX
	GPIO_InitStruct.Pin = display_CSX_pin ;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
	HAL_GPIO_Init(display_CSX_port , &GPIO_InitStruct);
	display_set_csx(1);

	//SDA as input
	display_set_sda_dir(0);
}

void display_set_reset(uint8_t state)
{
	HAL_GPIO_WritePin(display_RESET_port, display_RESET_pin, state);
}

void display_set_sck(uint8_t state)
{
	HAL_GPIO_WritePin(display_SCK_port, display_SCK_pin, state);
	waitCycle(10);
}

void display_set_csx(uint8_t state)
{
	HAL_GPIO_WritePin(display_CSX_port, display_CSX_pin, state);
	waitCycle(20);
}

void display_set_sda_dir(uint8_t dir)
{
	GPIO_InitTypeDef  GPIO_InitStruct;
	GPIO_InitStruct.Pin = display_SDA_pin ;
	if(dir == 1)
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	else
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;

	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	//GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(display_SDA_port , &GPIO_InitStruct);

}

void display_set_sda(uint8_t state)
{
	HAL_GPIO_WritePin(display_SDA_port, display_SDA_pin, state);
}

uint8_t display_read_sda(void)
{
	return HAL_GPIO_ReadPin(display_SDA_port, display_SDA_pin);
}

//DBI Type-C Interface Protocol - option 1 (3 wire)
void bitbang_write(unsigned char isCommand,unsigned char dat)
{
	//CONTROL Chip select before this !!!
	//init phase
	display_set_sck(0);
	display_set_sda_dir(1);

	//start BANG!
	uint16_t bits=dat;
	if (isCommand==0) bits|=1<<8;
	for (int i=8;i>=0;i--)
	{
		display_set_sck(0);
		//waitCycle(10);
		unsigned char value=((bits>>i) & 0x01)==1;
		if (value) display_set_sda(1);
		else display_set_sda(0);
		//waitCycle(10);
		display_set_sck(1);
	}
	display_set_sda(0);
	display_set_sck(0);
}
//DBI Type-C Interface Protocol - option 1 (3 wire)
uint8_t bitbang_read8(uint8_t dummyclock)
{
	//CONTROL Chip select before this !!!
	display_set_sda_dir(0);
	uint8_t bits=0;
	if (dummyclock!=0)
	{
		display_set_sck(0);
		waitCycle(1);
		display_set_sck(1);
	}
	for (int i=7;i>=0;i--)
	{
		display_set_sck(0);
		waitCycle(1);
		bits |= display_read_sda() << i;
		display_set_sck(1);
	}
	display_set_sck(0);
	return bits;
}

// write command to tft register
//first bit of nine bits is LOW = COMMAND!
void spi_write_com(unsigned char cmd)
{
	bitbang_write(1,cmd);
}
//first bit of nine bits is HIGH = DATA!
void spi_write_dat(unsigned char dat)
{
	bitbang_write(0,dat);
}

int spi_read8(unsigned char cmd)
{
	bitbang_write(1,cmd);
	uint8_t rd=bitbang_read8(0); //dummy
	rd=bitbang_read8(0);
	rd=bitbang_read8(0);
	return rd;
}


//DBI Type-C Interface Protocol - option 1 (3 wire)
void bitbang_read8More(unsigned char isCommand,unsigned char dat)
{
	//CONTROL Chip select before this !!!

	//NOT IMPLEMENTED!
}

uint32_t HX8357_read_register_u32(uint8_t dummyclk, uint8_t reg_add)
{
	uint8_t rd0=0,rd1=0, rd2=0,rd3=0;
	//while(1)
	{
		display_set_csx(0);
		bitbang_write(1,reg_add);
		if(dummyclk == 1)
			rd3 = bitbang_read8(1);
		else
			rd3 = bitbang_read8(0);
		rd2 = bitbang_read8(0);
		rd1 = bitbang_read8(0);
		rd0 = bitbang_read8(0);
		display_set_csx(1);
	}
	//printf("HX8357_read(0x%x): 0x%.2x 0x%.2x 0x%.2x 0x%.2x\r\n",reg_add,rd3,rd2,rd1,rd0);
	printf("HX8357_read(0x%x): 0x%x 0x%x 0x%x 0x%x\r\n",reg_add,rd3,rd2,rd1,rd0);
	return (rd3<<24) | (rd2<<16) | (rd1<<8) | rd0;
}

uint8_t HX8357_regdump(void)
{
	int i = 0;
	for (i = 0; i < 255; ++i)
	{
		HX8357_read_register_u32(0,i);
		display_wait(100);
	}
	return 0;
}

uint32_t HX8357_read_register_n(uint8_t dummyclk, uint8_t reg_add, uint8_t count)
{
	uint8_t rd[256] = {0};
	int i = 0 ;
	//while(1)
	{
		display_set_csx(0);
		bitbang_write(1,reg_add);
		if(dummyclk == 1)
			rd[0] = bitbang_read8(1);
		else
			rd[0] = bitbang_read8(0);
		for (i = 0; i < count-1; ++i)
		{
			rd[i+1] = bitbang_read8(0);
		}
		display_set_csx(1);
	}
	printf("HX8357_read(0x%x): %d parameter\r\n",reg_add,count);
	uint8_t crlf = 0;
	for (i = 0; i < count; ++i)
	{
		crlf = 0;
		printf("0x%x ",rd[i]);
		if((i%15==0)&&(i>0))
		{
			printf("\r\n");
			crlf = 1;
		}
	}
	if(crlf == 0)
		printf("\r\n");
	return 0;
}

// spi_write_com(0xB9); 	// Set EXTC

uint32_t HX8357_read_register2_n(uint8_t reg_add, uint8_t count)
{
	uint8_t rd[256] = {0};
	// SETREADINDEX: set SPI read index (FEh)
	printf("HX8357_read(0x%x): %d parameter\r\n",reg_add,count);
	display_set_csx(0);
	spi_write_com(0xFE);
	spi_write_dat(reg_add);
	display_wait(1);

	//Read SPI Command Data for User Define Command
	int i = 0;
	uint8_t crlf = 0;
	spi_write_com(0xFF);
	for (i = 0; i < count; ++i)
	{
		rd[i] = bitbang_read8(0);
		printf("0x%x ",rd[i]);
		if((i%15==0)&&(i>0))
		{
			printf("\r\n");
			crlf = 1;
		}
	}
	display_set_csx(1);
	if(crlf == 0)
		printf("\r\n");
	return rd[0];
}




void display_init(uint orientation)
{
	display_gpio_init();
	display_set_reset(1); //active low!
	display_wait(10);
	display_set_reset(0);
	display_wait(50);
	display_set_reset(1);
	display_wait(120);

	int rd1=0;
	int rd2=0;
	int rd3=0;
	int i = 0;
	//while(rd2!=0x80)
	while(1)
	{
		display_set_csx(0);
		bitbang_write(1,0x04);
		rd1 = bitbang_read8(1);
		rd2 = bitbang_read8(0);
		rd3 = bitbang_read8(0);
		//		bitbang_write(1,0xDB);
		//		//rd1 = bitbang_read8(1);
		//		rd2 = bitbang_read8(0);
		//		//rd3 = bitbang_read8(0);
		waitCycle(40);
		display_set_csx(1);
		display_set_sda_dir(1);
		display_set_sda(0);
		//printf("%d. Himax: %x %x %x\r\n",i,rd1,rd2,rd3);
		display_wait(1);
		i++;
		if(rd2==0x80)
		{
			printf("Display found: HX8357D id:%x-%x-%x\r\n",rd1,rd2,rd3);
			break;
		}
		if(i>5)
		{
			printf("Display not found\r\n");
			return;
		}
	}

	//uint32_t reg320 = HX8357_read_register_u32(1,0x09);
	//HX8357_read_register2_n(0x2a,4);
	//	while(1)
	//	{
	//		wait_us(1);
	//		display_set_csx(0);
	//		wait_us(1);
	//	    bitbang_write(1,0xd0);
	//	    wait_us(1);
	//	    rd1=bitbang_read8();
	//	    wait_us(1);
	//	    rd2=bitbang_read8();
	//	    wait_us(1);
	//		display_set_csx(1);
	//		pc.printf("himaxextid: %x %x\r\n",rd1,rd2);
	//		wait(0.3);
	//	}

	//	display_set_reset(1); //active low!
	//	display_wait(10);
	//	display_set_reset(0);
	//	display_wait(50);
	//	display_set_reset(1);
	//	display_wait(120);

	uint8_t readback = 0;
	uint8_t delay = 1;


	display_set_csx(0);
	spi_write_com(0xB9); 	// Set EXTC
	spi_write_dat(0xFF);
	spi_write_dat(0x83);
	spi_write_dat(0x57);
	display_set_csx(1);

	if(delay==1)display_wait(5);
	if(readback==1)HX8357_read_register_n(0,0xB9,3);

	//display_wait(5);

	display_set_csx(0);
	spi_write_com(0xB1); //SETPOWER: set power control
	spi_write_dat(0x00);
	spi_write_dat(0x12);
	spi_write_dat(0x18);//1D
	spi_write_dat(0x18);//1D
	spi_write_dat(0xC3);
	spi_write_dat(0x31);//FS
	display_set_csx(1);
	if(readback==1)HX8357_read_register_n(0,0xB1,6);
	//display_wait(50);
	display_set_csx(0);
	spi_write_com(0xB3); //SETRGB: set RGB interface
#ifdef serial_if
	spi_write_dat(0x02);
#else
	spi_write_dat(0x43); //DPI Interface (RGB) ++ CLK from DPI interface ++ BYPASS direct to display in RGB interface.
#endif
	spi_write_dat(0x00);
	spi_write_dat(0x06);
	spi_write_dat(0x06); //VPL[5:0]
	display_set_csx(1);
	if(readback==1)HX8357_read_register_n(0,0xB3,4);
	//display_wait(50);
	display_set_csx(0);
	spi_write_com(0xB4); //SETCYC: set display cycle register (B4h)
	spi_write_dat(0x02);
	spi_write_dat(0x40);
	spi_write_dat(0x00);
	spi_write_dat(0x2A);
	spi_write_dat(0x2A);
	spi_write_dat(0x35);
	spi_write_dat(0x4E); //GDOFF
	display_set_csx(1);
	if(readback==1)HX8357_read_register_n(0,0xB4,7);
	//display_wait(50);
	display_set_csx(0);
	spi_write_com(0xB5);
	spi_write_dat(0x06);//08
	spi_write_dat(0x06);//08
	display_set_csx(1);
	if(readback==1)HX8357_read_register_n(0,0xB5,2);
	//display_wait(50);
	display_set_csx(0);
	spi_write_com(0xB6);		//
	spi_write_dat(0x37);		//VCOMDC 54H
	display_set_csx(1);
	if(readback==1)HX8357_read_register_n(0,0xB6,1);
	//display_wait(50);
	display_set_csx(0);
	spi_write_com(0xC0);
	spi_write_dat(0x20);
	spi_write_dat(0x20);
	spi_write_dat(0x00);
	spi_write_dat(0x08);
	spi_write_dat(0x1C);
	spi_write_dat(0x08);
	display_set_csx(1);
	if(readback==1)HX8357_read_register_n(0,0xC0,6);
	//display_wait(50);
	display_set_csx(0);
	spi_write_com(0xE3);
	spi_write_dat(0x1B);
	spi_write_dat(0x1B);
	display_set_csx(1);
	if(readback==1)HX8357_read_register_n(0,0xE3,2);
	//display_wait(50);
	display_set_csx(0);
	spi_write_com(0xB6);
	spi_write_dat(0x10);
	if(readback==1)HX8357_read_register_n(0,0xB6,1);

	display_set_csx(1);
	//display_wait(50);
	display_set_csx(0);
	spi_write_com(0xC2);
	spi_write_dat(0x00);
	spi_write_dat(0x04);
	spi_write_dat(0x04);
	display_set_csx(1);
	if(readback==1)HX8357_read_register_n(0,0xC2,3);
	//display_wait(50);
	display_set_csx(0);
	spi_write_com(0xCC);		//Set Panel
	if(orientation == 1)
		spi_write_dat(0x07);
	else
		spi_write_dat(0x03);		//BGR_Panel
		//spi_write_dat(0b00011011);

	display_set_csx(1);
	if(readback==1)HX8357_read_register_n(0,0xCC,1);
	//display_wait(50);
	display_set_csx(0);
	spi_write_com(0xE0);
	spi_write_dat(0x00);
	spi_write_dat(0x06);
	spi_write_dat(0x0B);
	spi_write_dat(0x12);
	spi_write_dat(0x18);
	spi_write_dat(0x28);
	spi_write_dat(0x36);
	spi_write_dat(0x41);
	spi_write_dat(0x53);
	spi_write_dat(0x48);
	spi_write_dat(0x3A);
	spi_write_dat(0x27);
	spi_write_dat(0x17);
	spi_write_dat(0x08);
	spi_write_dat(0x01);
	spi_write_dat(0x00);

	spi_write_dat(0x00);
	spi_write_dat(0x06);
	spi_write_dat(0x0B);
	spi_write_dat(0x12);
	spi_write_dat(0x18);
	spi_write_dat(0x28);
	spi_write_dat(0x36);
	spi_write_dat(0x41);
	spi_write_dat(0x53);
	spi_write_dat(0x48);
	spi_write_dat(0x3A);
	spi_write_dat(0x27);
	spi_write_dat(0x17);
	spi_write_dat(0x08);
	spi_write_dat(0x01);
	spi_write_dat(0x00);
	spi_write_dat(0x00);
	spi_write_dat(0x01);

	display_set_csx(1);
	if(readback==1)HX8357_read_register_n(0,0xE0,26);
	//display_wait(50);
	display_set_csx(0);
	spi_write_com(0xC1);
	spi_write_dat(0x01);
	//R
	spi_write_dat(0x00);
	spi_write_dat(0x0F);
	spi_write_dat(0x15);
	spi_write_dat(0x1A);
	spi_write_dat(0x21);
	spi_write_dat(0x29);
	spi_write_dat(0x31);
	spi_write_dat(0x38);
	spi_write_dat(0x3F);
	spi_write_dat(0x46);
	spi_write_dat(0x4C);
	spi_write_dat(0x53);
	spi_write_dat(0x5B);
	spi_write_dat(0x62);
	spi_write_dat(0x6A);
	spi_write_dat(0x71);
	spi_write_dat(0x79);
	spi_write_dat(0x80);
	spi_write_dat(0x87);
	spi_write_dat(0x8F);
	spi_write_dat(0x96);
	spi_write_dat(0x9E);
	spi_write_dat(0xA5);
	spi_write_dat(0xAD);
	spi_write_dat(0xB6);
	spi_write_dat(0xBD);
	spi_write_dat(0xC3);
	spi_write_dat(0xC9);
	spi_write_dat(0xD2);
	spi_write_dat(0xDA);
	spi_write_dat(0xE1);
	spi_write_dat(0xE8);
	spi_write_dat(0xF0);

	//G
	spi_write_dat(0x00);
	spi_write_dat(0x0F);
	spi_write_dat(0x16);
	spi_write_dat(0x1B);
	spi_write_dat(0x23);
	spi_write_dat(0x2B);
	spi_write_dat(0x33);
	spi_write_dat(0x3B);
	spi_write_dat(0x42);
	spi_write_dat(0x49);
	spi_write_dat(0x4F);
	spi_write_dat(0x57);
	spi_write_dat(0x60);
	spi_write_dat(0x67);
	spi_write_dat(0x6F);
	spi_write_dat(0x77);
	spi_write_dat(0x7F);
	spi_write_dat(0x87);
	spi_write_dat(0x8E);
	spi_write_dat(0x96);
	spi_write_dat(0x9E);
	spi_write_dat(0xA6);
	spi_write_dat(0xAE);
	spi_write_dat(0xB7);
	spi_write_dat(0xBE);
	spi_write_dat(0xC5);
	spi_write_dat(0xCC);
	spi_write_dat(0xD6);
	spi_write_dat(0xDD);
	spi_write_dat(0xE4);
	spi_write_dat(0xED);
	spi_write_dat(0xF5);
	spi_write_dat(0xFF);

	//B
	spi_write_dat(0x00);
	spi_write_dat(0x0F);
	spi_write_dat(0x16);
	spi_write_dat(0x1B);
	spi_write_dat(0x23);
	spi_write_dat(0x2B);
	spi_write_dat(0x33);
	spi_write_dat(0x3B);
	spi_write_dat(0x42);
	spi_write_dat(0x49);
	spi_write_dat(0x4F);
	spi_write_dat(0x57);
	spi_write_dat(0x60);
	spi_write_dat(0x67);
	spi_write_dat(0x6F);
	spi_write_dat(0x77);
	spi_write_dat(0x7F);
	spi_write_dat(0x87);
	spi_write_dat(0x8E);
	spi_write_dat(0x96);
	spi_write_dat(0x9E);
	spi_write_dat(0xA6);
	spi_write_dat(0xAE);
	spi_write_dat(0xB7);
	spi_write_dat(0xBE);
	spi_write_dat(0xC5);
	spi_write_dat(0xCC);
	spi_write_dat(0xD6);
	spi_write_dat(0xDD);
	spi_write_dat(0xE4);
	spi_write_dat(0xED);
	spi_write_dat(0xF5);
	spi_write_dat(0xFF);
	display_set_csx(1);
	if(readback==1)HX8357_read_register_n(0,0xC1,63);
	//display_wait(50);
	display_set_csx(0);
	spi_write_com(0x3A);
#ifdef serial_if
	spi_write_dat(0x55);
#else
	spi_write_dat(0x60);
#endif
	display_set_csx(1);
	if(readback==1)HX8357_read_register_n(0,0x3A,2);
	//display_wait(50);
	display_set_csx(0);
	spi_write_com(0x36);
	spi_write_dat(0x40);
	display_set_csx(1);
	if(readback==1)HX8357_read_register_n(0,0x36,2);
	//display_wait(50);
	display_set_csx(0);
	spi_write_com(0x11);
	display_set_csx(1);
	if(delay==1)display_wait(500);//delay 500ms

	display_set_csx(0);
	spi_write_com(0x29);
	display_set_csx(1);
	if(delay==1)display_wait(10);
//	display_set_csx(0);
//
//		spi_write_com(0x2A);
//		spi_write_dat(0x01);
//		spi_write_dat(0x3f);
//		spi_write_dat(0x01);
//		spi_write_dat(0xdf);
//
//	spi_write_com(0x2C);
//		spi_write_com(0x13);
//		spi_write_com(0x23);
//		spi_write_com(0x21);
//	display_set_csx(1);



	//read GETICID: IC ID Read Command Data (D0h)
	i = 0;
	//	while(1)
	//	{
	//		display_set_csx(0);
	//		spi_write_com(0xB9); 	// Set EXTC
	//		spi_write_dat(0xAA);
	//		spi_write_dat(0x55);
	//		spi_write_dat(0xAA);
	//		wait(50);
	//		display_set_csx(1);
	//		wait(50);
	//		display_set_csx(0);
	//		//bitbang_write(1,0xdb);
	//		//bitbang_write(1,0xb9);
	//		//bitbang_write(1,0xdb);
	//		bitbang_write(1,0x09);
	//		rd1 = bitbang_read8(1);
	//		rd2 = bitbang_read8(0);
	//		rd3 = bitbang_read8(0);
	//		//		bitbang_write(1,0xDB);
	//		//		//rd1 = bitbang_read8(1);
	//		//		rd2 = bitbang_read8(0);
	//		//		//rd3 = bitbang_read8(0);
	//		wait(40);
	//		display_set_csx(1);
	//		display_set_sda_dir(1);
	//		display_set_sda(0);
	//		printf("\r%d. b9h: %x %x %x",i,rd1,rd2,rd3);
	//		while(1)
	//		{
	//			display_wait(1);
	//			printf("%d. ",i);
	//printf("Before: 0x%x : %s %s %s %s\r\n",reg320,byte_to_binary((reg320>>24)&0xff),byte_to_binary(reg320>>16),byte_to_binary((reg320>>8)),byte_to_binary(reg320));
	//	uint32_t reg32 = HX8357_read_register_u32(1,0x09);
	//	printf("After 0x%x : %s\r\n",reg32,u32_to_binary(reg32));
	//
	//	HX8357_read_register_n(1,0x09,9);

	//			//HX8357_read_register_u32(0,0xb9);
	//			i++;
	//		}
	//			if(rd2==0x80)break;
	//			if(i>5)
	//				{
	//				printf("Display not found\r\n");
	//				return;
	//				}
	//	}

#ifdef serial_if
#ifdef serial_if_3colorfill
	display_set_csx(0);
	spi_write_com(0x3C);
	for (int i=0;i<320*160;i++)
	{
		spi_write_dat(0b11111000);
		spi_write_dat(0b00000000);
	}
	for (int i=0;i<320*160;i++)
	{
		spi_write_dat(0b00000111);
		spi_write_dat(0b11100000);
	}
	for (int i=0;i<320*160;i++)
	{
		spi_write_dat(0b00000000);
		spi_write_dat(0b00011111);
	}
	display_set_csx(1);
#endif
#endif
	//
	//	}
}




