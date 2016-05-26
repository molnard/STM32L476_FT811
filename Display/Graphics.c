/*
 * Graphics.c
 *
 *  Created on: Apr 8, 2016
 *      Author: molnard
 */

#include <Graphics.h>


#define SAMAPP_DELAY_BTW_APIS (1000)
#define SAMAPP_ENABLE_DELAY() Ft_Gpu_Hal_Sleep(SAMAPP_DELAY_BTW_APIS)
#define SAMAPP_ENABLE_DELAY_VALUE(x) Ft_Gpu_Hal_Sleep(x)


ft_void_t graphics_BootupConfig();

/* Global variables for display resolution to support various display panels */
/* Default is WQVGA - 480x272 */
ft_int16_t FT_DispWidth = 480;
ft_int16_t FT_DispHeight = 272;
ft_int16_t FT_DispHCycle = 548;
ft_int16_t FT_DispHOffset = 43;
ft_int16_t FT_DispHSync0 = 0;
ft_int16_t FT_DispHSync1 = 41;
ft_int16_t FT_DispVCycle = 292;
ft_int16_t FT_DispVOffset = 12;
ft_int16_t FT_DispVSync0 = 0;
ft_int16_t FT_DispVSync1 = 10;
ft_uint8_t FT_DispPCLK = 5;
ft_char8_t FT_DispSwizzle = 0;
ft_char8_t FT_DispPCLKPol = 1;
ft_char8_t FT_DispCSpread = 1;
ft_char8_t FT_DispDither = 1;

/* Global used for buffer optimization */
Ft_Gpu_Hal_Context_t host,*phost;

ft_uint32_t Ft_CmdBuffer_Index;
ft_uint32_t Ft_DlBuffer_Index;

#ifdef BUFFER_OPTIMIZATION
ft_uint8_t  Ft_DlBuffer[FT_DL_SIZE];
ft_uint8_t  Ft_CmdBuffer[FT_CMD_FIFO_SIZE];
#endif

/* Boot up for FT800 followed by graphics primitive sample cases */
/* Initial boot up DL - make the back ground green color */
const ft_uint8_t FT_DLCODE_BOOTUP[12] =
{
    0,0,0,2,//GPU instruction CLEAR_COLOR_RGB
    7,0,0,38, //GPU instruction CLEAR
    0,0,0,0,  //GPU instruction DISPLAY
};

/* Boot up for FT800 followed by graphics primitive sample cases */
/* Initial boot up DL - make the back ground green color */
ft_void_t Ft_App_WrCoCmd_Buffer(Ft_Gpu_Hal_Context_t *phost,ft_uint32_t cmd)
{
#ifdef  BUFFER_OPTIMIZATION
   /* Copy the command instruction into buffer */
   ft_uint32_t *pBuffcmd;
   pBuffcmd =(ft_uint32_t*)&Ft_CmdBuffer[Ft_CmdBuffer_Index];
   *pBuffcmd = cmd;
#endif

   /* Increment the command index */
   Ft_CmdBuffer_Index += FT_CMD_SIZE;
}

ft_void_t Ft_App_WrDlCmd_Buffer(Ft_Gpu_Hal_Context_t *phost,ft_uint32_t cmd)
{
#ifdef BUFFER_OPTIMIZATION
   /* Copy the command instruction into buffer */
   ft_uint32_t *pBuffcmd;
   pBuffcmd =(ft_uint32_t*)&Ft_DlBuffer[Ft_DlBuffer_Index];
   *pBuffcmd = cmd;
#endif

   Ft_Gpu_Hal_Wr32(phost,(RAM_DL+Ft_DlBuffer_Index),cmd);
   /* Increment the command index */
   Ft_DlBuffer_Index += FT_CMD_SIZE;
}

ft_void_t Ft_App_WrCoStr_Buffer(Ft_Gpu_Hal_Context_t *phost,const ft_char8_t *s)
{
#ifdef  BUFFER_OPTIMIZATION
  ft_uint16_t length = 0;
  length = strlen(s) + 1;//last for the null termination

  strcpy(&Ft_CmdBuffer[Ft_CmdBuffer_Index],s);

  /* increment the length and align it by 4 bytes */
  Ft_CmdBuffer_Index += ((length + 3) & ~3);
#endif
}

ft_void_t Ft_App_Flush_DL_Buffer(Ft_Gpu_Hal_Context_t *phost)
{
#ifdef  BUFFER_OPTIMIZATION
   if (Ft_DlBuffer_Index > 0)
     Ft_Gpu_Hal_WrMem(phost,RAM_DL,Ft_DlBuffer,Ft_DlBuffer_Index);
#endif
   Ft_DlBuffer_Index = 0;

}

ft_void_t Ft_App_Flush_Co_Buffer(Ft_Gpu_Hal_Context_t *phost)
{
#ifdef  BUFFER_OPTIMIZATION
   if (Ft_CmdBuffer_Index > 0)
     Ft_Gpu_Hal_WrCmdBuf(phost,Ft_CmdBuffer,Ft_CmdBuffer_Index);
#endif
   Ft_CmdBuffer_Index = 0;
}


/* API to give fadeout effect by changing the display PWM from 100 till 0 */
ft_void_t SAMAPP_fadeout()
{
//   ft_int32_t i;
//
//	for (i = 100; i >= 0; i -= 3)
//	{
//		Ft_Gpu_Hal_Wr8(phost,REG_PWM_DUTY,i);
//
//		Ft_Gpu_Hal_Sleep(2);//sleep for 2 ms
//	}
}

/* API to perform display fadein effect by changing the display PWM from 0 till 100 and finally 128 */
ft_void_t SAMAPP_fadein()
{
//	ft_int32_t i;
//
//	for (i = 0; i <=100 ; i += 3)
//	{
//		Ft_Gpu_Hal_Wr8(phost,REG_PWM_DUTY,i);
//		Ft_Gpu_Hal_Sleep(2);//sleep for 2 ms
//	}
//	/* Finally make the PWM 100% */
//	i = 128;
//	Ft_Gpu_Hal_Wr8(phost,REG_PWM_DUTY,i);
}


/* API to check the status of previous DLSWAP and perform DLSWAP of new DL */
/* Check for the status of previous DLSWAP and if still not done wait for few ms and check again */
ft_void_t SAMAPP_GPU_DLSwap(ft_uint8_t DL_Swap_Type)
{
	ft_uint8_t Swap_Type = DLSWAP_FRAME,Swap_Done = DLSWAP_FRAME;

	if(DL_Swap_Type == DLSWAP_LINE)
	{
		Swap_Type = DLSWAP_LINE;
	}

	/* Perform a new DL swap */
	Ft_Gpu_Hal_Wr8(phost,REG_DLSWAP,Swap_Type);

	/* Wait till the swap is done */
	while(Swap_Done)
	{
		Swap_Done = Ft_Gpu_Hal_Rd8(phost,REG_DLSWAP);

		if(DLSWAP_DONE != Swap_Done)
		{
			Ft_Gpu_Hal_Sleep(10);//wait for 10ms
		}
	}
}

ft_void_t graphics_init()
{

	Ft_Gpu_Hal_Init(0);
	Ft_Gpu_Hal_Open(&host);
	phost = &host;
	graphics_BootupConfig();
	Touch_LoadCalibration();
}


ft_void_t graphics_BootupConfig()
{

	Ft_Gpu_Hal_Powercycle(phost,FT_TRUE);

	/* Access address 0 to wake up the FT800 */
	Ft_Gpu_HostCommand(phost,FT_GPU_ACTIVE_M);
	Ft_Gpu_Hal_Sleep(20);

		/* Set the clk to external clock */
#if (!defined(ME800A_HV35R) && !defined(ME810A_HV35R))
		Ft_Gpu_HostCommand(phost,FT_GPU_EXTERNAL_OSC);
		Ft_Gpu_Hal_Sleep(10);
#endif
		Ft_Gpu_81X_SelectSysCLK(phost,FT_GPU_SYSCLK_48M);
		Ft_Gpu_CoreReset(phost);


		ft_uint8_t chipid;
		//Read Register ID to check if FT800 is ready.
		chipid = Ft_Gpu_Hal_Rd8(phost, REG_ID);
		while(chipid != 0x7C)
		{
			chipid = Ft_Gpu_Hal_Rd8(phost, REG_ID);
			ft_delay(100);
		}
	#if defined(MSVC_PLATFORM) || defined (FT900_PLATFORM)
			printf("VC1 register ID after wake up %x\n",chipid);
	#endif
		printf("FT811 found:%x\r\n",chipid);


	//Configure video timing registers, except REG_PCLK
	/* Configuration of LCD display */
#ifdef DISPLAY_RESOLUTION_QVGA
	/* Values specific to QVGA LCD display */
	FT_DispWidth = 320;
	FT_DispHeight = 240;
	FT_DispHCycle =  408;
	FT_DispHOffset = 70;
	FT_DispHSync0 = 0;
	FT_DispHSync1 = 10;
	FT_DispVCycle = 263;
	FT_DispVOffset = 13;
	FT_DispVSync0 = 0;
	FT_DispVSync1 = 2;
	FT_DispPCLK = 8;
	FT_DispSwizzle = 2;
	FT_DispPCLKPol = 0;
	FT_DispCSpread = 1;
	FT_DispDither = 1;

#endif
#ifdef DISPLAY_RESOLUTION_WVGA
	/* Values specific to QVGA LCD display */
	FT_DispWidth = 800;
	FT_DispHeight = 480;
	FT_DispHCycle =  928;
	FT_DispHOffset = 88;
	FT_DispHSync0 = 0;
	FT_DispHSync1 = 48;
	FT_DispVCycle = 525;
	FT_DispVOffset = 32;
	FT_DispVSync0 = 0;
	FT_DispVSync1 = 3;
	FT_DispPCLK = 2;
	FT_DispSwizzle = 0;
	FT_DispPCLKPol = 1;
	FT_DispCSpread = 0;
	FT_DispDither = 1;
#endif
#ifdef DISPLAY_RESOLUTION_HVGA_PORTRAIT
	/* Values specific to HVGA LCD display */

	FT_DispWidth = 320;
	FT_DispHeight = 480;
	FT_DispHCycle =  400;
	FT_DispHOffset = 40;
	FT_DispHSync0 = 0;
	FT_DispHSync1 = 10;
	FT_DispVCycle = 500;
	FT_DispVOffset = 10;
	FT_DispVSync0 = 0;
	FT_DispVSync1 = 5;
	FT_DispPCLK = 4;
	FT_DispSwizzle = 2;
	FT_DispPCLKPol = 1;
	FT_DispCSpread = 1;
	FT_DispDither = 1;

#ifdef ME810A_HV35R
	FT_DispPCLK = 5;
#endif
#endif

#ifdef DISPLAY_RESOLUTION_HVGA
	/* Values specific to HVGA LCD display */

	FT_DispWidth = 320;// Active width of LCD display
	FT_DispHeight = 480;// Active height of LCD display
	FT_DispHCycle =  335;// Total number of clocks per line
	FT_DispHOffset = 10;// Start of active line
	FT_DispHSync0 = 3;// Start of horizontal sync pulse
	FT_DispHSync1 = 6;// End of horizontal sync pulse
	FT_DispVCycle = 495;// Total number of lines per screen
	FT_DispVOffset = 10;// Start of active screen
	FT_DispVSync0 = 3;// Start of vertical sync pulse
	FT_DispVSync1 = 6;// End of vertical sync pulse
	FT_DispPCLK = 6;// Pixel Clock
	FT_DispSwizzle = 0;// Define RGB output pins
	FT_DispPCLKPol = 0;
	FT_DispCSpread = 0;
	FT_DispDither = 0;


#ifdef ME810A_HV35R
	FT_DispPCLK = 5;
#endif
#endif

#if (defined(ME800A_HV35R) || defined(ME810A_HV35R))
	/* After recognizing the type of chip, perform the trimming if necessary */
    Ft_Gpu_ClockTrimming(phost,LOW_FREQ_BOUND);
#endif


    Ft_Gpu_Hal_Wr16(phost, REG_HCYCLE, FT_DispHCycle);

	Ft_Gpu_Hal_Wr16(phost, REG_HOFFSET, FT_DispHOffset);
	Ft_Gpu_Hal_Wr16(phost, REG_HSYNC0, FT_DispHSync0);
	Ft_Gpu_Hal_Wr16(phost, REG_HSYNC1, FT_DispHSync1);
	Ft_Gpu_Hal_Wr16(phost, REG_VCYCLE, FT_DispVCycle);
	Ft_Gpu_Hal_Wr16(phost, REG_VOFFSET, FT_DispVOffset);
	Ft_Gpu_Hal_Wr16(phost, REG_VSYNC0, FT_DispVSync0);
	Ft_Gpu_Hal_Wr16(phost, REG_VSYNC1, FT_DispVSync1);
	Ft_Gpu_Hal_Wr8(phost, REG_SWIZZLE, FT_DispSwizzle);
	Ft_Gpu_Hal_Wr8(phost, REG_PCLK_POL, FT_DispPCLKPol);
	Ft_Gpu_Hal_Wr16(phost, REG_HSIZE, FT_DispWidth);
	Ft_Gpu_Hal_Wr16(phost, REG_VSIZE, FT_DispHeight);
	Ft_Gpu_Hal_Wr16(phost, REG_CSPREAD, FT_DispCSpread);
	Ft_Gpu_Hal_Wr16(phost, REG_DITHER, FT_DispDither);

#if (defined(FT_800_ENABLE) || defined(FT_810_ENABLE) ||defined(FT_812_ENABLE))
    /* Touch configuration - configure the resistance value to 1200 - this value is specific to customer requirement and derived by experiment */
    Ft_Gpu_Hal_Wr16(phost, REG_TOUCH_RZTHRESH,RESISTANCE_THRESHOLD);
#endif
    Ft_Gpu_Hal_Wr8(phost, REG_GPIO_DIR,0xff);
    Ft_Gpu_Hal_Wr8(phost, REG_GPIO,0xff);

    Ft_Gpu_Hal_Wr16(phost, REG_ROTATE,2);
    //we doing a trick to ensure good values when we want to reference the screen sizes => swap
    uint16_t temp=FT_DispWidth;
    FT_DispWidth = FT_DispHeight;
    FT_DispHeight = temp;
    //Write first display list
    /*It is optional to clear the screen here*/
    Ft_Gpu_Hal_WrMem(phost, RAM_DL,(ft_uint8_t *)FT_DLCODE_BOOTUP,sizeof(FT_DLCODE_BOOTUP));
    //Write REG_DLSWAP, FT81X swaps the display list immediately
    Ft_Gpu_Hal_Wr8(phost, REG_DLSWAP,DLSWAP_FRAME);
    //Enable back light control for display
    backlight_init();
    //Write REG_PCLK, video output begins with the first display list
    Ft_Gpu_Hal_Wr8(phost, REG_PCLK,FT_DispPCLK);//after this display is visible on the LCD
    ft_delay(120);

#ifdef ENABLE_ILI9488_HVGA_PORTRAIT
	/* to cross check reset pin */
	Ft_Gpu_Hal_Wr8(phost, REG_GPIO,0xff);
	ft_delay(120);
	Ft_Gpu_Hal_Wr8(phost, REG_GPIO,0x7f);
	ft_delay(120);
	Ft_Gpu_Hal_Wr8(phost, REG_GPIO,0xff);

	ILI9488_Bootup();

	/* Reconfigure the SPI */
#ifdef FT900_PLATFORM
	printf("after ILI9488 bootup \n");
	//spi
	// Initialize SPIM HW
	sys_enable(sys_device_spi_master);
	gpio_function(27, pad_spim_sck); /* GPIO27 to SPIM_CLK */
	gpio_function(28, pad_spim_ss0); /* GPIO28 as CS */
	gpio_function(29, pad_spim_mosi); /* GPIO29 to SPIM_MOSI */
	gpio_function(30, pad_spim_miso); /* GPIO30 to SPIM_MISO */

	gpio_write(28, 1);
	spi_init(SPIM, spi_dir_master, spi_mode_0, 4);
#endif

#endif

//Use an MCU SPI clock of not more than 30MHz - 80 Mhz base
	hqspi.Init.ClockPrescaler = 3;
	//hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_8_CYCLE;
	//HAL_QSPI_Init(&hqspi);

	/* make the spi to quad mode - addition 2 bytes for silicon */
#ifdef FT_81X_ENABLE
	/* api to set quad and numbe of dummy bytes */
#ifdef ENABLE_SPI_QUAD
	Ft_Gpu_Hal_SetSPI(phost,FT_GPU_SPI_QUAD_CHANNEL,FT_GPU_SPI_TWODUMMY);
#elif ENABLE_SPI_DUAL
	Ft_Gpu_Hal_SetSPI(phost,FT_GPU_SPI_DUAL_CHANNEL,FT_GPU_SPI_TWODUMMY);
#else
	Ft_Gpu_Hal_SetSPI(phost,FT_GPU_SPI_SINGLE_CHANNEL,FT_GPU_SPI_ONEDUMMY);
#endif

#endif

#ifdef FT900_PLATFORM
	spi_init(SPIM, spi_dir_master, spi_mode_0, 4);

#if (defined(ENABLE_SPI_QUAD))
    /* Initialize IO2 and IO3 pad/pin for dual and quad settings */
    gpio_function(31, pad_spim_io2);
    gpio_function(32, pad_spim_io3);
    gpio_write(31, 1);
    gpio_write(32, 1);
#endif

	spi_option(SPIM,spi_option_fifo_size,64);
	spi_option(SPIM,spi_option_fifo,1);
	spi_option(SPIM,spi_option_fifo_receive_trigger,1);

#ifdef ENABLE_SPI_QUAD
	spi_option(SPIM,spi_option_bus_width,4);
#elif ENABLE_SPI_DUAL
	spi_option(SPIM,spi_option_bus_width,2);
#else
	spi_option(SPIM,spi_option_bus_width,1);
#endif


#endif


	phost->ft_cmd_fifo_wp = Ft_Gpu_Hal_Rd16(phost,REG_CMD_WRITE);

	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_4,GPIO_PIN_SET); //touch screen wake up! (Warning! not inverted!)

	Ft_Gpu_Hal_Wr8(phost,REG_CTOUCH_EXTENDED, CTOUCH_MODE_COMPATIBILITY); //extended mode 5 touch point + gestures

	//we do not use interrupt instead we read flag register continuously
	/*Ft_Gpu_Hal_Wr32(phost,REG_INT_EN,1);
	Ft_Gpu_Hal_Wr32(phost,REG_INT_MASK,0b11111111);*/
}

ft_void_t	SAMAPP_GPU_Clear()
{
	Ft_Gpu_Hal_Wr32(phost, RAM_DL + 0 , CLEAR_COLOR_RGB(128,128,128));
	Ft_Gpu_Hal_Wr32(phost, RAM_DL + 4 , CLEAR(1,1,1));
	Ft_Gpu_Hal_Wr32(phost, RAM_DL + 60, DISPLAY()); // display the image

	/* Do a swap */
	SAMAPP_GPU_DLSwap(DLSWAP_FRAME);
    SAMAPP_ENABLE_DELAY();
}

/*****************************************************************************/
/* Example code to display few points at various offsets with various colors */
/*****************************************************************************/
ft_void_t	SAMAPP_GPU_Points()
{
	ft_uint32_t *p_DLRAM = (ft_uint32_t *)RAM_DL;

	/* Construct DL of points */
	Ft_Gpu_Hal_Wr32(phost, RAM_DL + 0 , CLEAR_COLOR_RGB(128,128,128));
	Ft_Gpu_Hal_Wr32(phost, RAM_DL + 4 , CLEAR(1,1,1));
	Ft_Gpu_Hal_Wr32(phost, RAM_DL + 8 , COLOR_RGB(128, 0, 0) );
	Ft_Gpu_Hal_Wr32(phost, RAM_DL + 12, POINT_SIZE(5 * 16) );
	Ft_Gpu_Hal_Wr32(phost, RAM_DL + 16, BEGIN(FTPOINTS) );
	Ft_Gpu_Hal_Wr32(phost, RAM_DL + 20, VERTEX2F((FT_DispWidth/5) * 16, (FT_DispHeight/2) * 16) );
	Ft_Gpu_Hal_Wr32(phost, RAM_DL + 24, COLOR_RGB(0, 128, 0) );
	Ft_Gpu_Hal_Wr32(phost, RAM_DL + 28, POINT_SIZE(15 * 16) );
	Ft_Gpu_Hal_Wr32(phost, RAM_DL + 32, VERTEX2F((FT_DispWidth*2/5) * 16, (FT_DispHeight/2) * 16) );
	Ft_Gpu_Hal_Wr32(phost, RAM_DL + 36, COLOR_RGB(0, 0, 128) );
	Ft_Gpu_Hal_Wr32(phost, RAM_DL + 40, POINT_SIZE(25 * 16) );
	Ft_Gpu_Hal_Wr32(phost, RAM_DL + 44, VERTEX2F((FT_DispWidth*3/5) * 16, (FT_DispHeight/2) * 16) );
	Ft_Gpu_Hal_Wr32(phost, RAM_DL + 48, COLOR_RGB(128, 128, 0) );
	Ft_Gpu_Hal_Wr32(phost, RAM_DL + 52, POINT_SIZE(35 * 16) );
	Ft_Gpu_Hal_Wr32(phost, RAM_DL + 56, VERTEX2F((FT_DispWidth*4/5) * 16, (FT_DispHeight/2) * 16) );
	Ft_Gpu_Hal_Wr32(phost, RAM_DL + 60, DISPLAY()); // display the image
	//delayms(100);

	/* Do a swap */
	SAMAPP_GPU_DLSwap(DLSWAP_FRAME);
    SAMAPP_ENABLE_DELAY();
}



ft_void_t SAMAPP_GPU_Lines()
{
	ft_int16_t LineHeight = 25;

	Ft_App_WrDlCmd_Buffer(phost, CLEAR(1, 1, 1)); // clear screen
	Ft_App_WrDlCmd_Buffer(phost, COLOR_RGB(128, 0, 0) );
	Ft_App_WrDlCmd_Buffer(phost, LINE_WIDTH(5 * 16) );
	Ft_App_WrDlCmd_Buffer(phost, BEGIN(LINES) );
	Ft_App_WrDlCmd_Buffer(phost, VERTEX2F((FT_DispWidth/4) * 16,((FT_DispHeight - LineHeight)/2) * 16) );
	Ft_App_WrDlCmd_Buffer(phost, VERTEX2F((FT_DispWidth/4) * 16,((FT_DispHeight + LineHeight)/2) * 16) );
	Ft_App_WrDlCmd_Buffer(phost, COLOR_RGB(0, 128, 0) );
	Ft_App_WrDlCmd_Buffer(phost, LINE_WIDTH(10 * 16) );
	LineHeight = 40;
	Ft_App_WrDlCmd_Buffer(phost, VERTEX2F((FT_DispWidth*2/4) * 16,((FT_DispHeight - LineHeight)/2) * 16) );
	Ft_App_WrDlCmd_Buffer(phost, VERTEX2F((FT_DispWidth*2/4) * 16,((FT_DispHeight + LineHeight)/2) * 16) );
	Ft_App_WrDlCmd_Buffer(phost, COLOR_RGB(128, 128, 0) );
	Ft_App_WrDlCmd_Buffer(phost, LINE_WIDTH(20 * 16) );
	LineHeight = 55;
	Ft_App_WrDlCmd_Buffer(phost, VERTEX2F((FT_DispWidth*3/4) * 16, ((FT_DispHeight - LineHeight)/2) * 16) );
	Ft_App_WrDlCmd_Buffer(phost, VERTEX2F((FT_DispWidth*3/4) * 16, ((FT_DispHeight + LineHeight)/2) * 16) );
	Ft_App_WrDlCmd_Buffer(phost, DISPLAY() );

	/* Download the DL into DL RAM */
	Ft_App_Flush_DL_Buffer(phost);

	/* Do a swap */
	SAMAPP_GPU_DLSwap(DLSWAP_FRAME);
    SAMAPP_ENABLE_DELAY();
}

ft_void_t	SAMAPP_GPU_Rectangles()
{
	ft_int16_t RectWidth,RectHeight;


	Ft_App_WrDlCmd_Buffer(phost, CLEAR(1, 1, 1)); // clear screen
	Ft_App_WrDlCmd_Buffer(phost, COLOR_RGB(0, 0, 128) );
	Ft_App_WrDlCmd_Buffer(phost, LINE_WIDTH(1 * 16) );//LINE_WIDTH is used for corner curvature
	Ft_App_WrDlCmd_Buffer(phost, BEGIN(RECTS) );
	RectWidth = 5;RectHeight = 25;
	Ft_App_WrDlCmd_Buffer(phost, VERTEX2F( ((FT_DispWidth/4) - (RectWidth/2)) * 16,((FT_DispHeight - RectHeight)/2) * 16) );
	Ft_App_WrDlCmd_Buffer(phost, VERTEX2F( ((FT_DispWidth/4) + (RectWidth/2)) * 16,((FT_DispHeight + RectHeight)/2) * 16) );
	Ft_App_WrDlCmd_Buffer(phost, COLOR_RGB(0, 128, 0) );
	Ft_App_WrDlCmd_Buffer(phost, LINE_WIDTH(5 * 16) );
	RectWidth = 10;RectHeight = 40;
	Ft_App_WrDlCmd_Buffer(phost, VERTEX2F( ((FT_DispWidth*2/4) - (RectWidth/2)) * 16,((FT_DispHeight - RectHeight)/2) * 16) );
	Ft_App_WrDlCmd_Buffer(phost, VERTEX2F( ((FT_DispWidth*2/4) + (RectWidth/2)) * 16,((FT_DispHeight + RectHeight)/2) * 16) );
	Ft_App_WrDlCmd_Buffer(phost, COLOR_RGB(128, 128, 0) );
	Ft_App_WrDlCmd_Buffer(phost, LINE_WIDTH(10 * 16) );
	RectWidth = 20;RectHeight = 55;
	Ft_App_WrDlCmd_Buffer(phost, VERTEX2F( ((FT_DispWidth*3/4) - (RectWidth/2)) * 16,((FT_DispHeight - RectHeight)/2) * 16) );
	Ft_App_WrDlCmd_Buffer(phost, VERTEX2F( ((FT_DispWidth*3/4) + (RectWidth/2)) * 16,((FT_DispHeight + RectHeight)/2) * 16) );
	Ft_App_WrDlCmd_Buffer(phost, DISPLAY() );

	/* Download the DL into DL RAM */
	Ft_App_Flush_DL_Buffer(phost);

	/* Do a swap */
	SAMAPP_GPU_DLSwap(DLSWAP_FRAME);
        SAMAPP_ENABLE_DELAY();
}


ft_void_t Touch_IRQHandler(uint16_t GPIO_Pin) //not used. Currently easier to read the flag register
{
	if(__HAL_GPIO_EXTI_GET_IT(GPIO_Pin) == RESET) return;
	if (phost->status != FT_GPU_HAL_OPENED) return; //nem jó
	//uint8_t flags=Ft_Gpu_Hal_Rd8(phost,REG_INT_FLAGS);
	//printf("%d\r\n",flags);
}
ft_bool_t Touch_IsTouched()
{
	ft_bool_t res=(Ft_Gpu_Hal_Rd8(phost,REG_INT_FLAGS)!=0); //read also cleares the flag!
	return res;
}

Touch_Event* Touch_GetEvent()
{
	static Touch_Event events[4];
	events[0].isValid = 0;
	uint32_t readWord = Ft_Gpu_Hal_Rd32(phost, REG_CTOUCH_TOUCH0_XY);
	if (readWord!=0x80008000)
	{
		events[0].x = (readWord>>16);
		events[0].y = (readWord & 0xffff);
		events[0].isValid = 1;
	}
//	readWord = Ft_Gpu_Hal_Rd32(phost, REG_CTOUCH_TOUCH1_XY);
//	events[1].x = (readWord & 0xffff);
//	events[1].y = FT_DispWidth - (readWord>>16);
//	readWord = Ft_Gpu_Hal_Rd32(phost, REG_CTOUCH_TOUCH2_XY);
//	events[2].x = (readWord & 0xffff);
//	events[2].y = FT_DispWidth - (readWord>>16);
//	readWord = Ft_Gpu_Hal_Rd32(phost, REG_CTOUCH_TOUCH3_XY);
//	events[3].x = (readWord & 0xffff);
//	events[3].y = FT_DispWidth - (readWord>>16);
	return events;
}



ft_bool_t Touch_IsCalibrated()
{
	uint32_t calibmagic;
	EE_ReadVariable(EE_IsTouchCalibrated,&calibmagic);
	return (calibmagic == 0x4545);
}
ft_void_t Touch_LoadCalibration()
{
	uint32_t a,b,c,d,e,f;
	if (Touch_IsCalibrated())
	{
		EE_ReadVariable(EE_REG_TOUCH_TRANSFORM_A,&a);
		EE_ReadVariable(EE_REG_TOUCH_TRANSFORM_B,&b);
		EE_ReadVariable(EE_REG_TOUCH_TRANSFORM_C,&c);
		EE_ReadVariable(EE_REG_TOUCH_TRANSFORM_D,&d);
		EE_ReadVariable(EE_REG_TOUCH_TRANSFORM_E,&e);
		EE_ReadVariable(EE_REG_TOUCH_TRANSFORM_F,&f);
	}
	else
	{
		a=0xffffef80;
		b=0xf04a;
		c=0xd4b2f;
		d=0xfffef1c9;
		e=0x233;
		f=0x143e6e6;
	}
	Ft_Gpu_Hal_Wr32(phost,REG_TOUCH_TRANSFORM_A,a);
	Ft_Gpu_Hal_Wr32(phost,REG_TOUCH_TRANSFORM_B,b);
	Ft_Gpu_Hal_Wr32(phost,REG_TOUCH_TRANSFORM_C,c);
	Ft_Gpu_Hal_Wr32(phost,REG_TOUCH_TRANSFORM_D,d);
	Ft_Gpu_Hal_Wr32(phost,REG_TOUCH_TRANSFORM_E,e);
	Ft_Gpu_Hal_Wr32(phost,REG_TOUCH_TRANSFORM_F,f);
}
ft_void_t Touch_Calibrate()
{
	Ft_Gpu_CoCmd_Dlstart(phost);
	Ft_App_WrCoCmd_Buffer(phost,CLEAR_COLOR_RGB(64,64,64));
	Ft_App_WrCoCmd_Buffer(phost,CLEAR(1,1,1));
	Ft_App_WrCoCmd_Buffer(phost,COLOR_RGB(255,255,255));
	Ft_Gpu_CoCmd_BgColor(phost, 0x0000ff);
	Ft_App_WrCoCmd_Buffer(phost,COLOR_RGB(0xff,0x00,0x00));
	Ft_Gpu_CoCmd_Calibrate(phost,0);
	Ft_App_WrCoCmd_Buffer(phost,DISPLAY());
	Ft_Gpu_CoCmd_Swap(phost);
	/* Download the commands into fifo */
	Ft_App_Flush_Co_Buffer(phost);
	/* Wait till coprocessor completes the operation */
	Ft_Gpu_Hal_WaitCmdfifo_empty(phost);

	uint32_t a=Ft_Gpu_Hal_Rd32(phost,REG_TOUCH_TRANSFORM_A);
	uint32_t b=Ft_Gpu_Hal_Rd32(phost,REG_TOUCH_TRANSFORM_B);
	uint32_t c=Ft_Gpu_Hal_Rd32(phost,REG_TOUCH_TRANSFORM_C);
	uint32_t d=Ft_Gpu_Hal_Rd32(phost,REG_TOUCH_TRANSFORM_D);
	uint32_t e=Ft_Gpu_Hal_Rd32(phost,REG_TOUCH_TRANSFORM_E);
	uint32_t f=Ft_Gpu_Hal_Rd32(phost,REG_TOUCH_TRANSFORM_F);

	EE_WriteVariable(EE_REG_TOUCH_TRANSFORM_A,a);
	EE_WriteVariable(EE_REG_TOUCH_TRANSFORM_B,b);
	EE_WriteVariable(EE_REG_TOUCH_TRANSFORM_C,c);
	EE_WriteVariable(EE_REG_TOUCH_TRANSFORM_D,d);
	EE_WriteVariable(EE_REG_TOUCH_TRANSFORM_E,e);
	EE_WriteVariable(EE_REG_TOUCH_TRANSFORM_F,f);
	EE_WriteVariable(EE_IsTouchCalibrated,0x4545);

	printf("A:%x\r\n",a);
	printf("B:%x\r\n",b);
	printf("C:%x\r\n",c);
	printf("D:%x\r\n",d);
	printf("E:%x\r\n",e);
	printf("F:%x\r\n",f);


}


ft_void_t SAMAPP_Touch()
{
	ft_int32_t LoopFlag = 0,wbutton,hbutton,tagval,tagoption;
	ft_char8_t StringArray[100],StringArray1[100];
	ft_uint32_t ReadWord;
	ft_int16_t xvalue,yvalue,pendown;


	/*************************************************************************/
	/* Below code demonstrates the usage of touch function. Display info     */
	/* touch raw, touch screen, touch tag, raw adc and resistance values     */
	/*************************************************************************/
	LoopFlag = 300;
	wbutton = FT_DispWidth/8;
	hbutton = FT_DispHeight/8;




	uint8_t res1=Ft_Gpu_Hal_Rd8(phost,REG_INT_EN);
	uint8_t res2=Ft_Gpu_Hal_Rd8(phost,REG_INT_MASK);
	printf("%d %d\r\n",res1,res2);
//	while(1){
//		uint8_t res3=Ft_Gpu_Hal_Rd8(phost,REG_INT_FLAGS);
//		printf("%d\r\n",res3);
//	}
	while(1)
	{
		while (Ft_Gpu_Hal_Rd8(phost,REG_INT_FLAGS)==0);

		StringArray[0] = '\0';
		strcat(StringArray,"\r\nTouch Screen XY0 (");
		ReadWord = Ft_Gpu_Hal_Rd32(phost, REG_CTOUCH_TOUCH0_XY);
  		yvalue = (ReadWord & 0xffff);
		xvalue = (ReadWord>>16);
		Ft_Gpu_Hal_Dec2Ascii(StringArray,(ft_int32_t)xvalue);
		strcat(StringArray,",");
		Ft_Gpu_Hal_Dec2Ascii(StringArray,(ft_int32_t)yvalue);
		strcat(StringArray,")");
		printf(StringArray);
		StringArray[0] = '\0';
		strcat(StringArray,"Touch Screen XY1 (");
		ReadWord = Ft_Gpu_Hal_Rd32(phost, REG_CTOUCH_TOUCH1_XY);
  		yvalue = (ReadWord & 0xffff);
		xvalue = (ReadWord>>16);
		Ft_Gpu_Hal_Dec2Ascii(StringArray,(ft_int32_t)xvalue);
		strcat(StringArray,",");
		Ft_Gpu_Hal_Dec2Ascii(StringArray,(ft_int32_t)yvalue);
		strcat(StringArray,")");
		printf(StringArray);

		StringArray[0] = '\0';
		strcat(StringArray,"Touch TAG (");
		ReadWord = Ft_Gpu_Hal_Rd8(phost, REG_TOUCH_TAG);
		Ft_Gpu_Hal_Dec2Ascii(StringArray,ReadWord);
		strcat(StringArray,")");
		printf(StringArray);

	}
	return;
	{




		Ft_Gpu_CoCmd_Dlstart(phost);
		Ft_App_WrCoCmd_Buffer(phost,CLEAR_COLOR_RGB(64,64,64));
		Ft_App_WrCoCmd_Buffer(phost,CLEAR(1,1,1));
		Ft_App_WrCoCmd_Buffer(phost,COLOR_RGB(0xff,0xff,0xff));
		Ft_App_WrCoCmd_Buffer(phost,TAG_MASK(0));

		StringArray[0] = '\0';
		strcat(StringArray,"\r\nTouch Screen XY0 (");
		ReadWord = Ft_Gpu_Hal_Rd32(phost, REG_CTOUCH_TOUCH0_XY);
		/*yvalue = (ft_uint16_t)(ReadWord & 0xffff);
		xvalue = (ft_uint16_t)((ReadWord>>16) & 0xffff);*/
  		yvalue = (ReadWord & 0xffff);
		xvalue = (ReadWord>>16);
		Ft_Gpu_Hal_Dec2Ascii(StringArray,(ft_int32_t)xvalue);
		strcat(StringArray,",");
		Ft_Gpu_Hal_Dec2Ascii(StringArray,(ft_int32_t)yvalue);
		strcat(StringArray,")");
		Ft_Gpu_CoCmd_Text(phost,FT_DispWidth/2, 50, 26, OPT_CENTER, StringArray);
		printf(StringArray);
		StringArray[0] = '\0';
		strcat(StringArray,"Touch Screen XY1 (");
		ReadWord = Ft_Gpu_Hal_Rd32(phost, REG_CTOUCH_TOUCH1_XY);
  		yvalue = (ReadWord & 0xffff);
		xvalue = (ReadWord>>16);
		Ft_Gpu_Hal_Dec2Ascii(StringArray,(ft_int32_t)xvalue);
		strcat(StringArray,",");
		Ft_Gpu_Hal_Dec2Ascii(StringArray,(ft_int32_t)yvalue);
		strcat(StringArray,")");
		Ft_Gpu_CoCmd_Text(phost,FT_DispWidth/2, 70, 26, OPT_CENTER, StringArray);

		printf(StringArray);
		StringArray[0] = '\0';
		strcat(StringArray,"Touch Screen XY2 (");
		ReadWord = Ft_Gpu_Hal_Rd32(phost, REG_CTOUCH_TOUCH2_XY);
  		yvalue = (ReadWord & 0xffff);
		xvalue = (ReadWord>>16);
		Ft_Gpu_Hal_Dec2Ascii(StringArray,(ft_int32_t)xvalue);
		strcat(StringArray,",");
		Ft_Gpu_Hal_Dec2Ascii(StringArray,(ft_int32_t)yvalue);
		strcat(StringArray,")");
		Ft_Gpu_CoCmd_Text(phost,FT_DispWidth/2, 90, 26, OPT_CENTER, StringArray);
		printf(StringArray);
		StringArray[0] = '\0';
		strcat(StringArray,"Touch Screen XY3 (");
		ReadWord = Ft_Gpu_Hal_Rd32(phost, REG_CTOUCH_TOUCH3_XY);
  		yvalue = (ReadWord & 0xffff);
		xvalue = (ReadWord>>16);
		Ft_Gpu_Hal_Dec2Ascii(StringArray,(ft_int32_t)xvalue);
		strcat(StringArray,",");
		Ft_Gpu_Hal_Dec2Ascii(StringArray,(ft_int32_t)yvalue);
		strcat(StringArray,")");
		Ft_Gpu_CoCmd_Text(phost,FT_DispWidth/2, 110, 26, OPT_CENTER, StringArray);
		printf(StringArray);
		StringArray[0] = '\0';
			StringArray1[0] = '\0';
		strcat(StringArray,"Touch Screen XY4 (");
		xvalue = Ft_Gpu_Hal_Rd16(phost, REG_CTOUCH_TOUCH4_X);
		yvalue = Ft_Gpu_Hal_Rd16(phost, REG_CTOUCH_TOUCH4_Y);


		Ft_Gpu_Hal_Dec2Ascii(StringArray,(ft_int32_t)xvalue);
		strcat(StringArray,",");
		Ft_Gpu_Hal_Dec2Ascii(StringArray1,(ft_int32_t)yvalue);
		strcat(StringArray1,")");
		strcat(StringArray,StringArray1);
		Ft_Gpu_CoCmd_Text(phost,FT_DispWidth/2, 130, 26, OPT_CENTER, StringArray);
		printf(StringArray);
		StringArray[0] = '\0';
		strcat(StringArray,"Touch TAG (");
		ReadWord = Ft_Gpu_Hal_Rd8(phost, REG_TOUCH_TAG);
		Ft_Gpu_Hal_Dec2Ascii(StringArray,ReadWord);
		strcat(StringArray,")");
		Ft_Gpu_CoCmd_Text(phost,FT_DispWidth/2, 170, 26, OPT_CENTER, StringArray);
		tagval = ReadWord;


		Ft_Gpu_CoCmd_FgColor(phost,0x008000);
		Ft_App_WrCoCmd_Buffer(phost,TAG_MASK(1));

		Ft_App_WrCoCmd_Buffer(phost,TAG(13));
		tagoption = 0;
		if(13 == tagval)
		{
			tagoption = OPT_FLAT;
		}
		Ft_Gpu_CoCmd_Button(phost,(FT_DispWidth/2)- (wbutton/2) ,(FT_DispHeight*3/4) - (hbutton/2),wbutton,hbutton,26,tagoption,"Tag13");

		Ft_App_WrCoCmd_Buffer(phost,DISPLAY());
		Ft_Gpu_CoCmd_Swap(phost);

		/* Download the commands into fifo */
		Ft_App_Flush_Co_Buffer(phost);

		/* Wait till coprocessor completes the operation */
		Ft_Gpu_Hal_WaitCmdfifo_empty(phost);
		Ft_Gpu_Hal_Sleep(30);

	}

		Ft_Gpu_Hal_Wr8(phost,REG_CTOUCH_EXTENDED, CTOUCH_MODE_COMPATIBILITY);
		Ft_Gpu_Hal_Sleep(30);
}

ft_void_t	Graphics_Clear()
{

	Ft_App_WrDlCmd_Buffer(phost, CLEAR(1, 1, 1)); // clear screen
	/* Download the DL into DL RAM */
	Ft_App_Flush_DL_Buffer(phost);
	SAMAPP_GPU_DLSwap(DLSWAP_FRAME);
}


ft_void_t	Graphics_Test()
{
	Ft_Gpu_CoCmd_Dlstart(phost);
	Ft_Gpu_CoCmd_Text(phost,(FT_DispWidth/2), (FT_DispHeight/2), 27,OPT_CENTER,"Please Tap on the dot");
	while(1);
	Ft_App_Flush_Co_Buffer(phost);
	Ft_Gpu_CoCmd_Calibrate(phost,10);
	Ft_App_Flush_Co_Buffer(phost);

	while(1);
	Ft_App_WrDlCmd_Buffer(phost, CMD_DLSTART );
	Ft_App_WrDlCmd_Buffer(phost,  CLEAR_COLOR_RGB(64,64,64));
	Ft_App_WrDlCmd_Buffer(phost,  CLEAR(1,1,1));
	Ft_App_WrDlCmd_Buffer(phost, COLOR_RGB(0xFF, 0xFF, 0xFF));

	Ft_Gpu_CoCmd_Text(phost,(FT_DispWidth/2), (FT_DispHeight/2), 27,OPT_CENTER,"Please Tap on the dot");
	Ft_Gpu_CoCmd_Calibrate(phost,0);
	Ft_Gpu_CoCmd_Swap(phost);
	/* Wait till coprocessor completes the operation */
	Ft_Gpu_Hal_WaitCmdfifo_empty(phost);

	/* Download the DL into DL RAM */
	Ft_App_Flush_DL_Buffer(phost);
	SAMAPP_GPU_DLSwap(DLSWAP_FRAME);
}


ft_void_t StartupScreen()
{


//	while(1)
//	{
//		while(!Touch_IsTouched());
//		Touch_Event* te= Touch_GetEvent();
//		if (!te->isValid) continue;
//
//		Ft_App_WrDlCmd_Buffer(phost, CLEAR_COLOR_RGB(128,128,128)); // clear screen
//		Ft_App_WrDlCmd_Buffer(phost,  CLEAR(1,1,1));
//		Ft_App_WrDlCmd_Buffer(phost, COLOR_RGB(128, 0, 0) );
//		Ft_App_WrDlCmd_Buffer(phost, POINT_SIZE(40 * 16)  );
//		Ft_App_WrDlCmd_Buffer(phost, BEGIN(FTPOINTS) );
//
//		Ft_App_WrDlCmd_Buffer(phost, VERTEX2F((te->x) * 16,(te->y) * 16) );
//		Ft_App_WrDlCmd_Buffer(phost, DISPLAY() );
//
//		/* Download the DL into DL RAM */
//		Ft_App_Flush_DL_Buffer(phost);
//
//		/* Do a swap */
//		SAMAPP_GPU_DLSwap(DLSWAP_FRAME);
//
//	}

	ft_int16_t xOffset,yOffset,cRadius,xDistBtwClocks;

	xDistBtwClocks = FT_DispWidth/5;
	cRadius = 100;
	xOffset = FT_DispHeight/2;
	yOffset = FT_DispWidth/2;
	while(1)
	{
		while(!Touch_IsTouched());
		Touch_Event* te= Touch_GetEvent();
		if (te->isValid)
		{
			xOffset = te->x;
			yOffset = te->y;
		}
		Ft_Gpu_CoCmd_Dlstart(phost);
		Ft_App_WrCoCmd_Buffer(phost,CLEAR_COLOR_RGB(64,64,64));
		Ft_App_WrCoCmd_Buffer(phost,CLEAR(1,1,1));
		Ft_App_WrCoCmd_Buffer(phost,COLOR_RGB(255,255,255));
//		Ft_Gpu_CoCmd_BgColor(phost, 0x0000ff);
//		Ft_App_WrCoCmd_Buffer(phost,COLOR_RGB(0xff,0x00,0x00));
//		Ft_App_WrCoCmd_Buffer(phost,POINT_SIZE(5 * 16));
//		Ft_App_WrCoCmd_Buffer(phost, BEGIN(FTPOINTS) );
		//Ft_App_WrCoCmd_Buffer(phost, VERTEX2F((xOffset) * 16, (yOffset) * 16) );


		RTC_TimeTypeDef stimestructureget;
		RTC_DateTypeDef sdatestructureget;

		/* Get the RTC current Time */
		HAL_RTC_GetTime(&hrtc, &stimestructureget, RTC_FORMAT_BIN);
		/* Get the RTC current Date */
		HAL_RTC_GetDate(&hrtc, &sdatestructureget, RTC_FORMAT_BIN);

		Ft_Gpu_CoCmd_Clock(phost, xOffset,yOffset,cRadius,OPT_FLAT,stimestructureget.Hours,stimestructureget.Minutes,stimestructureget.Seconds,0);
		//Ft_Gpu_CoCmd_Clock(phost, xOffset,yOffset,cRadius,OPT_FLAT,4,4,54,0);
		Ft_App_WrCoCmd_Buffer(phost,DISPLAY());
		Ft_Gpu_CoCmd_Swap(phost);
		/* Download the commands into fifo */
		Ft_App_Flush_Co_Buffer(phost);
		/* Wait till coprocessor completes the operation */
		Ft_Gpu_Hal_WaitCmdfifo_empty(phost);

	}


	ft_int16_t LineHeight = 25;
	Ft_App_WrDlCmd_Buffer(phost, CLEAR(1, 1, 1)); // clear screen
	Ft_App_WrDlCmd_Buffer(phost, COLOR_RGB(128, 0, 0) );
	Ft_App_WrDlCmd_Buffer(phost, LINE_WIDTH(5 * 16) );
	Ft_App_WrDlCmd_Buffer(phost, BEGIN(LINES) );
	Ft_App_WrDlCmd_Buffer(phost, VERTEX2F((FT_DispWidth/4) * 16,((FT_DispHeight - LineHeight)/2) * 16) );
	Ft_App_WrDlCmd_Buffer(phost, VERTEX2F((FT_DispWidth/4) * 16,((FT_DispHeight + LineHeight)/2) * 16) );
	Ft_App_WrDlCmd_Buffer(phost, COLOR_RGB(0, 128, 0) );
	Ft_App_WrDlCmd_Buffer(phost, LINE_WIDTH(10 * 16) );
	LineHeight = 40;
	Ft_App_WrDlCmd_Buffer(phost, VERTEX2F((FT_DispWidth*2/4) * 16,((FT_DispHeight - LineHeight)/2) * 16) );
	Ft_App_WrDlCmd_Buffer(phost, VERTEX2F((FT_DispWidth*2/4) * 16,((FT_DispHeight + LineHeight)/2) * 16) );
	Ft_App_WrDlCmd_Buffer(phost, COLOR_RGB(128, 128, 0) );
	Ft_App_WrDlCmd_Buffer(phost, LINE_WIDTH(20 * 16) );
	LineHeight = 55;
	Ft_App_WrDlCmd_Buffer(phost, VERTEX2F((FT_DispWidth*3/4) * 16, ((FT_DispHeight - LineHeight)/2) * 16) );
	Ft_App_WrDlCmd_Buffer(phost, VERTEX2F((FT_DispWidth*3/4) * 16, ((FT_DispHeight + LineHeight)/2) * 16) );
	Ft_App_WrDlCmd_Buffer(phost, DISPLAY() );

	/* Download the DL into DL RAM */
	Ft_App_Flush_DL_Buffer(phost);

	/* Do a swap */
	SAMAPP_GPU_DLSwap(DLSWAP_FRAME);

    SAMAPP_ENABLE_DELAY();

    Ft_App_Flush_Co_Buffer(phost);

	while(1)
	{
		printf("b");
		delay(500);
	}

}


ft_void_t	Graphics_Circle(uint32_t x,uint32_t y )
{
	/* Construct DL of points */
	Ft_App_WrDlCmd_Buffer(phost, CMD_DLSTART );
	Ft_App_WrDlCmd_Buffer(phost,  CLEAR_COLOR_RGB(0,0,0));
	Ft_App_WrDlCmd_Buffer(phost,  CLEAR(1,1,1));
	Ft_App_WrDlCmd_Buffer(phost, COLOR_RGB(255, 0, 0));
	Ft_App_WrDlCmd_Buffer(phost, POINT_SIZE(320));
	Ft_App_WrDlCmd_Buffer(phost, BEGIN(FTPOINTS));
	Ft_App_WrDlCmd_Buffer(phost, VERTEX2II(x,y,0,0) );
	Ft_App_WrDlCmd_Buffer(phost, END() );
	Ft_App_WrDlCmd_Buffer(phost, DISPLAY() );
	/* Download the DL into DL RAM */
	Ft_App_Flush_DL_Buffer(phost);
	SAMAPP_GPU_DLSwap(DLSWAP_FRAME);
	//SAMAPP_ENABLE_DELAY();

}

/* API to demonstrate jpeg decode functionality */
ft_void_t SAMAPP_CoPro_Loadimage()
{
	ft_uint8_t *pbuff;
	const SAMAPP_Bitmap_header_t *pBitmapHdr = NULL;
	ft_int16_t ImgW,ImgH,xoffset,yoffset;

	/*************************************************************************/
	/* Below code demonstrates the usage of loadimage function               */
	/* Download the jpg data into command buffer and in turn coprocessor decodes      */
	/* and dumps into location 0 with rgb565 format                          */
	/*************************************************************************/
	{
		FIL fil;       /* File object */
		HAL_Error_Handler(f_open(&fil,"autumn.jpg",FA_READ));


	//	ft_int32_t FileLen = 0;
	//	pBitmapHdr = &SAMAPP_Bitmap_RawData_Header[0];
		ImgW =194;
		ImgH = 320;

		xoffset = 0;
		yoffset = 0;
		/* Clear the memory at location 0 - any previous bitmap data */

		Ft_App_WrCoCmd_Buffer(phost, CMD_MEMSET);
		Ft_App_WrCoCmd_Buffer(phost, 0L);//starting address of memset
		Ft_App_WrCoCmd_Buffer(phost, 255L);//value of memset
		Ft_App_WrCoCmd_Buffer(phost, 320*2*194);//number of elements to be changed

		/* Set the display list for graphics processor */

		/* Bitmap construction by MCU - display lena at 112x8 offset */
		/* Transfer the data into coprocessor memory directly word by word */
		Ft_App_WrCoCmd_Buffer(phost, CMD_DLSTART);
		Ft_App_WrCoCmd_Buffer(phost, CLEAR_COLOR_RGB(0,255,255));
		Ft_App_WrCoCmd_Buffer(phost, CLEAR(1,1,1));
		Ft_App_WrCoCmd_Buffer(phost, COLOR_RGB(255,255,255));
		Ft_App_WrCoCmd_Buffer(phost, BEGIN(BITMAPS));
		Ft_App_WrCoCmd_Buffer(phost, BITMAP_SOURCE(0));
		Ft_App_WrCoCmd_Buffer(phost, BITMAP_LAYOUT(RGB565,ImgW*2,ImgH));
		Ft_App_WrCoCmd_Buffer(phost, BITMAP_SIZE(BILINEAR,BORDER,BORDER,ImgW,ImgH));
		Ft_App_WrCoCmd_Buffer(phost, VERTEX2F(xoffset*16,yoffset*16));
		Ft_App_WrCoCmd_Buffer(phost, END());

		/*  Display the text information */
		xoffset = ((FT_DispWidth)/2);
		yoffset = ((FT_DispHeight)/2);
		Ft_Gpu_CoCmd_Text(phost,xoffset, yoffset, 26, OPT_CENTER, "Display bitmap by jpg decode");
		Ft_App_WrCoCmd_Buffer(phost,DISPLAY());
		Ft_Gpu_CoCmd_Swap(phost);

		/* Download the commands into fifo */
		Ft_App_Flush_Co_Buffer(phost);

		/* Wait till coprocessor completes the operation */
		Ft_Gpu_Hal_WaitCmdfifo_empty(phost);
#if defined(MSVC_PLATFORM) || defined(MSVC_FT800EMU)

	/* decode the jpeg data */
	if(NULL == pFile)
	{
		printf("Error in opening file %s \n","mandrill256.jpg");
	}
	else
	{
#endif

		/******************* Decode jpg output into location 0 and output color format as RGB565 *********************/
		Ft_Gpu_Hal_WrCmd32(phost,  CMD_LOADIMAGE);
		Ft_Gpu_Hal_WrCmd32(phost,  0);//destination address of jpg decode
		Ft_Gpu_Hal_WrCmd32(phost,  0);//output format of the bitmap


		uint32_t FileLen = f_size(&fil);
		f_lseek(&fil,0);

		pbuff = (ft_uint8_t *)malloc(8192);

		while(FileLen > 0)
		{
			/* download the data into the command buffer by 2kb one shot */
			ft_uint16_t blocklen = FileLen>8192?8192:FileLen;
			UINT readlen;
			/* copy the data into pbuff and then transfter it to command buffer */
			f_read(&fil,pbuff,blocklen,&readlen);
			//fread(pbuff,1,blocklen,pFile);
			FileLen -= readlen;
			/* copy data continuously into command memory */
			Ft_Gpu_Hal_WrCmdBuf(phost,pbuff, readlen);//alignment is already taken care by this api
		}


		SAMAPP_ENABLE_DELAY();
		/******************** Decode jpg output into location 0 & output as MONOCHROME ******************************/
		/* Clear the memory at location 0 - any previous bitmap data */\
		xoffset = ((FT_DispWidth - ImgW)/2);
		yoffset = ((FT_DispHeight - ImgH)/2);


		Ft_App_WrCoCmd_Buffer(phost, CMD_MEMSET);
		Ft_App_WrCoCmd_Buffer(phost, 0L);//starting address of memset
		Ft_App_WrCoCmd_Buffer(phost, 255L);//value of memset
		Ft_App_WrCoCmd_Buffer(phost, 340L*2*194);//number of elements to be changed

		/* Set the display list for graphics processor */
		/* Bitmap construction by MCU - display lena at 112x8 offset */
		/* Transfer the data into coprocessor memory directly word by word */
		Ft_App_WrCoCmd_Buffer(phost, CMD_DLSTART);
		Ft_App_WrCoCmd_Buffer(phost, CLEAR_COLOR_RGB(0,0,0));
		Ft_App_WrCoCmd_Buffer(phost, CLEAR(1,1,1));
		Ft_App_WrCoCmd_Buffer(phost, COLOR_RGB(255,255,255));
		Ft_App_WrCoCmd_Buffer(phost, BEGIN(BITMAPS));
		Ft_App_WrCoCmd_Buffer(phost, BITMAP_SOURCE(0));
		Ft_App_WrCoCmd_Buffer(phost, BITMAP_LAYOUT(L8,ImgW,ImgH));//monochrome
		Ft_App_WrCoCmd_Buffer(phost, BITMAP_SIZE(BILINEAR,BORDER,BORDER,ImgW,ImgH));
		Ft_App_WrCoCmd_Buffer(phost, VERTEX2F(xoffset*16,yoffset*16));
		Ft_App_WrCoCmd_Buffer(phost, END());

		/*  Display the text information */
		xoffset = ((FT_DispWidth)/2);
		yoffset = ((FT_DispHeight)/2);
		Ft_Gpu_CoCmd_Text(phost,xoffset, yoffset, 26, OPT_CENTER, "Display bitmap by jpg decode L8");
		Ft_App_WrCoCmd_Buffer(phost,DISPLAY());
		Ft_Gpu_CoCmd_Swap(phost);

		/* Download the commands into fifo */
		Ft_App_Flush_Co_Buffer(phost);

		/* Wait till coprocessor completes the operation */
		Ft_Gpu_Hal_WaitCmdfifo_empty(phost);
                SAMAPP_ENABLE_DELAY();

		Ft_Gpu_Hal_WrCmd32(phost,  CMD_LOADIMAGE);
		Ft_Gpu_Hal_WrCmd32(phost,  0);//destination address of jpg decode
		Ft_Gpu_Hal_WrCmd32(phost,  OPT_MONO);//output format of the bitmap - default is rgb565

		f_lseek(&fil,0);
		while(FileLen > 0)
		{
			/* download the data into the command buffer by 2kb one shot */
			ft_uint16_t blocklen = FileLen>8192?8192:FileLen;
			UINT readlen;
			/* copy the data into pbuff and then transfter it to command buffer */
			f_read(&fil,pbuff,blocklen,&readlen);
			//fread(pbuff,1,blocklen,pFile);
			FileLen -= readlen;
			/* copy data continuously into command memory */
			Ft_Gpu_Hal_WrCmdBuf(phost,pbuff, readlen);//alignment is already taken care by this api
		}
		free(pbuff);

		/* close the opened jpg file */
		f_close(&fil);
        }

        SAMAPP_ENABLE_DELAY();

}


static ft_uint8_t imbuff[8192];

typedef enum
{
	JPEGERR_NONE,
	JPEGERR_FILEOPEN,
	JPEGERR_NOTJPEG,
	JPEGERR_TOOLARGE,
	JPEGERR_NOTBASELINE
}EjpegError;

uint8_t r; //determines the location in Graphics RAM

void myLoadJpeg(uint16_t x,uint16_t y,char * filename)
{
	FIL fil;       /* File object */
	EjpegError error_code=JPEGERR_NONE;

	if (f_open(&fil,filename,FA_READ)!=FR_OK) error_code=JPEGERR_FILEOPEN;

	uint32_t filesize = f_size(&fil);
	ft_uint16_t image_height = 0;
	ft_uint16_t image_width = 0;

	if (error_code==JPEGERR_NONE)
	{

		f_lseek(&fil,0);

		ft_uint8_t markerID[2];
		UINT readcnt;
		//read first 2 bytes to check for JPEG SOI marker 0xFFD8
		f_read(&fil,&markerID, 2, &readcnt); //read first 2 bytes

		if (((markerID[0] << 8) + markerID[1]) != 0xFFD8)
		{
			error_code=JPEGERR_NOTJPEG; //No SOI marker (0xFFD8) detected
		}

		ft_uint8_t marker_info[8];
		ft_uint8_t marker_next_byte[8];


		while (f_read(&fil,&markerID, 1, &readcnt) == 0 && readcnt!=0 && error_code==JPEGERR_NONE)
		{
			if (markerID[0] == 0xFF)
			{
				f_read(&fil,&marker_next_byte, 1,&readcnt);
				if (marker_next_byte[0] >> 4 == 0xE) //APPn marker found if true
				{
					f_read(&fil,&marker_info, 2, &readcnt);
					f_lseek(&fil, ((marker_info[0]*256) + marker_info[1]-2)); //skip APPn marker
				}
				if (marker_next_byte[0] == 0xC0) //baseline JPEG marker found
				{
					f_read(&fil,&marker_info, 8, &readcnt);
					image_height = marker_info[3]*256 + marker_info[4];
					image_width = marker_info[5]*256 + marker_info[6];

					if (image_height > FT_DispHeight) error_code=JPEGERR_TOOLARGE;//Max is 272 for WQVGA, 240 for QVGA
					if (image_width > FT_DispWidth) error_code=JPEGERR_TOOLARGE;	//Max is 480 for WQVGA, 320 for QVGA
				}
				if (marker_next_byte[0] == 0xC2)
				{
					error_code=JPEGERR_NOTBASELINE;//check if progressive JPEG
				}
			}
		}
	}

	if (error_code != JPEGERR_NONE) //error flagged, display message
	{
		f_close(&fil); /* close the opened JPEG file */
		Ft_Gpu_CoCmd_Dlstart(phost);
		Ft_App_WrCoCmd_Buffer(phost,CLEAR_COLOR_RGB(64,64,64));
		Ft_App_WrCoCmd_Buffer(phost, CLEAR(1, 1, 1));
		Ft_App_WrCoCmd_Buffer(phost, COLOR_RGB(255, 255, 255));
		if (error_code == JPEGERR_NOTJPEG) Ft_Gpu_CoCmd_Text(phost, FT_DispWidth / 2, FT_DispHeight / 2, 26, OPT_CENTERX | OPT_CENTERY, "Error: Selected file is not a JPEG!");
		if (error_code == JPEGERR_TOOLARGE) Ft_Gpu_CoCmd_Text(phost, FT_DispWidth / 2, FT_DispHeight / 2, 26, OPT_CENTERX | OPT_CENTERY, "Error: Image size is not compatible!");
		if (error_code == JPEGERR_NOTBASELINE) Ft_Gpu_CoCmd_Text(phost, FT_DispWidth / 2, FT_DispHeight / 2, 26, OPT_CENTERX | OPT_CENTERY, "Error: Selected file is a progressive JPEG!");
		Ft_App_WrCoCmd_Buffer(phost, DISPLAY());
		Ft_Gpu_CoCmd_Swap(phost);
		Ft_App_Flush_Co_Buffer(phost);
		Ft_Gpu_Hal_WaitCmdfifo_empty(phost);
	}


//	Ft_Gpu_CoCmd_LoadImage(phost, 0,0);
//    Ft_Gpu_Hal_WrCmdBuf_nowait(phost,Ft_CmdBuffer,Ft_CmdBuffer_Index);

	//no error, therefore load image into FT800 via command processor
	Ft_Gpu_Hal_WrCmd32(phost, CMD_LOADIMAGE); //Load a JPEG image via command processor
	Ft_Gpu_Hal_WrCmd32(phost, (r ? 131072L : 100)); //Destination starting address in Graphics RAM
	//Ft_Gpu_Hal_WrCmd32(phost, 0); //Options 0 for RGB565, 1 for L8 monochrome,
	Ft_Gpu_Hal_WrCmd32(phost,OPT_NODL);
	//2 for RGB565 with no display list commands, 3 for L8 with no display list commands

	f_lseek(&fil,0);
	UINT rd;
	while (filesize > 0)
	{
		UINT blocklen = filesize > 8192 ? 8192 : filesize;
		/*copy the data into imbuff and then transfter it to command buffer*/
		f_read(&fil,imbuff,blocklen,&rd);
		filesize -= rd; //reduce filesize by blocklen
		Ft_Gpu_Hal_WrCmdBuf(phost, imbuff, rd); //write JPEG file to command buffer
	}
	f_close(&fil); /* close the opened JPEG file */

	//centre image on display
//	ft_uint16_t xcoord = (FT_DispWidth/2) - (0.5*image_width);
//	ft_uint16_t ycoord = (FT_DispHeight/2) - (0.5*image_height);
	ft_uint16_t xcoord = x;
	ft_uint16_t ycoord = y;

	Ft_CmdBuffer_Index = 0;

	Ft_Gpu_CoCmd_Dlstart(phost);
	Ft_App_WrCoCmd_Buffer(phost, CLEAR(1, 1, 1)); //clear screen to predefined values
	//***************************************************************************************************
	//Display list commands in this section are generated by CMD_LOADIMAGE unless option
	//is set to 2 or 3
	//Ft_App_WrCoCmd_Buffer(phost, BITMAP_HANDLE(r));
	Ft_App_WrCoCmd_Buffer(phost, BITMAP_SOURCE((r ? 131072L : 100))); //specify the source address of //bitmap in RAM_G
	//specify bit map format, linestride and height for RGB565
	Ft_App_WrCoCmd_Buffer(phost, BITMAP_LAYOUT(RGB565, image_width * 2, image_height));
	//specify bit map format, linestride and height for L8
	//Ft_App_WrCoCmd_Buffer(phost, BITMAP_LAYOUT(L8, image_width * 1, image_height));
	// controls drawing of bitmap
	Ft_App_WrCoCmd_Buffer(phost, BITMAP_SIZE(NEAREST, BORDER, BORDER, image_width, image_height));
	//***************************************************************************************************
	Ft_App_WrCoCmd_Buffer(phost, BEGIN(BITMAPS));
	Ft_App_WrCoCmd_Buffer(phost, VERTEX2II(xcoord, ycoord, 0, 0));
	Ft_App_WrCoCmd_Buffer(phost, DISPLAY());  //ends the display all commands after this ignored
	Ft_Gpu_CoCmd_Swap(phost);
	Ft_App_Flush_Co_Buffer(phost);
	Ft_Gpu_Hal_WaitCmdfifo_empty(phost);
	if (r==1) r=0;
	else r=1;
	//r=(r^1);
}



