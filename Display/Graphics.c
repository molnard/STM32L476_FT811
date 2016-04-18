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
#ifdef FT900_PLATFORM
//TUARTBasicConfig     UART0Config;
//TSPIBasicConfig      SPIMConfig;

#endif

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
#ifdef ARDUINO_PLATFORM
   Ft_Gpu_Hal_WrCmd32(phost,cmd);
#endif
#ifdef FT900_PLATFORM
   Ft_Gpu_Hal_WrCmd32(phost,cmd);
#endif
   Ft_Gpu_Hal_WrCmd32(phost,cmd);
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

#ifdef ARDUINO_PLATFORM
   Ft_Gpu_Hal_Wr32(phost,(RAM_DL+Ft_DlBuffer_Index),cmd);
#endif
#ifdef FT900_PLATFORM
   Ft_Gpu_Hal_Wr32(phost,(RAM_DL+Ft_DlBuffer_Index),cmd);
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

//Use an MCU SPI clock of not more than 30MHz

//	/* make the spi to quad mode - addition 2 bytes for silicon */
//#ifdef FT_81X_ENABLE
//	/* api to set quad and numbe of dummy bytes */
//#ifdef ENABLE_SPI_QUAD
//	Ft_Gpu_Hal_SetSPI(phost,FT_GPU_SPI_QUAD_CHANNEL,FT_GPU_SPI_TWODUMMY);
//#elif ENABLE_SPI_DUAL
//	Ft_Gpu_Hal_SetSPI(phost,FT_GPU_SPI_DUAL_CHANNEL,FT_GPU_SPI_TWODUMMY);
//#else
//	Ft_Gpu_Hal_SetSPI(phost,FT_GPU_SPI_SINGLE_CHANNEL,FT_GPU_SPI_ONEDUMMY);
//#endif
//
//#endif

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



