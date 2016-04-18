/**
  ******************************************************************************
  * @file   fatfs.c
  * @brief  Code for fatfs applications
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

#include "fatfs.h"

uint8_t retUSER;    /* Return value for USER */
char USER_Path[4];  /* USER logical drive path */

/* USER CODE BEGIN Variables */
FATFS mynewdiskFatFs; /* File system object for User logical drive */
FIL MyFile; /* File object */
char mynewdiskPath[4]; /* User logical drive path */
/* USER CODE END Variables */    

void MX_FATFS_Init(void) 
{
  /*## FatFS: Link the USER driver ###########################*/
  retUSER = FATFS_LinkDriver(&USER_Driver, USER_Path);

  /* USER CODE BEGIN Init */
  /* additional user code for init */
	HAL_Error_Handler(f_mount(&mynewdiskFatFs, USER_Path, 0));
	printf("drive number:%d path:%s successfully mounted\r\n",retUSER,USER_Path);
	MX_FATFS_Speedtest();
  /* USER CODE END Init */
}

/**
  * @brief  Gets Time from RTC 
  * @param  None
  * @retval Time in DWORD
  */
DWORD get_fattime(void)
{
  /* USER CODE BEGIN get_fattime */
  return 0;
  /* USER CODE END get_fattime */  
}

/* USER CODE BEGIN Application */



void delete_file(char *fname)
{
	FRESULT res;
	res = f_unlink(fname);
	if(res != FR_OK)
		printf("Error deleting file:%u\r\n", res);
	return;
}

/*
 * write_file()
 *
 * Writes a number of records to the file with the given name.
 * This function writes 50 sectors of 512 bytes
 */
void write_file_test()
{
	FIL file;
	FRESULT res;
	UINT written;
	uint32_t i, idx;

	res = f_open(&file, "test.txt", FA_CREATE_NEW | FA_WRITE);
	if(res != FR_OK)
	{
		printf("Cannot open file for writing:%d\r\n", res);
		return;
	}
	uint8_t Buffer[512];
	uint32_t elapsed=0;
	uint32_t sum_written =0;
	for(i=0; i<200; i++)
	{
		/*
		 * Fill thebuffer with some data to check during read.
		 */
		for(idx=0; idx<512; idx++)
			Buffer[idx] = ((i * 3) + (idx * 5)) & 0xff;

		uint32_t start=HAL_GetTick();
		res = f_write(&file, Buffer, 512, &written);
		uint32_t end=HAL_GetTick();
		elapsed += end-start;

		sum_written+=written;
		if(res != FR_OK)
		{
			printf("Error writing: %d\r\n", res);
			break;
		} else
		{
			if(written != 512)
			{
				printf("Write incomplete - wrote %d bytes\n", (int)written);
			}
		}
	}
	res = f_close(&file);
	if(res != FR_OK)
		printf("Error closing file: %d\r\n", res);
	else
	{


		uint32_t speed= sum_written / elapsed;
		printf("Write test complete: %u kbytes/s\n\r", (unsigned int)(speed));
	}
}


/*
 * read_file()
 *
 * Reads the file with the given name.
 * This function reads blocks of 512 bytes
 */
void read_file_test()
{
	FIL file;
	FRESULT res;
	UINT read;
	uint32_t sum_read;
	uint32_t i, idx;
	uint32_t data_err;
	res = f_open(&file, "test.txt", FA_READ);
	if(res != FR_OK)
	{
		printf("Cannot open file for reading:%u\r\n", res);
		return;
	}
	sum_read = 0;
	i=0;
	data_err = 0;
	uint8_t Buffer[512];
	uint32_t elapsed=0;
	while(!f_eof(&file))
	{
		uint32_t start=HAL_GetTick();
		res = f_read(&file, Buffer, 512, &read);
		uint32_t end = HAL_GetTick();
		elapsed+=(end-start);

		sum_read += read;

		if(res != FR_OK)
		{
			printf("Error reading file: %u", res);
			break;
		} else
		{
			/*
			 * Check data
			 */
			for(idx=0; idx<read; idx++)
			{
				if(Buffer[idx] != (((i * 3) + (idx * 5)) & 0xff)) data_err++;
			}
			i++;


			if(read != 512)
			{
				printf("Read incomplete - read %d bytes\n", (int)read);
			}
		}
	}
	if(data_err) printf("Error: data does not match expected pattern\n\r");

	res = f_close(&file);
	if(res != FR_OK)
		printf("Error closing file%u\r\n", res);


	uint32_t speed= sum_read / elapsed;
	printf("Read test complete: %u kbytes/s\n\r", (unsigned int)(speed));
	delete_file("test.txt");
}

void MX_FATFS_Speedtest(void)
{
	write_file_test();
	read_file_test();
}



/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
