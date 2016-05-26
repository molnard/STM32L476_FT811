/*
 * _write.c
 *
 *  Created on: Apr 21, 2016
 *      Author: molnard
 */
#include "stm32l4xx.h"
#include "main.h"

int _write(int fd, char *str, int len)
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart2, (uint8_t *)str, len, 0xFFFF);

  return len;
}
