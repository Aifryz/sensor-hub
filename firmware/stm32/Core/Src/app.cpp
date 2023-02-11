/*
 * app.cpp
 *
 *  Created on: Dec 4, 2022
 *      Author: Robert Bicz
 */

#include "main.h"
#include "usart.h"
#include "stdio.h"

extern "C" int main(void)
{
	HAL_Init();

	SystemClock_Config();
	MX_USART2_UART_Init();

	HAL_UART_Transmit(&huart2, (uint8_t*)"Hello\r\n", 7, 100);


	while (1)
	{

	}
}
