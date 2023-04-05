/*
 * app.cpp
 *
 *  Created on: Dec 4, 2022
 *      Author: Robert Bicz
 */

#include "main.h"
#include "usart.h"
#include <cstdio>

#include "FreeRTOS.h"
#include "task.h"

#include "log.hpp"
#include "nrf_radio.hpp"

extern "C" int main(void)
{
	HAL_Init();
	init_nrf_radio();

	SystemClock_Config();
	MX_USART2_UART_Init();
	log::log("Hi?");
	log::log("This is a number: {}", 3);
	log::log("Bytes, rx {}, tx {}", 3, 5);

	vTaskStartScheduler();

	while (1)
	{

	}
}
