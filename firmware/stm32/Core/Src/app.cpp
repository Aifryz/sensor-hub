/*
 * app.cpp
 *
 *  Created on: Dec 4, 2022
 *      Author: Robert Bicz
 */

#include "main.h"
#include "usart.h"
#include "spi.h"
#include "gpio.h"
#include <cstdio>

#include "FreeRTOS.h"
#include "task.h"

#include "log.hpp"
#include "nrf_radio.hpp"

extern "C" void SystemClock_Config(void);

extern "C" int main(void)
{
	HAL_Init();
	init_nrf_radio();

	SystemClock_Config();
	MX_USART2_UART_Init();
	MX_GPIO_Init();
	MX_SPI1_Init();
	MX_SPI4_Init();
	MX_SPI5_Init();
	logging::log("Hi?");
	logging::log("This is a number: {}", 3);
	logging::log("Bytes, rx {}, tx {}", 3, 5);

	vTaskStartScheduler();

	while (1)
	{

	}
}
