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
#include "lcd_display.hpp"

extern "C" void SystemClock_Config(void);

extern "C" int main(void)
{
	HAL_Init();
	init_nrf_radio();
	init_lcd_display();

	SystemClock_Config();
	MX_USART2_UART_Init();
	MX_GPIO_Init();
	MX_SPI1_Init();
	MX_SPI4_Init();
	MX_SPI5_Init();
	logging::log("Hi?");
	logging::log("This is a number: {}", 3);
	logging::log("Bytes, rx {}, tx {}", 3, 5);

	// temporary PWM here
	__HAL_RCC_GPIOB_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_PIN_10;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);

	vTaskStartScheduler();

	while (1)
	{

	}
}
