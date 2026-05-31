/*
 * app.cpp
 *
 *  Created on: Dec 4, 2022
 *      Author: Robert Bicz
 */

#include "i2c.h"
#include "main.h"
#include "stm32f4xx_hal_tim.h"
#include "usart.h"
#include "spi.h"
#include "gpio.h"
#include "tim.h"
#include <cstdio>

#include "FreeRTOS.h"
#include "task.h"

#include "log.hpp"
#include <tasks/nrf_radio.hpp>
#include <tasks/lcd_display.hpp>
#include <tasks/io_expander.hpp>
#include <bsp/board_gpio.hpp>

extern "C" void SystemClock_Config(void);

extern "C" int main(void)
{
	HAL_Init();

	init_nrf_radio();
	init_lcd_display();
	init_io_expander();
	logging::impl::get_logger().init();

	SystemClock_Config();
	MX_USART2_UART_Init();
	MX_GPIO_Init();
	MX_SPI1_Init();
	MX_SPI4_Init();
	MX_SPI5_Init();
	MX_I2C1_Init();
	MX_TIM2_Init();
	//logging::log("Hi?");
	//logging::log("This is a number: {}", 3);
	//logging::log("Bytes, rx {}, tx {}", 3, 5);

	nrf_cs_pin::set();
	nrf_ce_pin::clear();

	sys_reset_pin::set(); // Deassert reset pin

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

	//logging::log("\r\nStarting scheduler\r\n");

	vTaskStartScheduler();

	while (1)
	{

	}
}
