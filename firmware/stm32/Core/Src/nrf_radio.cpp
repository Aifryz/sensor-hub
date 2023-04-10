/*
 * nrf_radio.cpp
 *
 *  Created on: Apr 3, 2023
 *      Author: robal
 */

#include <cstdio>
#include <cstring>

#include "FreeRTOS.h"
#include "task.h"

#include "drv/spi.hpp"
#include "bsp/board_gpio.hpp"
void nrf_task(void* arg)
{
	nrf_cs_pin::set();
	vTaskDelay(100);
	std::byte rx_buf[4];
	std::byte tx_buf[4];
	std::memset(rx_buf, 0, 4);
	std::memset(tx_buf, 0, 4);
	nrf_cs_pin::clear();
	nrf_spi.transfer(tx_buf, rx_buf, 4);
	nrf_cs_pin::set();
	user_led_pin::set();
	while(1)
	{
		std::printf("Hi nrf\r\n");
		//user_led_pin::toggle();

		vTaskDelay(500);
	}
}


TaskHandle_t nrf_task_handle;
void init_nrf_radio()
{
	BaseType_t ret = xTaskCreate(&nrf_task, "radio", 512, NULL, 7, &nrf_task_handle);
}
