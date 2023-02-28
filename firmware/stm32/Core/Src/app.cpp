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

void nrf_task(void* arg)
{
	while(1)
	{
		std::printf("Hi\r\n");
		vTaskDelay(500);
	}
}

extern "C" int main(void)
{
	HAL_Init();

	SystemClock_Config();
	MX_USART2_UART_Init();
	TaskHandle_t nrf_task_handle;
	BaseType_t ret = xTaskCreate(&nrf_task, "radio", 512, NULL, 7, &nrf_task_handle);
	HAL_UART_Transmit(&huart2, (const uint8_t*)"HI?", 3, 100);
	log::log("Hi?");
	log::log("This is a number: {}", 3);
	log::log("Bytes, rx {}, tx {}", 3, 5);

	vTaskStartScheduler();




	while (1)
	{

	}
}
