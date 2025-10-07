/*
 * usart.hpp
 *
 *  Created on: 4 mar 2023
 *      Author: robal
 */

#ifndef USART_HPP_
#define USART_HPP_

#include <cstddef>

#include "FreeRTOS.h"
#include "semphr.h"

#include "usart.h"

/*
 * RTOS aware USART driver
 */

class usart
{
public:
	usart(UART_HandleTypeDef* handle);
	void send(const std::byte* buf, size_t num);
private:
	UART_HandleTypeDef* handle;
	TaskHandle_t waiting_task;
};

static usart debug_uart{&huart2};




#endif /* USART_HPP_ */
