/*
 * usart.cpp
 *
 *  Created on: 4 mar 2023
 *      Author: robal
 */

#include "usart.hpp"

usart::usart(UART_HandleTypeDef* handle):
	handle(handle),
	waiting_task(nullptr)
{

}

void usart::send(const std::byte* buf, size_t num)
{
	HAL_UART_Transmit(handle, reinterpret_cast<const uint8_t*>(buf), num, 100);
}
