/*
 * syscalls.cpp
 *
 *  Created on: 4 mar 2023
 *      Author: robal
 */

/*
 * Instead of writing implementations directly in the syscalls.c file,
 *  we define them here so we can use nice C++ features
 */

#include <drivers/mcu/usart.hpp>

extern "C" int _write(int file, char *ptr, int len)
{
	debug_uart.send(reinterpret_cast<const std::byte*>(ptr), len);
	return len;
}
