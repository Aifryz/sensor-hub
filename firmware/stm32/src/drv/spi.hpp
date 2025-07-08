/*
 * spi.hpp
 *
 *  Created on: Apr 3, 2023
 *      Author: robal
 */

#ifndef SPI_HPP_
#define SPI_HPP_

#include <cstddef>
#include <system_error>

#include "FreeRTOS.h"
#include "semphr.h"

#include "spi.h"

/*
 * RTOS aware SPI driver
 */

class spi
{
public:
	spi(SPI_HandleTypeDef* handle);
	int transfer(const std::byte* tx_bytes, std::byte* rx_bytes, size_t length);
	int send(const std::byte* tx_bytes, size_t length);
	int receive(std::byte* rx_bytes, size_t length);

private:
	SPI_HandleTypeDef* handle;
	TaskHandle_t waiting_task;
};

static spi lcd_spi{&hspi1};
static spi nrf_spi{&hspi4};
static spi eth_spi{&hspi5};


#endif /* SPI_HPP_ */
