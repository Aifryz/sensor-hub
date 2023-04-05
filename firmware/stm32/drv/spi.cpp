/*
 * spi.cpp
 *
 *  Created on: Apr 3, 2023
 *      Author: robal
 */

#include "drv/spi.hpp"




spi::spi(SPI_HandleTypeDef* spi):
	handle(spi){

}

int spi::transfer(const std::byte* tx_bytes, std::byte* rx_bytes, size_t length)
{
	const uint8_t* ctx = reinterpret_cast<const uint8_t*>(tx_bytes);
	uint8_t* rx = reinterpret_cast<uint8_t*>(rx_bytes);
	// ST does not respect const correctnes
	uint8_t* tx = const_cast<uint8_t*>(ctx);
	HAL_SPI_TransmitReceive(handle, tx, rx, length, 100);
	return 0;
}
