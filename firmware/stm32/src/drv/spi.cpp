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

int spi::transfer(const uint8_t* tx_bytes, uint8_t* rx_bytes, size_t length)
{
	// ST does not respect const correctnes
	uint8_t* tx = const_cast<uint8_t*>(tx_bytes);
	HAL_StatusTypeDef ret = HAL_SPI_TransmitReceive(handle, tx, rx_bytes, length, 100);
	return 0;
}
