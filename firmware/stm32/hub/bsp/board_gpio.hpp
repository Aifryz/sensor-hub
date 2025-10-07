/*
 * board_gpio.hpp
 *
 *  Created on: 5 kwi 2023
 *      Author: robal
 */

#ifndef BOARD_GPIO_HPP_
#define BOARD_GPIO_HPP_

#include <drivers/mcu/gpio.hpp>

using sys_led_pin = pin<GPIOB_BASE, 2>;
using sys_reset_pin = pin<GPIOA_BASE, 4>;

using lcd_cs_pin = pin<GPIOA_BASE, 15>;
using lcd_dcrs_pin = pin<GPIOB_BASE, 6>;

using nrf_cs_pin = pin<GPIOB_BASE, 12>;
using nrf_ce_pin = pin<GPIOA_BASE, 0>;



#endif /* BOARD_GPIO_HPP_ */
