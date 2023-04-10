/*
 * board_gpio.hpp
 *
 *  Created on: 5 kwi 2023
 *      Author: robal
 */

#ifndef BOARD_GPIO_HPP_
#define BOARD_GPIO_HPP_

#include "drv/gpio.hpp"

using user_led_pin = pin<GPIOC_BASE, 13>;
using user_button_pin = pin<GPIOA_BASE, 0>;

using lcd_cs_pin = pin<GPIOA_BASE, 4>;
using lcd_dcrs_pin = pin<GPIOB_BASE, 0>;
using lcd_reset_pin = pin<GPIOB_BASE, 1>;
using lcd_led_pin = pin<GPIOB_BASE, 2>;

using nrf_cs_pin = pin<GPIOB_BASE, 12>;
using nrf_ce_pin = pin<GPIOA_BASE, 8>;

using sd_cs_pin = pin<GPIOB_BASE, 5>;



#endif /* BOARD_GPIO_HPP_ */
