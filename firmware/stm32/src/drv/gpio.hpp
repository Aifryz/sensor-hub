/*
 * gpio.hpp
 *
 *  Created on: 5 kwi 2023
 *      Author: robal
 */

#ifndef GPIO_HPP_
#define GPIO_HPP_

#include "gpio.h"

template<uintptr_t base, int num>
struct pin
{
	static void set()
	{
		constexpr uint32_t mask = 1<<num;
		reinterpret_cast<GPIO_TypeDef*>(base)->BSRR |= mask;
	}
	static void clear()
	{
		constexpr uint32_t mask = 1<<(num+16);
		reinterpret_cast<GPIO_TypeDef*>(base)->BSRR |= mask;
	}

	static bool get()
	{
		constexpr uint32_t mask = 1<<(num);
		return (reinterpret_cast<GPIO_TypeDef*>(base)->IDR & mask) == mask;
	}

	static void toggle()
	{
		constexpr uint32_t mask = 1<<(num);
		bool is_high = (reinterpret_cast<GPIO_TypeDef*>(base)->ODR&mask) == mask;
		if(is_high){
			clear();
		} else {
			set();
		}
	}
};

#endif /* GPIO_HPP_ */
