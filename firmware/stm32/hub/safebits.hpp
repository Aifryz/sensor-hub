/*
 * safebits.hpp
 *
 *  Created on: 16 maj 2023
 *      Author: robal
 */

#ifndef INC_SAFEBITS_HPP_
#define INC_SAFEBITS_HPP_

namespace safebits
{

template<typename T, T reg_addr, typename addres_space_tag>
struct reg
{
	//Register definition
	template<T bmask, T bvalue>
	struct bits
	{
		static constexpr T value = bvalue;
		static constexpr T mask = bmask;

		template <T obm, T obv >
		constexpr auto operator|([[maybe_unused]] bits<obm, obv> other) const
		{
			static_assert(obm != bmask, "Cannot set differing values for the same bitmask");
			static_assert(obm);
			return bits<obm|bmask, obv|bvalue>{};
		}
	};

	template<T bmask, T bvalue, T min_value, T max_value>
	struct bits_range
	{
		static constexpr T value = bvalue;
		static constexpr T mask = bmask;
		static_assert(bvalue >= min_value);
		static_assert(bvalue <= max_value);
	};

	T value;
};


}





#endif /* INC_SAFEBITS_HPP_ */
