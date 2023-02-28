/*
 * log.hpp
 *
 *  Created on: Feb 13, 2023
 *      Author: robal
 */

#ifndef INC_LOG_HPP_
#define INC_LOG_HPP_


namespace log
{
	namespace impl
	{
		void log(const char* fmt, size_t off)
		{
			std::puts(fmt);
		}

		template<class U, class...T>
		void log(const char* fmt, size_t off, U arg, T...args)
		{
			log(fmt, off, args...);
		}

	}

	template<class...T>
	void log(const char* fmt, T...args)
	{
		size_t start = 0U;
		log::impl::log(fmt, start, args...);
	}


}


#endif /* INC_LOG_HPP_ */
