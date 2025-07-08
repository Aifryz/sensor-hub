/*
 * log.hpp
 *
 *  Created on: Feb 13, 2023
 *      Author: robal
 */

#ifndef INC_LOG_HPP_
#define INC_LOG_HPP_

#include <cstring>
#include <charconv>
#include <mutex>

namespace logging
{
	namespace impl
	{
		void log_part(const char* fmt, size_t beg, size_t end);

		template<class T>
		const char* log_var(const char* spec, T var)
		{
			log_part("???", 0, 3);
			return spec+2;
		}

		template<>
		const char* log_var(const char* spec, int var)
		{
			char buf[16];
			std::to_chars_result x = std::to_chars(buf, buf+16, var);
			int n = x.ptr-buf;

			log_part(buf, 0, n);
			const char* fmt_end = spec;
			while (*fmt_end != '\0')
			{
				if(*fmt_end == '}' && *(fmt_end+1) != '}'){
					fmt_end++;
					break;
				}
				fmt_end++;
			}
			return fmt_end;
		}

		void log(const char* spec, size_t off)
		{
			size_t len = strlen(spec);
			log_part(spec, off, len);
		}

		template<class U, class...T>
		void log(const char* fmt, size_t off, U arg, T...args)
		{
			size_t beg = off;
			size_t end = off;
			while(fmt[end] != '\0')
			{
				if(fmt[end] == '{' && fmt[end+1] != '{'){
					break;
				}
				end++;
			}
			//Found { or NUL, log preceding
			log_part(fmt, beg, end);
			//Format the data
			const char* spec_end = log_var(fmt+end, arg);
			//Format rest
			log(spec_end, off, args...);
		}

	}

	template<class...T>
	void log(const char* fmt, T...args)
	{
		size_t start = 0U;
		impl::log(fmt, start, args...);
	}


}


#endif /* INC_LOG_HPP_ */
