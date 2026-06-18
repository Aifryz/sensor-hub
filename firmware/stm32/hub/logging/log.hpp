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
#include <utility>
#include <array>
#include <concepts>

#include "FreeRTOS.h"
#include "semphr.h"

#include "log_impl.hpp"

namespace logging
{
	template<class...T>
	void log(const char* fmt, T...args)
	{
        impl::log_stream& stream = impl::get_logger().get_stream();

        impl::get_logger().lock();


		size_t start = 0U;
		impl::log(stream, fmt, start, args...);

        //stream.write("\r\n", 2);

        // for now, just flush single logs
        // twice since the message may be split
        for (int i = 0; i < 2; i++)
        {
            const auto &[data, len] = stream.get_contiguous_data();
            if (data != nullptr && len > 0)
            {
                // debug_uart.send(reinterpret_cast<const std::byte*>(data), len);
                //  for now, just write to stdout
                impl::write_data(reinterpret_cast<const char *>(data), len);
            }
        }

        impl::get_logger().unlock();

	}


}


#endif /* INC_LOG_HPP_ */
