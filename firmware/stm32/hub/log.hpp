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

#include "FreeRTOS.h"
#include "semphr.h"

namespace logging
{
	namespace impl
	{
        // User facing log stream object 
        class log_stream
        {
        public:

            void write(const char* data, size_t len);

            std::pair<const uint8_t*, size_t> get_contiguous_data();
            
        private:
            constexpr static size_t m_buffer_size = 256;

            std::array<uint8_t, m_buffer_size> m_buffer;
            size_t m_write_pos = 0;
            size_t m_capacity = m_buffer_size;
            bool m_overflowed;

            friend class logger;
        };

        class logger
        {
            public:
            void init();
            log_stream& get_stream();
            void lock();
            void unlock();
            

            private:
            SemaphoreHandle_t m_mutex;
            log_stream m_stream;
            
        };

        void write_data(const char* data, size_t len);


		template<class T>
		inline const char* log_var([[maybe_unused]] log_stream& stream, [[maybe_unused]] const char* spec, [[maybe_unused]] T var)
		{
			static_assert(false, "Unsupported type for logging");
			return nullptr;
		}

        template<>
        inline const char* log_var(log_stream& stream, const char* spec, const char* var)
        {
            stream.write(var, strlen(var));
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

        template<>
        inline const char* log_var(log_stream& stream, const char* spec, char var)
        {
            char buf[2] = {var, '\0'};
            stream.write(buf, 1);
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

        template<>
        inline const char* log_var(log_stream& stream, const char* spec, uint32_t var)
        {
            char buf[16];
            std::to_chars_result x = std::to_chars(buf, buf+16, var);
            int n = x.ptr-buf;

            stream.write(buf, n);
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

		template<>
		inline const char* log_var(log_stream& stream, const char* spec, int var)
		{
			char buf[16];
			std::to_chars_result x = std::to_chars(buf, buf+16, var);
			int n = x.ptr-buf;

            stream.write(buf, n);
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

		inline void log(log_stream& stream, const char* spec, size_t off)
		{
			size_t len = strlen(spec);
			stream.write(spec+off, len-off);
		}

		template<class U, class...T>
		void log(log_stream& stream, const char* fmt, size_t off, U arg, T...args)
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
            stream.write(fmt+beg, end-beg);
			//Format the data
			const char* spec_end = log_var(stream, fmt+end, arg);
			//Format rest
			log(stream, spec_end, off, args...);
		}

        logger& get_logger();

	}

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
