#pragma once
#include <cstdint>
#include <string_view>
#include <array>
#include <charconv>

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

        struct format_spec
        {
            format_spec();

            enum class alignment_type: uint8_t
            {
                left,
                right,
                center,
                none
            };

            enum class sign_type: uint8_t
            {
                plus,
                minus,
                space,
                none
            };

            char fill;
            uint8_t width;
            uint8_t precision;

            alignment_type align: 2;
            sign_type sign: 2;
            bool alt_form: 1;
            bool zero_pad: 1;
            char type;

            static format_spec parse(std::string_view spec);
        };

        inline size_t find_format_start(const char* fmt, size_t off)
        {
            size_t end = off;
            while(fmt[end] != '\0')
			{
				if(fmt[end] == '{' && fmt[end+1] != '{'){
					break;
				}
				end++;
			}
            return end;
        }

        void write_data(const char* data, size_t len);


		template<class T>
		inline void log_var([[maybe_unused]] log_stream& stream, [[maybe_unused]] const format_spec& spec, [[maybe_unused]] T var)
		{
			static_assert(false, "Unsupported type for logging");
		}

        template<class T> requires std::integral<T>
        inline void log_var(log_stream& stream, const format_spec& spec, T var)
        {
            char buf[16];
            std::to_chars_result x = std::to_chars(buf, buf+16, var);
            int n = x.ptr-buf;
            stream.write(buf, n);
        }

        template<>
        inline void log_var(log_stream& stream, const format_spec& spec, const char* var)
        {
            stream.write(var, strlen(var));
        }

        template<>
        inline void log_var(log_stream& stream, const format_spec& spec, char var)
        {
            char buf[2] = {var, '\0'};
            stream.write(buf, 1);
        }

        template<>
        inline void log_var(log_stream& stream, const format_spec& spec, uint32_t var)
        {
            char buf[16];
            std::to_chars_result x = std::to_chars(buf, buf+16, var);
            int n = x.ptr-buf;

            stream.write(buf, n);
        }

		template<>
		inline void log_var(log_stream& stream, const format_spec& spec, int var)
		{
			char buf[16];
			std::to_chars_result x = std::to_chars(buf, buf+16, var);
			int n = x.ptr-buf;

            stream.write(buf, n);
		}

		inline void log(log_stream& stream, const char* spec, size_t off)
		{
			size_t len = strlen(spec);
			stream.write(spec+off, len-off);
		}

		template<class U, class...T>
		void log(log_stream& stream, const char* fmt, size_t off, U arg, T...args)
		{
            if(fmt == nullptr || fmt[off] == '\0')
            {
                // todo - maybe assert?
                return;
            }
			size_t beg = off;
			size_t end = find_format_start(fmt, off);
            size_t part_end = end-beg;
            const char* format_start = fmt+end;
            //find end of format specifier
            while(fmt[end] != '\0' && fmt[end] != '}')
            {
                end++;
            }
            
            const char* format_end = fmt+end;
			
			//Found { or NUL, log preceding
            stream.write(fmt+beg, part_end);
			//Format the data
            auto spec = format_spec::parse(std::string_view(format_start, format_end-format_start));
			log_var(stream, spec, arg);
			//Format rest
			log(stream, format_end+1, off, args...);
		}

        logger& get_logger();

	}
}