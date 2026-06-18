/*
 * log.cpp
 *
 *  Created on: Feb 13, 2023
 *      Author: robal
 */

#include "unistd.h"
#include <cstring>
#include "log.hpp"
#include <cctype>

namespace logging::impl{

void write_data(int fd, const char* data, size_t len)
{
    write(fd, data, len);
}

void write_data(const char* data, size_t len)
{
    write(STDOUT_FILENO, data, len);
}

log_stream& logger::get_stream()
{
    return m_stream;
}

void logger::init()
{
    m_mutex = xSemaphoreCreateMutex();
}

void logger::lock()
{
    xSemaphoreTake(m_mutex, portMAX_DELAY);
}

void logger::unlock()
{
    xSemaphoreGive(m_mutex);
}

format_spec::format_spec()
    : width(0), precision(0), align(alignment_type::none), sign(sign_type::none)
{
}

namespace 
{
    constexpr bool is_align(char c)
    {
        return c == '<' || c == '>' || c == '^';
    }

    constexpr bool is_sign(char c)
    {
        return c == '+' || c == '-' || c == ' ';
    }
}

format_spec format_spec::parse(std::string_view spec_str)
{
    size_t pos = 0;
    auto peek = [&spec_str](size_t pos) -> char
    {
        if(pos < spec_str.size())
        {
            return spec_str[pos];
        }
        else
        {
            return '\0';
        } 
    };
    char maybe_fill = peek(pos);
    char maybe_align = peek(pos + 1); 
    if(is_align(maybe_align))
    {
        pos += 2;
    }
    else if(is_align(maybe_fill))
    {
        pos += 1;
        maybe_align = maybe_fill;
        maybe_fill = ' ';
    }
    else
    {
        maybe_align = '\0';
        maybe_fill = ' ';
    }

    char maybe_sign = peek(pos);
    if(is_sign(maybe_sign))
    {
        pos += 1;
    }
    else
    {
        maybe_sign = '-';
    }

    char maybe_alt = peek(pos);
    if(maybe_alt == '#')
    {
        pos += 1;
    }
    else
    {
        maybe_alt = '\0';
    }

    char maybe_zero = peek(pos);
    if(maybe_zero == '0')
    {
        pos += 1;
    }
    else 
    {
        maybe_zero = '\0';
    } 

    size_t width = 0;
    size_t precision = 0;
    char maybe_digit = peek(pos);
    while(std::isdigit(maybe_digit))
    {
        width = width * 10 + (maybe_digit - '0');
        pos += 1;
        maybe_digit = peek(pos);
    }

    if(maybe_digit == '.')
    {
        pos += 1;
        maybe_digit = peek(pos);
        while(std::isdigit(maybe_digit))
        {
            precision = precision * 10 + (maybe_digit - '0');
            pos += 1;
            maybe_digit = peek(pos);
        }
    }

    if(maybe_digit == 'L')
    {
        // we don't support locale, so just ignore
        pos += 1;
    }

    char maybe_type = peek(pos);

    // Finally, construct the format spec
    format_spec spec;
    spec.fill = maybe_fill;
    spec.align = static_cast<format_spec::alignment_type>(
        maybe_align == '<' ? format_spec::alignment_type::left : 
        maybe_align == '>' ? format_spec::alignment_type::right : 
        maybe_align == '^' ? format_spec::alignment_type::center : format_spec::alignment_type::none);
    spec.sign = static_cast<format_spec::sign_type>(
        maybe_sign == '+' ? format_spec::sign_type::plus : 
        maybe_sign == '-' ? format_spec::sign_type::minus : 
        maybe_sign == ' ' ? format_spec::sign_type::space : format_spec::sign_type::none);
    spec.alt_form = maybe_alt == '#' ? true : false;
    spec.zero_pad = maybe_zero == '0' ? true : false;
    spec.width = width;
    spec.precision = precision;
    spec.type = maybe_type;
    
    return spec;
}

void log_stream::write(const char* data, size_t len)
{
    if(len > m_capacity)
    {
        m_overflowed = true;
    }

    size_t end = m_write_pos + len;

    if(end > m_buffer_size)
    {
        size_t first_part = m_buffer_size - m_write_pos;
        std::memcpy(m_buffer.data() + m_write_pos, data, first_part);
        
        size_t second_part = len - first_part;
        std::memcpy(m_buffer.data(), data + first_part, second_part);
        m_write_pos = second_part;
    }
    else
    {
        std::memcpy(m_buffer.data() + m_write_pos, data, len);
        m_write_pos += len;
    }

    m_capacity -= len;
}

std::pair<const uint8_t*, size_t> log_stream::get_contiguous_data()
{

    size_t read_pos = (m_write_pos + m_capacity) % m_buffer_size;
    if(read_pos == m_write_pos)
    {
        return {nullptr, 0};
    }
    else if(read_pos < m_write_pos)
    {
        m_capacity += m_write_pos - read_pos;
        return {m_buffer.data() + read_pos, m_write_pos - read_pos};
    }
    else
    {
        m_capacity += m_buffer_size - read_pos;
        return {m_buffer.data() + read_pos, m_buffer_size - read_pos};
    }

}

logger logger_instance;

logger& get_logger()
{
    return logger_instance;
}


}
