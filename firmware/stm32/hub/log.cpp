/*
 * log.cpp
 *
 *  Created on: Feb 13, 2023
 *      Author: robal
 */

#include "unistd.h"
#include <cstring>
#include "log.hpp"

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
