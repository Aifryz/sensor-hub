#include "fifo_message_buffer.hpp"

#include <algorithm>
#include <cstring>

fifo_message_buffer::fifo_message_buffer(uint8_t* buffer, size_t buffer_size)
    : m_buffer(buffer)
    , m_buffer_size(buffer_size)
    , m_write_index(0)
    , m_read_index(0)
    , m_last_write_index(0)
    , m_full(false)
    , m_message_header_size(determine_message_header_size(buffer_size))
{
    // Ensure the message header is initialized to 0
    // No need to clear the remaining buffer
    std::fill(m_buffer, m_buffer + m_message_header_size, 0); 
}

bool fifo_message_buffer::write(const uint8_t* data, size_t len)
{
    if(m_full)
        return false;

    size_t total_message_size = m_message_header_size + len;

    size_t remaining_capacity = (m_write_index >= m_read_index) 
        ? (m_buffer_size - m_write_index) 
        : (m_read_index - m_write_index);

    // Check if there's place in the back buffer
    if (total_message_size <= remaining_capacity)
    {
        write_message_length(m_write_index, len);
        size_t data_index = m_write_index + m_message_header_size;
        std::memcpy(&m_buffer[data_index], data, len);
        m_write_index += total_message_size;
        if (m_write_index == m_buffer_size) m_write_index = 0;
        if (m_write_index == m_read_index) m_full = true;
        return true;
    }

    // else - check if there's place in the front buffer
    size_t front_space = m_read_index;

    if (total_message_size <= front_space) 
    {
        // There was not enough space at the end of the buffer, but there is enough space at the front of the buffer
        m_last_write_index = m_write_index; // Save the last write index for wrap-around
        
        write_message_length(0, len);
        std::memcpy(&m_buffer[m_message_header_size], data, len);
        m_write_index = total_message_size;
        if (m_write_index == m_buffer_size) m_write_index = 0;
        if (m_write_index == m_read_index) m_full = true;
        return true;
    }

    // No place otherwise
    return false;
}

std::pair<const uint8_t*, size_t> fifo_message_buffer::read_next()
{
    size_t message_length = get_next_read_size();
    if (message_length == 0)
        return {nullptr, 0}; // Buffer is empty

    size_t data_index = (m_read_index + m_message_header_size);
    return {&m_buffer[data_index], message_length};
}

void fifo_message_buffer::release()
{
    size_t message_length = get_next_read_size();
    if (message_length == 0)
        return; // Buffer is empty

    size_t total_message_size = m_message_header_size + message_length;
    m_read_index += total_message_size;

    if(m_read_index == m_buffer_size)
        m_read_index = 0;

    // If read caught up to write, we can reset the buffer to the initial state
    if(m_read_index == m_write_index)
    {
        m_read_index = 0;
        m_write_index = 0;
        m_last_write_index = 0;
    }

    if (m_read_index == m_last_write_index)
    {
        // We've reached the end of the buffer, wrap around
        m_read_index = 0;
    }
    m_full = false;
}

size_t fifo_message_buffer::determine_message_header_size(size_t buffer_size) const
{
    // Reserve some space for the message header, which will store the length of the message
    if (buffer_size <= 0xFF)
        return 1;
    else if (buffer_size <= 0xFFFF)
        return 2;
    else if (buffer_size <= 0xFFFFFF)
        return 3;
    else
        return 4;
}

size_t fifo_message_buffer::get_next_read_size() const
{
    if (m_write_index == m_read_index && !m_full)
        return 0; // Buffer is empty

    size_t header_index = m_read_index;

    return read_message_length(header_index);
}

size_t fifo_message_buffer::read_message_length(size_t header_index) const
{
    size_t message_length = 0;
    for (size_t i = 0; i < m_message_header_size; ++i)
    {
        message_length |= static_cast<size_t>(m_buffer[header_index]) << (8 * i);
        header_index = (header_index + 1) % m_buffer_size;
    }
    return message_length;
}

void fifo_message_buffer::write_message_length(size_t header_index, size_t message_length)
{
    for (size_t i = 0; i < m_message_header_size; ++i)
    {
        m_buffer[header_index] = static_cast<uint8_t>(message_length >> (8 * i));
        header_index = (header_index + 1) % m_buffer_size;
    }
}