#pragma once
#include <cstdint>
#include <cstddef>
#include <utility>

/* A simple linear fifo stream buffer implementation.
 * 
 *
 */
class fifo_message_buffer
{
public:
    fifo_message_buffer(uint8_t* buffer, size_t buffer_size);

    /// Writes data to the buffer, the data is copied into the buffer
    /// Returns true if the data was written successfully, false if there was not enough space in the buffer
    /// The whole data is written or nothing is written, there is no partial write
    bool write(const uint8_t* data, size_t len);

    /// Reads next contiguous data from the buffer, returns a pointer to the data and the length of the contiguous data
    std::pair<const uint8_t*, size_t> read_next();

    /// Releases the read data, this must be called after read_next to free up space in the buffer
    void release();

private:

    size_t get_next_read_size() const;
    size_t determine_message_header_size(size_t buffer_size) const;

    size_t read_message_length(size_t header_index) const;
    void write_message_length(size_t header_index, size_t message_length);


    uint8_t* m_buffer = nullptr;
    size_t m_buffer_size;

    size_t m_write_index;
    size_t m_last_write_index;
    size_t m_read_index;
    bool m_full;

    const size_t m_message_header_size;
};
