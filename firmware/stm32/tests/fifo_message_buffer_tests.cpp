#include <catch2/catch_test_macros.hpp>
#include "logging/fifo_message_buffer.hpp"
#include <vector>

TEST_CASE("buffer exposes contiguous bytes", "[logging]")
{
    uint8_t buffer_memory[10];
    fifo_message_buffer buffer(buffer_memory, sizeof(buffer_memory));

    const char* data = "abcdef";
    buffer.write(reinterpret_cast<const uint8_t*>(data), 6);

    const auto [read_data, len] = buffer.read_next();

    REQUIRE(read_data != nullptr);
    REQUIRE(len == 6);
    REQUIRE(std::string_view(reinterpret_cast<const char*>(read_data), len) == "abcdef");
}

TEST_CASE("buffer handles multiple writes and reads", "[logging]")
{
    uint8_t buffer_memory[10];
    fifo_message_buffer buffer(buffer_memory, sizeof(buffer_memory));

    const char* data1 = "abc";
    const char* data2 = "defG";
    buffer.write(reinterpret_cast<const uint8_t*>(data1), 3);
    buffer.write(reinterpret_cast<const uint8_t*>(data2), 4);

    const auto [read_data, len] = buffer.read_next();

    REQUIRE(read_data != nullptr);
    REQUIRE(len == 3);
    REQUIRE(std::string_view(reinterpret_cast<const char*>(read_data), len) == "abc");

    buffer.release();

    const auto [read_data2, len2] = buffer.read_next();

    REQUIRE(read_data2 != nullptr);
    REQUIRE(len2 == 4);
    REQUIRE(std::string_view(reinterpret_cast<const char*>(read_data2), len2) == "defG");
}

TEST_CASE("buffer handles wrap around", "[logging]") 
{
    uint8_t buffer_memory[10];
    fifo_message_buffer buffer(buffer_memory, sizeof(buffer_memory));

    const char* data1 = "abcdefgh";
    buffer.write(reinterpret_cast<const uint8_t*>(data1), 8);

    const auto [read_data, len] = buffer.read_next();

    REQUIRE(read_data != nullptr);
    REQUIRE(len == 8);
    REQUIRE(std::string_view(reinterpret_cast<const char*>(read_data), len) == "abcdefgh");

    buffer.release();

    const char* data2 = "xyz";
    buffer.write(reinterpret_cast<const uint8_t*>(data2), 3);

    const auto [read_data2, len2] = buffer.read_next();

    REQUIRE(read_data2 != nullptr);
    REQUIRE(len2 == 3);
    REQUIRE(std::string_view(reinterpret_cast<const char*>(read_data2), len2) == "xyz");
}

TEST_CASE("buffer does not allow overflow", "[logging]")
{
    uint8_t buffer_memory[10];
    fifo_message_buffer buffer(buffer_memory, sizeof(buffer_memory));

    const char* data1 = "abcdefghi";
    bool result1 = buffer.write(reinterpret_cast<const uint8_t*>(data1), 9);
    REQUIRE(result1 == true);

    const char* data2 = "k";
    bool result2 = buffer.write(reinterpret_cast<const uint8_t*>(data2), 1);
    REQUIRE(result2 == false);
}

TEST_CASE("buffer writes data in parts", "[logging]")
{
    uint8_t buffer_memory[20];
    fifo_message_buffer stream(buffer_memory, sizeof(buffer_memory));

    stream.write(reinterpret_cast<const uint8_t*>("abcefgh"), 7);
    stream.write(reinterpret_cast<const uint8_t*>("ijklm"), 5);
    const auto [data, len] = stream.read_next();
    REQUIRE(data != nullptr);
    REQUIRE(len == 7);
    REQUIRE(std::string_view(reinterpret_cast<const char*>(data), len) == "abcefgh");

    stream.release();
    const auto [data2, len2] = stream.read_next();
    REQUIRE(data2 != nullptr);
    REQUIRE(len2 == 5);
    REQUIRE(std::string_view(reinterpret_cast<const char*>(data2), len2) == "ijklm");
}

TEST_CASE("buffer perfect fill", "[logging]")
{
    uint8_t buffer_memory[10];
    fifo_message_buffer stream(buffer_memory, sizeof(buffer_memory));

    REQUIRE(stream.write(reinterpret_cast<const uint8_t*>("abc"), 3));
    REQUIRE(stream.write(reinterpret_cast<const uint8_t*>("def"), 3));
    REQUIRE(stream.write(reinterpret_cast<const uint8_t*>("g"), 1));

    const auto [data, len] = stream.read_next();
    REQUIRE(data != nullptr);
    REQUIRE(len == 3);
    REQUIRE(std::string_view(reinterpret_cast<const char*>(data), len) == "abc");
    stream.release();

    const auto [data2, len2] = stream.read_next();
    REQUIRE(data2 != nullptr);
    REQUIRE(len2 == 3);
    REQUIRE(std::string_view(reinterpret_cast<const char*>(data2), len2) == "def");
    stream.release();

    const auto [data3, len3] = stream.read_next();
    REQUIRE(data3 != nullptr);
    REQUIRE(len3 == 1);
    REQUIRE(std::string_view(reinterpret_cast<const char*>(data3), len3) == "g");
    stream.release();
}

TEST_CASE("buffer perfect across wrap", "[logging]")
{
    uint8_t buffer_memory[10];
    fifo_message_buffer stream(buffer_memory, sizeof(buffer_memory));

    REQUIRE(stream.write(reinterpret_cast<const uint8_t*>("abc"), 3));
    REQUIRE(stream.write(reinterpret_cast<const uint8_t*>("def"), 3));

    const auto [data, len] = stream.read_next();
    REQUIRE(data != nullptr);
    REQUIRE(len == 3);
    REQUIRE(std::string_view(reinterpret_cast<const char*>(data), len) == "abc");
    stream.release();

    REQUIRE(stream.write(reinterpret_cast<const uint8_t*>("g"), 1)); // Should be written to back still

    const auto [data2, len2] = stream.read_next();
    REQUIRE(data2 != nullptr);
    REQUIRE(len2 == 3);
    REQUIRE(std::string_view(reinterpret_cast<const char*>(data2), len2) == "def");
    stream.release();

    REQUIRE(stream.write(reinterpret_cast<const uint8_t*>("hij"), 3));
    REQUIRE(stream.write(reinterpret_cast<const uint8_t*>("klm"), 3));

    const auto [data3, len3] = stream.read_next();
    REQUIRE(data3 != nullptr);
    REQUIRE(len3 == 1);
    REQUIRE(std::string_view(reinterpret_cast<const char*>(data3), len3) == "g");
    stream.release();

    const auto [data4, len4] = stream.read_next();
    REQUIRE(data4 != nullptr);
    REQUIRE(len4 == 3);
    REQUIRE(std::string_view(reinterpret_cast<const char*>(data4), len4) == "hij");
    stream.release();

    const auto [data5, len5] = stream.read_next();
    REQUIRE(data5 != nullptr);
    REQUIRE(len5 == 3);
    REQUIRE(std::string_view(reinterpret_cast<const char*>(data5), len5) == "klm");
    stream.release();
}

TEST_CASE("buffer write one", "[logging]")
{
    uint8_t buffer_memory[10];
    fifo_message_buffer stream(buffer_memory, sizeof(buffer_memory));

    REQUIRE(stream.write(reinterpret_cast<const uint8_t*>("abcdefghi"), 9));

    const auto [data, len] = stream.read_next();
    REQUIRE(data != nullptr);
    REQUIRE(len == 9);
    REQUIRE(std::string_view(reinterpret_cast<const char*>(data), len) == "abcdefghi");
    stream.release();

    REQUIRE(stream.write(reinterpret_cast<const uint8_t*>("abcdefghi"), 9));

    const auto [data2, len2] = stream.read_next();
    REQUIRE(data2 != nullptr);
    REQUIRE(len2 == 9);
    REQUIRE(std::string_view(reinterpret_cast<const char*>(data2), len2) == "abcdefghi");
    stream.release();
}

TEST_CASE("buffer write two", "[logging]")
{
    uint8_t buffer_memory[10];
    fifo_message_buffer stream(buffer_memory, sizeof(buffer_memory));

    REQUIRE(stream.write(reinterpret_cast<const uint8_t*>("abcd"), 4));
    REQUIRE(stream.write(reinterpret_cast<const uint8_t*>("efgh"), 4));

    const auto [data, len] = stream.read_next();
    REQUIRE(data != nullptr);
    REQUIRE(len == 4);
    REQUIRE(std::string_view(reinterpret_cast<const char*>(data), len) == "abcd");
    stream.release();

    const auto [data2, len2] = stream.read_next();
    REQUIRE(data2 != nullptr);
    REQUIRE(len2 == 4);
    REQUIRE(std::string_view(reinterpret_cast<const char*>(data2), len2) == "efgh");
    stream.release();

    REQUIRE(stream.write(reinterpret_cast<const uint8_t*>("abcd"), 4));
    REQUIRE(stream.write(reinterpret_cast<const uint8_t*>("efgh"), 4));

    const auto [data3, len3] = stream.read_next();
    REQUIRE(data3 != nullptr);
    REQUIRE(len3 == 4);
    REQUIRE(std::string_view(reinterpret_cast<const char*>(data3), len3) == "abcd");
    stream.release();

    const auto [data4, len4] = stream.read_next();
    REQUIRE(data4 != nullptr);
    REQUIRE(len4 == 4);
    REQUIRE(std::string_view(reinterpret_cast<const char*>(data4), len4) == "efgh");
    stream.release();
}

TEST_CASE("buffer handles empty read", "[logging]")
{
    uint8_t buffer_memory[10];
    fifo_message_buffer buffer(buffer_memory, sizeof(buffer_memory));

    const auto [read_data, len] = buffer.read_next();
    REQUIRE(read_data == nullptr);
    REQUIRE(len == 0);
}

TEST_CASE("buffer handles release on empty", "[logging]")
{
    uint8_t buffer_memory[10];
    fifo_message_buffer buffer(buffer_memory, sizeof(buffer_memory));

    buffer.release(); // Should not crash

    const auto [read_data, len] = buffer.read_next();
    REQUIRE(read_data == nullptr);
}

TEST_CASE("buffer handles single byte messages", "[logging]")
{
    uint8_t buffer_memory[10];
    fifo_message_buffer buffer(buffer_memory, sizeof(buffer_memory));

    buffer.write(reinterpret_cast<const uint8_t*>("a"), 1);
    buffer.write(reinterpret_cast<const uint8_t*>("b"), 1);
    buffer.write(reinterpret_cast<const uint8_t*>("c"), 1);

    for (char expected : {'a', 'b', 'c'}) {
        const auto [read_data, len] = buffer.read_next();
        REQUIRE(len == 1);
        REQUIRE(*read_data == expected);
        buffer.release();
    }
}

TEST_CASE("buffer multiple reads without release blocked by data", "[logging]")
{
    uint8_t buffer_memory[10];
    fifo_message_buffer buffer(buffer_memory, sizeof(buffer_memory));

    buffer.write(reinterpret_cast<const uint8_t*>("abc"), 3);

    const auto [read_data1, len1] = buffer.read_next();
    REQUIRE(len1 == 3);

    // Read again without release should return same data
    const auto [read_data2, len2] = buffer.read_next();
    REQUIRE(len2 == 3);
    REQUIRE(read_data1 == read_data2);

    buffer.release();
}

TEST_CASE("buffer interleaved small and large writes", "[logging]")
{
    uint8_t buffer_memory[30];
    fifo_message_buffer buffer(buffer_memory, sizeof(buffer_memory));

    buffer.write(reinterpret_cast<const uint8_t*>("x"), 1);
    buffer.write(reinterpret_cast<const uint8_t*>("abcdefgh"), 8);
    buffer.write(reinterpret_cast<const uint8_t*>("y"), 1);

    const auto [data1, len1] = buffer.read_next();
    REQUIRE(len1 == 1);
    REQUIRE(*data1 == 'x');
    buffer.release();

    const auto [data2, len2] = buffer.read_next();
    REQUIRE(len2 == 8);
    REQUIRE(std::string_view(reinterpret_cast<const char*>(data2), len2) == "abcdefgh");
    buffer.release();

    const auto [data3, len3] = buffer.read_next();
    REQUIRE(len3 == 1);
    REQUIRE(*data3 == 'y');
    buffer.release();
}

TEST_CASE("buffer wrap-around with last_write_index sentinel", "[logging]")
{
    uint8_t buffer_memory[10];
    fifo_message_buffer buffer(buffer_memory, sizeof(buffer_memory));

    // Fill to trigger wrap
    REQUIRE(buffer.write(reinterpret_cast<const uint8_t*>("abcd"), 4));
    REQUIRE(buffer.write(reinterpret_cast<const uint8_t*>("ef"), 2));

    buffer.release();
    buffer.release();

    // Write to front of buffer (wrap around)
    REQUIRE(buffer.write(reinterpret_cast<const uint8_t*>("ghij"), 4));
    REQUIRE(buffer.write(reinterpret_cast<const uint8_t*>("kl"), 2));

    const auto [data1, len1] = buffer.read_next();
    REQUIRE(std::string_view(reinterpret_cast<const char*>(data1), len1) == "ghij");
    buffer.release();

    const auto [data2, len2] = buffer.read_next();
    REQUIRE(std::string_view(reinterpret_cast<const char*>(data2), len2) == "kl");
    buffer.release();
}

TEST_CASE("buffer exact fill with multiple messages", "[logging]")
{
    uint8_t buffer_memory[10];
    fifo_message_buffer buffer(buffer_memory, sizeof(buffer_memory));

    // Exactly fill: 1 header + 4 data + 1 header + 4 data = 10 bytes
    REQUIRE(buffer.write(reinterpret_cast<const uint8_t*>("aaaa"), 4));
    REQUIRE(buffer.write(reinterpret_cast<const uint8_t*>("bbbb"), 4));
    REQUIRE(!buffer.write(reinterpret_cast<const uint8_t*>("c"), 1)); // Should fail

    buffer.release();
    buffer.release();

    // Now should be empty and we can write again
    REQUIRE(buffer.write(reinterpret_cast<const uint8_t*>("cccc"), 4));
}

TEST_CASE("buffer prevents write when full but allows after release", "[logging]")
{
    uint8_t buffer_memory[10];
    fifo_message_buffer buffer(buffer_memory, sizeof(buffer_memory));

    REQUIRE(buffer.write(reinterpret_cast<const uint8_t*>("abcdefghi"), 9));
    REQUIRE(!buffer.write(reinterpret_cast<const uint8_t*>("x"), 1)); // Should fail - buffer full

    buffer.release();

    REQUIRE(buffer.write(reinterpret_cast<const uint8_t*>("xyz"), 3)); // Should succeed now
}

TEST_CASE("buffer message length encoding for various sizes", "[logging]")
{
    uint8_t buffer_memory[50];
    fifo_message_buffer buffer(buffer_memory, sizeof(buffer_memory));

    // Test various message sizes
    for (size_t size : {1, 5, 10, 15, 20}) {
        std::vector<uint8_t> data(size, 'X');
        REQUIRE(buffer.write(data.data(), size));

        const auto [read_data, len] = buffer.read_next();
        REQUIRE(len == size);
        buffer.release();
    }
}

TEST_CASE("buffer complex wrap scenario", "[logging]")
{
    uint8_t buffer_memory[15];
    fifo_message_buffer buffer(buffer_memory, sizeof(buffer_memory));

    // Create a complex wrap pattern
    buffer.write(reinterpret_cast<const uint8_t*>("12345"), 5);
    buffer.write(reinterpret_cast<const uint8_t*>("67"), 2);

    buffer.release(); // Read first
    buffer.write(reinterpret_cast<const uint8_t*>("89"), 2); // Write in middle

    buffer.release(); // Read second
    buffer.write(reinterpret_cast<const uint8_t*>("ABC"), 3); // More writes
    buffer.write(reinterpret_cast<const uint8_t*>("DEF"), 3);

    // Verify all reads work correctly
    const auto [d1, l1] = buffer.read_next();
    REQUIRE(std::string_view(reinterpret_cast<const char*>(d1), l1) == "89");
    buffer.release();

    const auto [d2, l2] = buffer.read_next();
    REQUIRE(std::string_view(reinterpret_cast<const char*>(d2), l2) == "ABC");
    buffer.release();

    const auto [d3, l3] = buffer.read_next();
    REQUIRE(std::string_view(reinterpret_cast<const char*>(d3), l3) == "DEF");
    buffer.release();
}

TEST_CASE("buffer with 256-byte size uses 2-byte header", "[logging]")
{
    std::vector<uint8_t> buffer_memory(256);
    fifo_message_buffer buffer(buffer_memory.data(), buffer_memory.size());

    // Test that large messages can be written
    std::vector<uint8_t> large_data(200, 'A');
    REQUIRE(buffer.write(large_data.data(), large_data.size()));

    const auto [read_data, len] = buffer.read_next();
    REQUIRE(len == 200);
    buffer.release();
}

TEST_CASE("buffer wrap to front after read and release", "[logging]")
{
    // Demonstrates the wrap-around protection:
    // When wrap happens, verify data isn't corrupted
    uint8_t buffer_memory[15];
    fifo_message_buffer buffer(buffer_memory, sizeof(buffer_memory));

    // Write first message (1 header + 4 data = 5 bytes at positions 0-4)
    const char msg1[] = "AAAA";
    REQUIRE(buffer.write(reinterpret_cast<const uint8_t*>(msg1), 4));
    
    // Read it but DON'T release (data is still in positions 0-4)
    const auto [read_data1, len1] = buffer.read_next();
    REQUIRE(len1 == 4);
    
    // Write more to fill buffer (1 header + 4 data = 5 bytes at positions 5-9)
    const char msg2[] = "BBBB";
    REQUIRE(buffer.write(reinterpret_cast<const uint8_t*>(msg2), 4));
    
    // Write another (1 header + 4 data = 5 bytes at positions 10-14)
    const char msg3[] = "CCCC";
    REQUIRE(buffer.write(reinterpret_cast<const uint8_t*>(msg3), 4));
    
    // Buffer is now completely full, no wrap yet
    // Now release first message - ri advances to 5
    buffer.release();
    
    // Now we can write to the front (positions 0-4)
    // This tests that wrap-to-front logic works after release
    const char msg4[] = "DDDD";
    REQUIRE(buffer.write(reinterpret_cast<const uint8_t*>(msg4), 4));
    
    // Read remaining messages to verify no corruption
    const auto [rd2, l2] = buffer.read_next();
    REQUIRE(l2 == 4);
    REQUIRE(std::string_view(reinterpret_cast<const char*>(rd2), l2) == "BBBB");
    buffer.release();
    
    const auto [rd3, l3] = buffer.read_next();
    REQUIRE(l3 == 4);
    REQUIRE(std::string_view(reinterpret_cast<const char*>(rd3), l3) == "CCCC");
    buffer.release();
    
    const auto [rd4, l4] = buffer.read_next();
    REQUIRE(l4 == 4);
    REQUIRE(std::string_view(reinterpret_cast<const char*>(rd4), l4) == "DDDD");
    buffer.release();
}
