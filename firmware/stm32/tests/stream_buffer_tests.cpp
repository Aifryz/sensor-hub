#include <catch2/catch_test_macros.hpp>
#include "logging/fifo_buffer.hpp"

TEST_CASE("buffer exposes contiguous bytes", "[logging]")
{
    uint8_t buffer_memory[10];
    fifo_buffer buffer(buffer_memory, sizeof(buffer_memory));

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
    fifo_buffer buffer(buffer_memory, sizeof(buffer_memory));

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
    fifo_buffer buffer(buffer_memory, sizeof(buffer_memory));

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
    fifo_buffer buffer(buffer_memory, sizeof(buffer_memory));

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
    fifo_buffer stream(buffer_memory, sizeof(buffer_memory));

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
    fifo_buffer stream(buffer_memory, sizeof(buffer_memory));

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
    fifo_buffer stream(buffer_memory, sizeof(buffer_memory));

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
    fifo_buffer stream(buffer_memory, sizeof(buffer_memory));

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
    fifo_buffer stream(buffer_memory, sizeof(buffer_memory));

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
