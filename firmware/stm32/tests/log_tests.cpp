#include <string_view>

#include <catch2/catch_test_macros.hpp>

#include "log.hpp"

using logging::impl::log;
using logging::impl::log_stream;

TEST_CASE("log_stream exposes contiguous bytes", "[logging]")
{
    log_stream stream;

    stream.write("abc", 3);
    stream.write("def", 3);

    const auto [data, len] = stream.get_contiguous_data();

    REQUIRE(data != nullptr);
    REQUIRE(len == 6);
    REQUIRE(std::string_view(reinterpret_cast<const char*>(data), len) == "abcdef");
}

TEST_CASE("log formatting writes the expected payload", "[logging]")
{
    log_stream stream;

    log(stream, "temp={}", 0, 21);

    const auto [data, len] = stream.get_contiguous_data();

    REQUIRE(data != nullptr);
    REQUIRE(len == 7);
    REQUIRE(std::string_view(reinterpret_cast<const char*>(data), len) == "temp=21");
}
