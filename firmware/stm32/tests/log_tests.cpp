#include <string_view>

#include <catch2/catch_test_macros.hpp>

#include "logging/log.hpp"

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

TEST_CASE("log formatting handles no arguments", "[logging]")
{
    log_stream stream;

    log(stream, "just some args", 0);

    const auto [data, len] = stream.get_contiguous_data();

    REQUIRE(data != nullptr);
    std::string_view expected("just some args");
    std::string_view actual(reinterpret_cast<const char*>(data), len);

    REQUIRE(actual == expected);
}

TEST_CASE("log formatting handles more arguments than spec", "[logging]")
{
    log_stream stream;

    log(stream, "temp={}", 0, 21, 42, "extra");

    const auto [data, len] = stream.get_contiguous_data();

    REQUIRE(data != nullptr);
    std::string_view expected("temp=21");
    std::string_view actual(reinterpret_cast<const char*>(data), len);

    REQUIRE(actual == expected);
}

TEST_CASE("log formatting handles more specs than arguments", "[logging]")
{
    log_stream stream;

    log(stream, "temp={} other={}, some more={}", 0, 21);

    const auto [data, len] = stream.get_contiguous_data();

    REQUIRE(data != nullptr);
    std::string_view expected("temp=21 other={}, some more={}");
    std::string_view actual(reinterpret_cast<const char*>(data), len);

    REQUIRE(actual == expected);
}

TEST_CASE("log uint8_t", "[logging]")
{
    log_stream stream;

    log(stream, "temp={}", 0, static_cast<uint8_t>(21));

    const auto [data, len] = stream.get_contiguous_data();

    REQUIRE(data != nullptr);
    REQUIRE(len == 7);
    REQUIRE(std::string_view(reinterpret_cast<const char*>(data), len) == "temp=21");
}

TEST_CASE("log uint16_t", "[logging]")
{
    log_stream stream;

    log(stream, "temp={}", 0, static_cast<uint16_t>(21));

    const auto [data, len] = stream.get_contiguous_data();

    REQUIRE(data != nullptr);
    REQUIRE(len == 7);
    REQUIRE(std::string_view(reinterpret_cast<const char*>(data), len) == "temp=21");
}

TEST_CASE("log uint32_t", "[logging]")
{
    log_stream stream;

    log(stream, "temp={}", 0, static_cast<uint32_t>(21));

    const auto [data, len] = stream.get_contiguous_data();

    REQUIRE(data != nullptr);
    REQUIRE(len == 7);
    REQUIRE(std::string_view(reinterpret_cast<const char*>(data), len) == "temp=21");
}

TEST_CASE("log int8_t", "[logging]")
{
    log_stream stream;

    log(stream, "temp={}", 0, static_cast<int8_t>(21));

    const auto [data, len] = stream.get_contiguous_data();

    REQUIRE(data != nullptr);
    REQUIRE(len == 7);
    REQUIRE(std::string_view(reinterpret_cast<const char*>(data), len) == "temp=21");
}

TEST_CASE("log int16_t", "[logging]")
{
    log_stream stream;

    log(stream, "temp={}", 0, static_cast<int16_t>(21));

    const auto [data, len] = stream.get_contiguous_data();

    REQUIRE(data != nullptr);
    REQUIRE(len == 7);
    REQUIRE(std::string_view(reinterpret_cast<const char*>(data), len) == "temp=21");
}

TEST_CASE("log int32_t", "[logging]")
{
    log_stream stream;

    log(stream, "temp={}", 0, static_cast<int32_t>(21));

    const auto [data, len] = stream.get_contiguous_data();

    REQUIRE(data != nullptr);
    REQUIRE(len == 7);
    REQUIRE(std::string_view(reinterpret_cast<const char*>(data), len) == "temp=21");
}

TEST_CASE("log char", "[logging]")
{
    log_stream stream;

    log(stream, "temp={}", 0, 'A');

    const auto [data, len] = stream.get_contiguous_data();

    REQUIRE(data != nullptr);
    REQUIRE(len == 6);
    REQUIRE(std::string_view(reinterpret_cast<const char*>(data), len) == "temp=A");
}

TEST_CASE("parse format", "[logging]")
{
    logging::impl::format_spec spec = logging::impl::format_spec::parse("_< 10.2d");

    REQUIRE(spec.width == 10);
    REQUIRE(spec.precision == 2);
    REQUIRE(spec.align == logging::impl::format_spec::alignment_type::left);
    REQUIRE(spec.sign == logging::impl::format_spec::sign_type::space);
    REQUIRE(spec.type == 'd');
    REQUIRE(spec.fill == '_');

    spec = logging::impl::format_spec::parse("#02x");
    REQUIRE(spec.width == 2);
    REQUIRE(spec.precision == 0);
    REQUIRE(spec.align == logging::impl::format_spec::alignment_type::none);
    REQUIRE(spec.sign == logging::impl::format_spec::sign_type::minus);
    REQUIRE(spec.type == 'x');
    REQUIRE(spec.fill == ' '); // By default fill is space, but align is none, so fill is ignored
    REQUIRE(spec.alt_form == true);
    REQUIRE(spec.zero_pad == true);
}