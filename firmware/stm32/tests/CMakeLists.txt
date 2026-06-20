cmake_minimum_required(VERSION 3.22)

project(host_logging_tests LANGUAGES C CXX)

include(CTest)
include(FetchContent)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

FetchContent_Declare(
    Catch2
    URL https://github.com/catchorg/Catch2/archive/refs/tags/v3.15.0.zip
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
FetchContent_MakeAvailable(Catch2)

add_executable(logging_tests
    log_tests.cpp
    fifo_message_buffer_tests.cpp
    ../hub/logging/log_impl.cpp
    ../hub/logging/fifo_message_buffer.cpp
)

target_include_directories(logging_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/mocks
    ${CMAKE_CURRENT_SOURCE_DIR}/../hub
    ${CMAKE_CURRENT_SOURCE_DIR}/../FreeRTOS-Kernel/include
)

target_link_libraries(logging_tests PRIVATE Catch2::Catch2WithMain)

add_test(NAME logging_tests COMMAND logging_tests)
