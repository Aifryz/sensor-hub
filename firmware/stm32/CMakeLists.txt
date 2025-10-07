cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME stm32)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Add FreeRTOS sources
add_library(freertos_config INTERFACE)
target_include_directories(freertos_config SYSTEM
       INTERFACE hub/config/) # The config file directory

target_compile_definitions(freertos_config
       INTERFACE
       projCOVERAGE_TEST=0)

set(FREERTOS_PORT GCC_ARM_CM4F CACHE STRING "")
add_subdirectory(FreeRTOS-Kernel)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    
    hub/board_main.cpp
    hub/log.cpp
    hub/drivers/mcu/spi.cpp
    hub/drivers/mcu/usart.cpp
    hub/drivers/peripheral/ili9341.cpp
    hub/system/syscalls.cpp
    hub/system/heap_4.c
    hub/system/system_hooks.cpp

    hub/tasks/lcd_display.cpp
    hub/tasks/nrf_radio.cpp

)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    FreeRTOS-Kernel/include/
    hub/
    hub/config/
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    freertos_kernel

    # Add user defined libraries
)
