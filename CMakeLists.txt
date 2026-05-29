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
set(CMAKE_PROJECT_NAME f4oled)


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

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/Core/Src/btn.c
    ${CMAKE_SOURCE_DIR}/Core/Src/adc3_uart_report.c
    ${CMAKE_SOURCE_DIR}/Core/Src/angle_sensor.c
    ${CMAKE_SOURCE_DIR}/Core/Src/balance_control.c
    ${CMAKE_SOURCE_DIR}/Core/Src/doji.c
    ${CMAKE_SOURCE_DIR}/Core/Src/Emm_V5.c
    ${CMAKE_SOURCE_DIR}/Core/Src/encoder.c
    ${CMAKE_SOURCE_DIR}/Core/Src/gray.c
    ${CMAKE_SOURCE_DIR}/Core/Src/hcsr04.c
    ${CMAKE_SOURCE_DIR}/Core/Src/moter.c
    ${CMAKE_SOURCE_DIR}/Core/Src/OLED.c
    ${CMAKE_SOURCE_DIR}/Core/Src/pid.c
    ${CMAKE_SOURCE_DIR}/Core/Src/atk_ms53l0m.c
    ${CMAKE_SOURCE_DIR}/Core/Src/atk_ms53l0m_uart.c
    ${CMAKE_SOURCE_DIR}/Core/Src/jy61p.c
    ${CMAKE_SOURCE_DIR}/Core/Src/shijue.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)



