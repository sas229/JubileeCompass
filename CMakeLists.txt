# Generated cmake Pico project file.
cmake_minimum_required(VERSION 3.13)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Project details.
set(PROJECT JubileeCompass)
set(MAJOR_VERSION 0)
set(MINOR_VERSION 1)

# Directories.
set(BUILD "${CMAKE_SOURCE_DIR}/build")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

# Declare project.
project(${PROJECT} C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Make project data accessible to compiler
add_compile_definitions(PROJECT=${PROJECT})
add_compile_definitions(MAJOR_VERSION=${MAJOR_VERSION})
add_compile_definitions(MINOR_VERSION=${MINOR_VERSION})

# Create executable.
add_executable(${PROJECT})

# Specify sources for the executable.
target_sources(${PROJECT} PRIVATE
    src/${PROJECT}.cpp
    src/hw_config.cpp
)

# Specify link libraries.
target_link_libraries(
    ${PROJECT}
    pico_stdlib
    pico_cyw43_arch_lwip_threadsafe_background
    json
    TinyGPSPlus
    FatFs_SPI
    WS2812
    WMM
)

target_include_directories(JubileeCompass PRIVATE ${CMAKE_CURRENT_LIST_DIR}/include)

# Enable stdio outputs as required.
pico_enable_stdio_usb(${PROJECT} 1)
pico_enable_stdio_uart(${PROJECT} 0)

# Create additional executables.
pico_add_extra_outputs(${PROJECT})

# Generate PIO header for WS2812 library.
pico_generate_pio_header(JubileeCompass ${CMAKE_CURRENT_LIST_DIR}/external/WS2812/src/WS2812.pio)

# Add required subdirectories for the libraries required.
add_subdirectory(external/json)
message("json library added.")
add_subdirectory(external/TinyGPSPlus)
message("TinyGPSPlus library added.")
add_subdirectory(external/no-OS-FatFS-SD-SPI-RPi-Pico/FatFs_SPI)
message("FatFs_SPI library added.")
add_subdirectory(external/WS2812)
message("WS2812 library added.")
add_subdirectory(external/WMM)
message("WMM library added.")






