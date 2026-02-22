# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD waveshare_rp2040_lcd_0.96 CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(SCD41 C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(SCD41 
    SCD41.c 
    scd4x/sensirion_common.c
    scd4x/sensirion_i2c_hal.c
    scd4x/sensirion_i2c.c
    scd4x/scd4x_i2c.c
)

pico_set_program_name(SCD41 "SCD41")
pico_set_program_version(SCD41 "0.1")

# Generate PIO header
pico_generate_pio_header(SCD41 ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio)

# And add blink.pio or remove it? The previous code had blink.pio.
# Let's keep blink.pio if it was there or just switch to ws2812.
# The user's code previously included "blink.pio.h". Now we switch to ws2812.pio.h.
# I will replace blink.pio with ws2812.pio in cmake.
# Also update the executable sources if I decided to keep blink.pio? 
# The user doesn't seem to care about blinking an LED on pin 25 anymore.
# Let's remove blink.pio reference if possible or keep if user wants.
# The user said "之前的检测和打印保留", referring to I2C scanning/reading.
# I will replace blink.pio with ws2812.pio.

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(SCD41 0)
pico_enable_stdio_usb(SCD41 1)

# Add the standard library to the build
target_link_libraries(SCD41
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(SCD41 PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/scd4x
)

# Add any user requested libraries
target_link_libraries(SCD41 
        hardware_i2c
        hardware_pio
        )

pico_add_extra_outputs(SCD41)

