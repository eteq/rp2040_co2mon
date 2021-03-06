cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(co2mon_project C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(PICO_BOARD adafruit_feather_rp2040)

pico_sdk_init()

add_executable(co2mon)

pico_generate_pio_header(co2mon ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio)

target_sources(co2mon PRIVATE co2mon.c)

pico_enable_stdio_usb(co2mon 1)
pico_enable_stdio_uart(co2mon 1)

pico_add_extra_outputs(co2mon)

target_link_libraries(co2mon
                      pico_stdlib
                      hardware_adc
                      hardware_dma
                      hardware_i2c
                      hardware_flash
                      hardware_sync
                      hardware_pio
                     )