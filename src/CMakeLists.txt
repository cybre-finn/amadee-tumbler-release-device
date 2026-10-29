cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD pico2)

# Pico SDK importieren
include(pico_sdk_import.cmake)

project(amadee C CXX ASM)

# SDK initialisieren
pico_sdk_init()

# Firmware
add_executable(my_program main.c)

# Bibliotheken
target_link_libraries(my_program pico_stdlib pico_multicore hardware_i2c)

# Beim Start nie auf ein USB-Terminal warten (ist auch der SDK-Default, hier
# nur explizit festgenagelt)
target_compile_definitions(my_program PRIVATE PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS=0)

# Serial output over USB (no UART)
pico_enable_stdio_usb(my_program 1)
pico_enable_stdio_uart(my_program 0)

# WICHTIG: Erzeugt die .uf2, .hex und .bin Dateien zum Flashen
pico_add_extra_outputs(my_program)

