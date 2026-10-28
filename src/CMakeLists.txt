cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD pico2)

# Pico SDK importieren
include(pico_sdk_import.cmake)

project(pico_blink C CXX ASM)

# SDK initialisieren
pico_sdk_init()

# Dein Programm definieren
add_executable(my_program main.c)

# Standard-Bibliotheken (für GPIO, sleep etc.) dazulinken
target_link_libraries(my_program pico_stdlib pico_multicore hardware_i2c)

# Serial output over USB (no UART)
pico_enable_stdio_usb(my_program 1)
pico_enable_stdio_uart(my_program 0)

# WICHTIG: Erzeugt die .uf2, .hex und .bin Dateien zum Flashen
pico_add_extra_outputs(my_program)

