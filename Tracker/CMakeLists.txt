cmake_minimum_required(VERSION 3.13...3.27)

# Write build/compile_commands.json for clangd (ESP-IDF does this on its own; the Pico SDK doesn't)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Raspberry Pi Pico (RP2040). Change to pico2 / pico_w / pico2_w for other boards.
set(PICO_BOARD pico CACHE STRING "Board type")

# Where the SDK lives. Override with -DPICO_SDK_PATH=... or the PICO_SDK_PATH env var.
if(NOT DEFINED PICO_SDK_PATH AND NOT DEFINED ENV{PICO_SDK_PATH})
    set(PICO_SDK_PATH "${CMAKE_CURRENT_LIST_DIR}/../../Share/pico-sdk")
endif()

# SDK 2.x builds the .uf2 with picotool, fetching and building it from git on first
# configure. Keep it outside build/ so a clean build doesn't rebuild it every time.
set(PICOTOOL_FETCH_FROM_GIT_PATH "$ENV{HOME}/.picotool" CACHE PATH "Where picotool is fetched/built")

# Pulls in the SDK; must come before project()
include(pico_sdk_import.cmake)

project(Tracker C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(Tracker
    main/main.c
)

target_include_directories(Tracker PRIVATE main)

# Shared drivers. ESP-IDF finds components/ by itself via EXTRA_COMPONENT_DIRS;
# the Pico SDK has no such discovery, so each one is added by hand. The second
# argument is the binary dir, required because the source is outside this tree.
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../components/neo_m8n neo_m8n)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../components/lora lora)

# pico_stdlib pulls in gpio, timers, uart and the runtime
target_link_libraries(Tracker pico_stdlib neo_m8n lora)

# printf over the Pico's own USB port, not the UART pins
pico_enable_stdio_usb(Tracker 1)
pico_enable_stdio_uart(Tracker 0)

# Produce Tracker.uf2 (drag-and-drop flashing) alongside the .elf
pico_add_extra_outputs(Tracker)
