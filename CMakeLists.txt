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
cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

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

set(PICO_BOARD pico CACHE STRING "Board type")

include(pico_sdk_import.cmake)

project(onee_chan C CXX ASM)
pico_sdk_init()

add_executable(onee_chan
    pca9685.c
    namiko.c
    ov7670.c
    ov7670_pio_dma.c
    frame_find_line.c
    frame_find_line_helpers.c
    labyrinthe.c
    line_following.c
    tof.c
)

# Génère ov7670.pio.h à partir de ov7670.pio
pico_generate_pio_header(onee_chan ${CMAKE_CURRENT_LIST_DIR}/ov7670.pio)

target_link_libraries(onee_chan
    pico_stdlib
    pico_multicore
    hardware_adc
    hardware_i2c
    hardware_pwm
    hardware_pio
    hardware_dma
    hardware_uart
)

if (PICO_CYW43_SUPPORTED)
    target_link_libraries(onee_chan pico_cyw43_arch_none)
endif()

pico_enable_stdio_usb(onee_chan 1)
pico_enable_stdio_uart(onee_chan 0)

target_include_directories(onee_chan PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

pico_add_extra_outputs(onee_chan)
