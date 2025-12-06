# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

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
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

# NOM DU PROJET
project(onee_chan C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Exécutable principal "onee_chan" avec tes deux fichiers source
add_executable(onee_chan
    namiko.c     # main()
    ov7670.c     # module caméra
)

# Dépendances communes
target_link_libraries(onee_chan
    pico_stdlib
    hardware_i2c
    hardware_pwm
)

if (PICO_CYW43_SUPPORTED)
    target_link_libraries(onee_chan pico_cyw43_arch_none)
endif()

# STDIO via USB (CDC)
pico_enable_stdio_usb(onee_chan 1)
pico_enable_stdio_uart(onee_chan 0)

# Optionnel : ajouter le répertoire courant comme include dir (pour ov7670.h)
target_include_directories(onee_chan PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

# Fichiers .uf2/.bin/.map etc.
pico_add_extra_outputs(onee_chan)
