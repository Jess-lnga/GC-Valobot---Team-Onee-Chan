# Use the SDK selected by CMake or the Pico extension, then the environment.
if (NOT PICO_SDK_PATH AND DEFINED ENV{PICO_SDK_PATH})
    set(PICO_SDK_PATH "$ENV{PICO_SDK_PATH}")
endif()
if (NOT EXISTS "${PICO_SDK_PATH}/external/pico_sdk_import.cmake")
    message(FATAL_ERROR "Pico SDK not found. Set PICO_SDK_PATH to your SDK directory.")
endif()
include("${PICO_SDK_PATH}/external/pico_sdk_import.cmake")
