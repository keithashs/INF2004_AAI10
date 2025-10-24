cmake_minimum_required(VERSION 3.12)

# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)
include(pico_extras_import_optional.cmake OPTIONAL)
include(FreeRTOS_Kernel_import.cmake OPTIONAL)

project(INF2004_AAI10 C CXX ASM) 
set(CMAKE_C_STANDARD 11) 
set(CMAKE_CXX_STANDARD 17) 
set(PICO_BOARD pico_w) 

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.3.0")
    message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.3.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

# Initialize the SDK
pico_sdk_init()

add_compile_options(-Wall
        -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
        -Wno-unused-function # we have some for the docs that aren't called
        )
if (CMAKE_C_COMPILER_ID STREQUAL "GNU")
    add_compile_options(-Wno-maybe-uninitialized)
endif()

# Add the directory that contains FreeRTOSConfig.h 
include_directories(${CMAKE_SOURCE_DIR}/freertos_config)

# ---- Wi-Fi & MQTT credentials
if (DEFINED ENV{WIFI_SSID} AND NOT WIFI_SSID)
    set(WIFI_SSID "$ENV{WIFI_SSID}")
endif()
if (DEFINED ENV{WIFI_PASSWORD} AND NOT WIFI_PASSWORD)
    set(WIFI_PASSWORD "$ENV{WIFI_PASSWORD}")
endif()
set(WIFI_SSID "${WIFI_SSID}" CACHE STRING "WiFi SSID")
set(WIFI_PASSWORD "${WIFI_PASSWORD}" CACHE STRING "WiFi Password")

if (NOT PICO_NO_HARDWARE) 
    add_subdirectory(car_demo1)
    add_subdirectory(wifi)
endif ()