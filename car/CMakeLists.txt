<<<<<<< Updated upstream
# This creates the target *car* first, then applies all Pico helpers to it.

add_executable(car
    main.c
    motor.c
    ultrasonic.c
    imu_lsm303.c
    ir.c
)

# Headers live in this folder
target_include_directories(car PRIVATE ${CMAKE_CURRENT_LIST_DIR})

# Link with Pico libs you use
target_link_libraries(car
    pico_stdlib
    hardware_pwm
    hardware_i2c
    hardware_adc
)

# Use USB CDC for printf; disable UART
pico_enable_stdio_usb(car 1)
pico_enable_stdio_uart(car 0)

# Create UF2/ELF/bin/hex
=======
cmake_minimum_required(VERSION 3.13)

# --- Pico SDK import (robust) ---
if (NOT DEFINED PICO_SDK_PATH)
    set(PICO_SDK_PATH $ENV{PICO_SDK_PATH})
endif()
if (NOT PICO_SDK_PATH)
    message(FATAL_ERROR "PICO_SDK_PATH is not set. Set it or run: cmake -DPICO_SDK_PATH=/absolute/path/to/pico-sdk ..")
endif()
include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(car C CXX)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_library(motors motor.c)
target_link_libraries(motors pico_stdlib hardware_pwm hardware_gpio)
target_include_directories(motors PUBLIC ${CMAKE_CURRENT_LIST_DIR})

add_library(ultrasonic ultrasonic.c)
target_link_libraries(ultrasonic pico_stdlib hardware_gpio hardware_timer)
target_include_directories(ultrasonic PUBLIC ${CMAKE_CURRENT_LIST_DIR})

add_executable(car main.c)
target_link_libraries(car
    pico_stdlib
    # hardware_adc          # <- ADC for line sensors
    motors
    ultrasonic
)

pico_enable_stdio_usb(car 1)
pico_enable_stdio_uart(car 0)
>>>>>>> Stashed changes
pico_add_extra_outputs(car)
