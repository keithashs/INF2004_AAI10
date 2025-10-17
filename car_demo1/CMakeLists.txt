cmake_minimum_required(VERSION 3.13)
set(PROJECT_NAME car_demo1)
project(${PROJECT_NAME} C CXX ASM)

include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)
pico_sdk_init()

add_executable(${PROJECT_NAME}
    main.c
    motor.c
    imu.c
    pid.c
    config.h
    motor.h
    imu.h
    pid.h
)

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(${PROJECT_NAME}
    pico_stdlib
    hardware_pwm
    hardware_gpio
    hardware_i2c
    hardware_irq
)

# USB serial on, UART off
pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)

# Enable float printf (handy for telemetry)
target_compile_definitions(${PROJECT_NAME} PRIVATE
    PICO_PRINTF_ALWAYS_INCLUDED=1
    PICO_PRINTF_ALWAYS_INCLUDE_FLOAT=1
)

pico_add_extra_outputs(${PROJECT_NAME})
