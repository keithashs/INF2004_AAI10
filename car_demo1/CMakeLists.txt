cmake_minimum_required(VERSION 3.13)
set(PROJECT_NAME car_demo1)
project(${PROJECT_NAME} C CXX ASM)

include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)
pico_sdk_init()

add_executable(${PROJECT_NAME}
    main/main.c
    motor/motor.c
    motor/motor.h
    imu/imu.c
    pid/pid.c
    pid/pid.h
    config.h
    imu/imu.h
    encoder/encoder.c
    encoder/encoder.h
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}            # car_demo1/ (for config.h)
    ${CMAKE_CURRENT_LIST_DIR}/main
    ${CMAKE_CURRENT_LIST_DIR}/motor
    ${CMAKE_CURRENT_LIST_DIR}/pid
    ${CMAKE_CURRENT_LIST_DIR}/imu
    ${CMAKE_CURRENT_LIST_DIR}/encoder
    ${CMAKE_SOURCE_DIR}/wifi             # if main.c includes wifi headers later
    ${CMAKE_SOURCE_DIR}/freertos_config  # if FreeRTOSConfig.h is here
)

target_link_libraries(${PROJECT_NAME}
    pico_stdlib
    hardware_gpio
    hardware_pwm
    hardware_adc
    hardware_i2c
    hardware_uart
    hardware_timer
    hardware_irq
    hardware_pio
    # freertos_kernel
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
