cmake_minimum_required(VERSION 3.13)
set(PROJECT_NAME car)
project(${PROJECT_NAME} C CXX ASM)

# ---- Main application (links all sensor/actuator libraries) ----
add_executable(${PROJECT_NAME}
    main.c
)

# Allow every module to include config.h from main/
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/..
    ${CMAKE_CURRENT_LIST_DIR}/../motor
    ${CMAKE_CURRENT_LIST_DIR}/../encoder
    # ${CMAKE_CURRENT_LIST_DIR}/../wifi
    ${CMAKE_CURRENT_LIST_DIR}/../imu
    ${CMAKE_CURRENT_LIST_DIR}/../pid
    # ${CMAKE_CURRENT_LIST_DIR}/../ir/barcode
    ${CMAKE_CURRENT_LIST_DIR}/../ir/line_following
)

# Link all libraries and SDKs used by main
target_link_libraries(${PROJECT_NAME}
    motor_lib
    encoder_lib
    imu_lib
    pid_lib
    pico_stdlib
    hardware_pwm
    hardware_gpio
    hardware_i2c
    hardware_adc
    hardware_timer
    wifi_connect_lib    # only wifi connection library
)

# create map/bin/hex file etc.
pico_add_extra_outputs(${PROJECT_NAME})

# enable usb output, disable uart output
pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)
