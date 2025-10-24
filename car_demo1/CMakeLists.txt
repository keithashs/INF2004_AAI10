#  car_demo1/CMakeLists.txt
cmake_minimum_required(VERSION 3.13)
set(PROJECT_NAME car_demo1)
project(${PROJECT_NAME} C CXX ASM)

include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)
pico_sdk_init()

# --- app sources ---
add_executable(${PROJECT_NAME}
    main/main.c
    motor/motor.c
    imu/imu.c
    pid/pid.c
    encoder/encoder.c

)

# Include paths for app modules + wifi/
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}            # car_demo1/ (for config.h)
    ${CMAKE_CURRENT_LIST_DIR}/main
    ${CMAKE_CURRENT_LIST_DIR}/motor
    ${CMAKE_CURRENT_LIST_DIR}/pid
    ${CMAKE_CURRENT_LIST_DIR}/imu
    ${CMAKE_CURRENT_LIST_DIR}/encoder
    ${CMAKE_SOURCE_DIR}/dashboard/wifi        # lwipopts_examples_common.h
    ${CMAKE_SOURCE_DIR}/dashboard/wifi/mqtt   # mqtt_client.h
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

    pico_lwip_mqtt
    wifi_mqtt
    pico_cyw43_arch_lwip_threadsafe_background
)

# USB serial on, UART off
pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)


pico_add_extra_outputs(${PROJECT_NAME})
