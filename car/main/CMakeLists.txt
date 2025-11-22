
# ---- Main application (links all sensor/actuator libraries) ----
add_executable(main
    main.c
)


target_include_directories(main PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}           # car/demos (for lwipopts.h)
    ${CMAKE_CURRENT_LIST_DIR}/..        # car/ (for config.h)
    ${CMAKE_CURRENT_LIST_DIR}/../motor
    ${CMAKE_CURRENT_LIST_DIR}/../encoder
    ${CMAKE_CURRENT_LIST_DIR}/../imu
    ${CMAKE_CURRENT_LIST_DIR}/../pid
    ${CMAKE_CURRENT_LIST_DIR}/../ultrasonic
    ${CMAKE_CURRENT_LIST_DIR}/../servo
    ${CMAKE_SOURCE_DIR}/freertos_config
    ${PICO_SDK_PATH}/src/rp2_common/pico_lwip/include

)

target_link_libraries(main
    pico_stdlib
    pico_cyw43_arch_lwip_sys_freertos
    pico_lwip_mqtt
    FreeRTOS-Kernel
    FreeRTOS-Kernel-Heap4
    encoder_lib
    motor_lib
    barcode_lib
    imu_lib
    ultrasonic_lib
    servo_lib
    hardware_gpio
    hardware_pwm
    hardware_timer
    hardware_i2c
)

pico_add_extra_outputs(main)
pico_enable_stdio_usb(main 1)
pico_enable_stdio_uart(main 0)