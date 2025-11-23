# ═══════════════════════════════════════════════════════════════════════════
# Test DEMO 1 with WIFI : Basic Motion & No IMU Integration
# ═══════════════════════════════════════════════════════════════════════════

add_executable(testDemo1
    testDemo1.c
)

target_include_directories(testDemo1 PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}           # car/demos (for lwipopts.h)
    ${CMAKE_CURRENT_LIST_DIR}/..        # car/ (for config.h)
    ${CMAKE_CURRENT_LIST_DIR}/../motor
    ${CMAKE_CURRENT_LIST_DIR}/../encoder
    ${CMAKE_CURRENT_LIST_DIR}/../imu
    ${CMAKE_CURRENT_LIST_DIR}/../pid
    ${CMAKE_SOURCE_DIR}/freertos_config
    ${PICO_SDK_PATH}/src/rp2_common/pico_lwip/include

)

target_link_libraries(testDemo1
    pico_stdlib
    pico_cyw43_arch_lwip_sys_freertos
    pico_lwip_mqtt
    FreeRTOS-Kernel
    FreeRTOS-Kernel-Heap4
    encoder_lib
    motor_lib
    # barcode_lib
    imu_lib
    hardware_gpio
    hardware_pwm
    hardware_timer
    hardware_i2c
)

add_compile_definitions(LWIP_MQTT=1)

# Add these lines for Pico W Wi-Fi support
    #target_compile_definitions(testDemo1 PRIVATE
        #PICO_CYW43_ARCH_THREADSAFE_BACKGROUND=1
        #CYW43_LWIP=1
    #)

pico_add_extra_outputs(testDemo1)
pico_enable_stdio_usb(testDemo1 1)
pico_enable_stdio_uart(testDemo1 0)

# ═══════════════════════════════════════════════════════════════════════════
# test DEMO 2: Line Following + Barcode Navigation
# ═══════════════════════════════════════════════════════════════════════════

add_executable(testDemo2
    testDemo2.c
)

target_include_directories(testDemo2 PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}           # car/demos (for lwipopts.h)
    ${CMAKE_CURRENT_LIST_DIR}/..        # car/ (for config.h)
    ${CMAKE_CURRENT_LIST_DIR}/../motor
    ${CMAKE_CURRENT_LIST_DIR}/../encoder
    ${CMAKE_CURRENT_LIST_DIR}/../imu
    ${CMAKE_CURRENT_LIST_DIR}/../pid
    ${CMAKE_CURRENT_LIST_DIR}/../ir
    ${CMAKE_CURRENT_LIST_DIR}/../barcode
    ${CMAKE_SOURCE_DIR}/freertos_config
    ${PICO_SDK_PATH}/src/rp2_common/pico_lwip/include
)

add_compile_definitions(LWIP_MQTT=1)

target_link_libraries(testDemo2
    pico_stdlib
    pico_cyw43_arch_lwip_sys_freertos
    pico_lwip_mqtt
    FreeRTOS-Kernel
    FreeRTOS-Kernel-Heap4
    encoder_lib
    motor_lib
    barcode_lib
    ir_linefollow_lib
    imu_lib
    hardware_gpio
    hardware_pwm
    hardware_timer
    hardware_i2c
)

pico_add_extra_outputs(testDemo2)
pico_enable_stdio_usb(testDemo2 1)
pico_enable_stdio_uart(testDemo2 0)


# ═══════════════════════════════════════════════════════════════════════════
# DEMO 3: Ultrasonic Obstacle Avoidance
# ═══════════════════════════════════════════════════════════════════════════

add_executable(testDemo3
    testDemo3.c
)

target_include_directories(testDemo3 PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/..
    ${CMAKE_CURRENT_LIST_DIR}/../main   # car/main (for lwipopts.h)
    ${CMAKE_CURRENT_LIST_DIR}/../motor
    ${CMAKE_CURRENT_LIST_DIR}/../encoder
    ${CMAKE_CURRENT_LIST_DIR}/../imu
    ${CMAKE_CURRENT_LIST_DIR}/../pid
    ${CMAKE_CURRENT_LIST_DIR}/../ultrasonic
    ${CMAKE_CURRENT_LIST_DIR}/../servo
    ${CMAKE_SOURCE_DIR}/freertos_config
    ${PICO_SDK_PATH}/src/rp2_common/pico_lwip/include

)

target_link_libraries(testDemo3
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

pico_add_extra_outputs(testDemo3)
pico_enable_stdio_usb(testDemo3 1)
pico_enable_stdio_uart(testDemo3 0)