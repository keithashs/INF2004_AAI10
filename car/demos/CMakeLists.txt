# ═══════════════════════════════════════════════════════════════════════════
# testing basic PID motor control
# ═══════════════════════════════════════════════════════════════════════════

add_executable(robo_pico_dc_pwm
    robo_pico_dc_pwm.c
)

<<<<<<< Updated upstream
target_include_directories(demo1 PRIVATE
=======
target_include_directories(robo_pico_dc_pwm PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}           # car/demos (for lwipopts.h)
>>>>>>> Stashed changes
    ${CMAKE_CURRENT_LIST_DIR}/..        # car/ (for config.h)
    ${CMAKE_CURRENT_LIST_DIR}/../motor
    ${CMAKE_CURRENT_LIST_DIR}/../encoder
    ${CMAKE_CURRENT_LIST_DIR}/../imu
    ${CMAKE_CURRENT_LIST_DIR}/../pid
)

<<<<<<< Updated upstream
target_link_libraries(demo1
=======
target_link_libraries(robo_pico_dc_pwm
    pico_stdlib
    # pico_cyw43_arch_lwip_sys_freertos
    # pico_lwip_mqtt
    FreeRTOS-Kernel
    FreeRTOS-Kernel-Heap4
    encoder_lib
>>>>>>> Stashed changes
    motor_lib
    encoder_lib
    imu_lib
    pid_lib
    pico_stdlib
    hardware_pwm
    hardware_gpio
    hardware_i2c
    hardware_timer
    wifi_connect_lib    # only wifi connection library
)

<<<<<<< Updated upstream
pico_add_extra_outputs(demo1)
pico_enable_stdio_usb(demo1 1)
pico_enable_stdio_uart(demo1 0)
=======
add_compile_definitions(LWIP_MQTT=1)

pico_add_extra_outputs(robo_pico_dc_pwm)
pico_enable_stdio_usb(robo_pico_dc_pwm 1)
pico_enable_stdio_uart(robo_pico_dc_pwm 0)

# ═══════════════════════════════════════════════════════════════════════════
# DEMO 1: Basic Motion & IMU Integration
# ═══════════════════════════════════════════════════════════════════════════

# add_executable(demo1
#     demo1.c
# )

# target_include_directories(demo1 PRIVATE
#     ${CMAKE_CURRENT_LIST_DIR}           # car/demos (for lwipopts.h)
#     ${CMAKE_CURRENT_LIST_DIR}/..        # car/ (for config.h)
#     ${CMAKE_CURRENT_LIST_DIR}/../motor
#     ${CMAKE_CURRENT_LIST_DIR}/../encoder
#     ${CMAKE_CURRENT_LIST_DIR}/../imu
#     ${CMAKE_CURRENT_LIST_DIR}/../pid
#     ${CMAKE_SOURCE_DIR}/freertos_config
#     ${PICO_SDK_PATH}/src/rp2_common/pico_lwip/include
# )

# target_link_libraries(demo1
#     pico_stdlib
#     pico_cyw43_arch_lwip_sys_freertos
#     pico_lwip_mqtt
#     FreeRTOS-Kernel
#     FreeRTOS-Kernel-Heap4
#     encoder_lib
#     motor_lib
#     # barcode_lib
#     imu_lib
#     hardware_gpio
#     hardware_pwm
#     hardware_timer
#     hardware_i2c
# )

# add_compile_definitions(LWIP_MQTT=1)

# pico_add_extra_outputs(demo1)
# pico_enable_stdio_usb(demo1 1)
# pico_enable_stdio_uart(demo1 0)
>>>>>>> Stashed changes

# ═══════════════════════════════════════════════════════════════════════════
# DEMO 2: Line Following + Barcode Navigation
# ═══════════════════════════════════════════════════════════════════════════

add_executable(demo2
    demo2.c
)

target_include_directories(demo2 PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/..        # car/ (for config.h)
    ${CMAKE_CURRENT_LIST_DIR}/../motor
    ${CMAKE_CURRENT_LIST_DIR}/../encoder
    ${CMAKE_CURRENT_LIST_DIR}/../imu
    ${CMAKE_CURRENT_LIST_DIR}/../pid
    ${CMAKE_CURRENT_LIST_DIR}/../ir/line_following
    ${CMAKE_CURRENT_LIST_DIR}/../ir/barcode
)

target_link_libraries(demo2
    motor_lib
    encoder_lib
    imu_lib
    pid_lib
    ir_lib                    # Line following library
    barcode_lib               # NEW barcode library
    pico_stdlib
    hardware_pwm
    hardware_gpio
    hardware_i2c
    hardware_adc
    hardware_timer
)

pico_add_extra_outputs(demo2)
pico_enable_stdio_usb(demo2 1)
pico_enable_stdio_uart(demo2 0)