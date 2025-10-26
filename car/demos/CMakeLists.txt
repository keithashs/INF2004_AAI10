# ═══════════════════════════════════════════════════════════════════════════
# DEMO 1: Basic Motion & IMU Integration
# ═══════════════════════════════════════════════════════════════════════════

add_executable(demo1
    demo1.c
)

target_include_directories(demo1 PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/..        # car/ (for config.h)
    ${CMAKE_CURRENT_LIST_DIR}/../motor
    ${CMAKE_CURRENT_LIST_DIR}/../encoder
    ${CMAKE_CURRENT_LIST_DIR}/../imu
    ${CMAKE_CURRENT_LIST_DIR}/../pid
)

target_link_libraries(demo1
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

pico_add_extra_outputs(demo1)
pico_enable_stdio_usb(demo1 1)
pico_enable_stdio_uart(demo1 0)

# ═══════════════════════════════════════════════════════════════════════════
# DEMO 2: Line Following + Barcode Navigation
# ═══════════════════════════════════════════════════════════════════════════

# add_executable(demo2
#     demo2.c
# )

# target_include_directories(demo2 PRIVATE
#     ${CMAKE_CURRENT_LIST_DIR}/..        # car/ (for config.h)
#     ${CMAKE_CURRENT_LIST_DIR}/../motor
#     ${CMAKE_CURRENT_LIST_DIR}/../encoder
#     ${CMAKE_CURRENT_LIST_DIR}/../imu
#     ${CMAKE_CURRENT_LIST_DIR}/../pid
#     ${CMAKE_CURRENT_LIST_DIR}/../line_following
# )

# target_link_libraries(demo2
#     motor_lib
#     encoder_lib
#     imu_lib
#     pid_lib
#     line_following_lib
#     pico_stdlib
#     hardware_pwm
#     hardware_gpio
#     hardware_i2c
#     hardware_adc
#     hardware_timer
# )

# pico_add_extra_outputs(demo2)
# pico_enable_stdio_usb(demo2 1)
# pico_enable_stdio_uart(demo2 0)