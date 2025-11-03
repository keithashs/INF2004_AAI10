# ═══════════════════════════════════════════════════════════════════════════
# DEMO 1: Basic Motion & IMU Integration
# ═══════════════════════════════════════════════════════════════════════════

add_executable(demo1
    demo1.c
)

target_include_directories(demo1 PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}           # car/demos (for lwipopts.h)
    ${CMAKE_CURRENT_LIST_DIR}/..        # car/ (for config.h)
    ${CMAKE_CURRENT_LIST_DIR}/../motor
    ${CMAKE_CURRENT_LIST_DIR}/../encoder
    ${CMAKE_CURRENT_LIST_DIR}/../imu
    ${CMAKE_CURRENT_LIST_DIR}/../pid
    ${CMAKE_SOURCE_DIR}/freertos_config
    ${PICO_SDK_PATH}/src/rp2_common/pico_lwip/include
)

target_link_libraries(demo1
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

pico_add_extra_outputs(demo1)
pico_enable_stdio_usb(demo1 1)
pico_enable_stdio_uart(demo1 0)

# ═══════════════════════════════════════════════════════════════════════════
# DEMO 1 with WIFI : Basic Motion & No IMU Integration
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
# DEMO 1 test: Basic Motion & No IMU Integration
# ═══════════════════════════════════════════════════════════════════════════

add_executable(demo1_copy
    demo1_copy.c
)

target_include_directories(demo1_copy PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/..        # car/ (for config.h)
    ${CMAKE_CURRENT_LIST_DIR}/../motor
    ${CMAKE_CURRENT_LIST_DIR}/../encoder
    ${CMAKE_CURRENT_LIST_DIR}/../imu
    ${CMAKE_CURRENT_LIST_DIR}/../pid
    ${CMAKE_SOURCE_DIR}/freertos_config
)

target_link_libraries(demo1_copy
    pico_stdlib
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

pico_add_extra_outputs(demo1_copy)
pico_enable_stdio_usb(demo1_copy 1)
pico_enable_stdio_uart(demo1_copy 0)

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
    ${CMAKE_CURRENT_LIST_DIR}/../ir
    ${CMAKE_CURRENT_LIST_DIR}/../barcode
    ${CMAKE_SOURCE_DIR}/freertos_config
)

target_link_libraries(demo2
    pico_stdlib
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

pico_add_extra_outputs(demo2)
pico_enable_stdio_usb(demo2 1)
pico_enable_stdio_uart(demo2 0)