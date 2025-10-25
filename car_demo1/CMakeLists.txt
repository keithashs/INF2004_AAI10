cmake_minimum_required(VERSION 3.13)
set(PROJECT_NAME car_demo1)
project(${PROJECT_NAME} C CXX ASM)

# Submodules as libraries (RobotCar-style)
add_subdirectory(motor)
add_subdirectory(encoder)
add_subdirectory(imu)
add_subdirectory(pid)
add_subdirectory(wifi)   # builds a separate TCP server example executable

# ---- Main application (links all sensor/actuator libraries) ----
add_executable(${PROJECT_NAME}
    main/main.c
)

# Allow every module to include config.h from main/
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/main         # for config.h (moved here)
    ${CMAKE_CURRENT_LIST_DIR}/motor
    ${CMAKE_CURRENT_LIST_DIR}/encoder
    ${CMAKE_CURRENT_LIST_DIR}/imu
    ${CMAKE_CURRENT_LIST_DIR}/pid
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
)

pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)
pico_add_extra_outputs(${PROJECT_NAME})
