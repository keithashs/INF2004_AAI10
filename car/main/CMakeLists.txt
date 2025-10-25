add_library(motor_lib motor.c)

target_include_directories(motor_lib PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}           # motor/
    ${CMAKE_CURRENT_LIST_DIR}/..        # car_demo1/
    ${CMAKE_CURRENT_LIST_DIR}/../main   # for #include "config.h"
)

target_link_libraries(motor_lib
    pico_stdlib
    hardware_pwm
    hardware_gpio
)
