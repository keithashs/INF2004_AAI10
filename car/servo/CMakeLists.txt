add_library(servo_lib
  servo.c
)

target_include_directories(servo_lib PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(servo_lib
  pico_stdlib
  hardware_gpio
  hardware_timer
  hardware_pwm
  FreeRTOS-Kernel
  FreeRTOS-Kernel-Heap4
)
