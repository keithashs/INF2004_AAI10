add_library(ultrasonic_lib
  ultrasonic.c
)

# Make headers visible to anything that links this lib
target_include_directories(ultrasonic_lib PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(ultrasonic_lib
  pico_stdlib
  hardware_gpio
  hardware_timer
  hardware_pwm
  FreeRTOS-Kernel
  FreeRTOS-Kernel-Heap4
)
