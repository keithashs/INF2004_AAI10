add_library(motor_lib
  motor.c
)

# Make motor headers visible to dependents (PUBLIC, not PRIVATE)
target_include_directories(motor_lib PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/../encoder
)

# motor_lib depends on encoder_lib and Pico/FreeRTOS bits
target_link_libraries(motor_lib
  pico_stdlib
  hardware_adc
  hardware_pwm
  FreeRTOS-Kernel
  FreeRTOS-Kernel-Heap4
  encoder_lib
)