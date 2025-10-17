# This creates the target *car* first, then applies all Pico helpers to it.

add_executable(car
    main.c
    motor.c
    ultrasonic.c
    imu_lsm303.c
    ir.c
)

# Headers live in this folder
target_include_directories(car PRIVATE ${CMAKE_CURRENT_LIST_DIR})

# Link with Pico libs you use
target_link_libraries(car
    pico_stdlib
    hardware_pwm
    hardware_i2c
    hardware_adc
)

# Use USB CDC for printf; disable UART
pico_enable_stdio_usb(car 1)
pico_enable_stdio_uart(car 0)

# Create UF2/ELF/bin/hex
pico_add_extra_outputs(car)
