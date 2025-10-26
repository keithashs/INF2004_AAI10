add_library(imu_lib imu.c)

target_include_directories(imu_lib PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}           # imu/
    ${CMAKE_CURRENT_LIST_DIR}/..
    ${CMAKE_CURRENT_LIST_DIR}/../main
)

target_link_libraries(imu_lib
    pico_stdlib
    hardware_i2c
    hardware_adc            # keep if you read any ADC in IMU code; harmless otherwise
    hardware_gpio
)
