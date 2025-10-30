cmake_minimum_required(VERSION 3.13)

add_library(imu_lib
    imu.c
)

# Public headers live in this folder
target_include_directories(imu_lib PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)

# Link Pico SDK libs needed by this module
target_link_libraries(imu_lib
    pico_stdlib
    hardware_i2c
)
