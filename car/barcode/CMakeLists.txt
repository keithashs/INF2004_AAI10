add_library(barcode_lib STATIC
    barcode.c
)

target_include_directories(barcode_lib PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/../..  # car/ for config.h
)

target_link_libraries(barcode_lib
    pico_stdlib
    hardware_gpio
    hardware_adc
    hardware_timer
    FreeRTOS-Kernel
    FreeRTOS-Kernel-Heap4
)