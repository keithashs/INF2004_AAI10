add_library(ir_lib STATIC
    ir_linefollow.c
)

target_include_directories(ir_lib PUBLIC
    ${CMAKE_CURRENT_LIST_DIR} 
    ${CMAKE_CURRENT_LIST_DIR}/../..
    ${CMAKE_CURRENT_LIST_DIR}/../main
)

target_link_libraries(ir_lib
    pico_stdlib
    hardware_gpio
    hardware_adc
    hardware_timer
)