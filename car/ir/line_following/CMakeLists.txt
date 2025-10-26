add_library(ir_lib STATIC
    ir_linefollow.c
)

target_include_directories(ir_lib PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(ir_lib
    pico_stdlib
    hardware_gpio
)