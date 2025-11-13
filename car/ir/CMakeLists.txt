add_library(ir_linefollow_lib STATIC
    ir_linefollow.c
)

target_include_directories(ir_linefollow_lib PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
    # ${CMAKE_CURRENT_LIST_DIR}/../main
)

target_link_libraries(ir_linefollow_lib
    pico_stdlib
    hardware_gpio
)