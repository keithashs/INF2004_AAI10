add_library(encoder_lib encoder.c)

target_include_directories(encoder_lib PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}           # encoder/
    ${CMAKE_CURRENT_LIST_DIR}/..        # car_demo1/
    ${CMAKE_CURRENT_LIST_DIR}/../main
)

target_link_libraries(encoder_lib
    pico_stdlib
    hardware_gpio
    hardware_timer
)
