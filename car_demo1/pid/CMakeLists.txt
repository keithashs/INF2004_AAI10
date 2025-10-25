add_library(pid_lib pid.c)

target_include_directories(pid_lib PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}           # pid/
    ${CMAKE_CURRENT_LIST_DIR}/..        # car_demo1/
    ${CMAKE_CURRENT_LIST_DIR}/../main
)

target_link_libraries(pid_lib
    pico_stdlib
)
