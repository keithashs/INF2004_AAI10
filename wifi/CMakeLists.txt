add_library(wifi_lib
    wifi.c
)


target_link_libraries(wifi_lib
    pico_stdlib
    FreeRTOS-Kernel-Heap4
    pico_cyw43_arch_lwip_sys_freertos
    
)

target_include_directories(wifi_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}     
)


target_compile_definitions(wifi_lib PRIVATE
  WIFI_SSID="${WIFI_SSID}"
  WIFI_PASSWORD="${WIFI_PASSWORD}"
)