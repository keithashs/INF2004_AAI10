# Cache Wi-Fi credentials; you can also pass via -D flags
set(WIFI_SSID "${WIFI_SSID}" CACHE INTERNAL "WiFi SSID for examples")
set(WIFI_PASSWORD "${WIFI_PASSWORD}" CACHE INTERNAL "WiFi password for examples")


# if ("${WIFI_SSID}" STREQUAL "" OR "${WIFI_PASSWORD}" STREQUAL "")
#     message(STATUS "Skipping Pico W tcp_server build because WIFI_SSID or WIFI_PASSWORD is empty")
# else()
#     add_subdirectory(tcp_server)
# endif()

# only wifi connection library
add_library(wifi_connect_lib
    wifi_connect.c
)
target_include_directories(wifi_connect_lib PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)
target_compile_definitions(wifi_connect_lib PUBLIC
    WIFI_SSID=\"${WIFI_SSID}\"
    WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
)

# Use the threadsafe background arch (recommended for apps with other work)
target_link_libraries(wifi_connect_lib
    pico_cyw43_arch_lwip_threadsafe_background
    pico_stdlib
)