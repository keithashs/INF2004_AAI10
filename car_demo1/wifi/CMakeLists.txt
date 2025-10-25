# Cache Wi-Fi credentials; you can also pass via -D flags
set(WIFI_SSID "${WIFI_SSID}" CACHE INTERNAL "WiFi SSID for examples")
set(WIFI_PASSWORD "${WIFI_PASSWORD}" CACHE INTERNAL "WiFi password for examples")


if ("${WIFI_SSID}" STREQUAL "" OR "${WIFI_PASSWORD}" STREQUAL "")
    message(STATUS "Skipping Pico W tcp_server build because WIFI_SSID or WIFI_PASSWORD is empty")
else()
    add_subdirectory(tcp_server)
endif()
