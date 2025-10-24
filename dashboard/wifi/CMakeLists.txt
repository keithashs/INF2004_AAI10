# dashboard/wifi/CMakeLists.txt
set(WIFI_SSID "${WIFI_SSID}" CACHE INTERNAL "WiFi SSID")
set(WIFI_PASSWORD "${WIFI_PASSWORD}" CACHE INTERNAL "WiFi password")

if ("${WIFI_SSID}" STREQUAL "")
    message("Skipping WiFi components as WIFI_SSID is not defined")
elseif ("${WIFI_PASSWORD}" STREQUAL "")
    message("Skipping WiFi components as WIFI_PASSWORD is not defined")
else()
    add_subdirectory(mqtt)
endif()
