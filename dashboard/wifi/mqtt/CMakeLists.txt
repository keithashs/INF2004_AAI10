cmake_minimum_required(VERSION 3.13)

# Library: wifi_mqtt
add_library(wifi_mqtt
    ${CMAKE_CURRENT_LIST_DIR}/mqtt_client.c
)

target_include_directories(wifi_mqtt PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/..
)

# Let downstream (car_demo1) see these include dirs too
target_include_directories(wifi_mqtt PUBLIC
    ${CMAKE_SOURCE_DIR}/dashboard/wifi
    ${CMAKE_SOURCE_DIR}/dashboard/wifi/mqtt
)

# ---- Compile-time config (propagates to dependents via PUBLIC) ----
# You can override WIFI_SSID / WIFI_PASSWORD at configure time:
#   cmake -S . -B build -DWIFI_SSID="YourSSID" -DWIFI_PASSWORD="YourPass"
target_compile_definitions(wifi_mqtt PUBLIC
  WIFI_SSID="${WIFI_SSID}"
  WIFI_PASSWORD="${WIFI_PASSWORD}"
  MQTT_BROKER_IP="172.20.10.2"
  MQTT_BROKER_PORT=1883
)

# ---- Link deps ----
target_link_libraries(wifi_mqtt
    pico_stdlib
    pico_cyw43_arch_lwip_threadsafe_background
    pico_lwip_mqtt
)
