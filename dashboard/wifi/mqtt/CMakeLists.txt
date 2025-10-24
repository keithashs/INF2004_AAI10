# dashboard/wifi/tcp_server/CMakeLists.txt
# Add the TCP server executable for background mode
add_executable(wifi_mqtt
    mqtt_client.c
)
target_compile_definitions(wifi_mqtt PUBLIC
    # WIFI_SSID="Keithiphone"
    # WIFI_PASSWORD="testong1"
    WIFI_SSID=\"${WIFI_SSID}\"
    WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
    MQTT_BROKER_IP="172.20.10.2"
    MQTT_BROKER_PORT=1883
)
target_include_directories(wifi_mqtt PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/..  # for common lwipopts
)

target_link_libraries(wifi_mqtt
    pico_cyw43_arch_lwip_threadsafe_background
    pico_stdlib
    pico_lwip_mqtt
)


# Enable USB output for the background server
pico_enable_stdio_usb(wifi_mqtt 1)
pico_enable_stdio_uart(wifi_mqtt 0)  # Set to 1 if you want UART output instead
