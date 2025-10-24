// dashboard/wifi/mqtt/mqtt_client.h
#pragma once
#include <stdbool.h>
#include <stdint.h>

// Boot Wi-Fi (STA), connect to AP, and connect to MQTT broker.
// Non-blocking: returns immediately; background will keep trying.
void mqtt_start(const char* ssid,
                const char* pass,
                const char* broker_ip,
                uint16_t    broker_port);

// Returns true when broker connection is up
bool mqtt_is_ready(void);

// Publish raw payload. qos = 0/1 (lwIP MQTT doesn’t really do QoS2)
// Renamed to avoid colliding with the lwIP mqtt_publish() symbol.
bool mqtt_publish_text(const char* topic, const char* payload, int qos, bool retain);

// printf-style publish convenience
bool mqtt_publish_printf(const char* topic, int qos, bool retain, const char* fmt, ...);

// Call periodically if you have a main loop (ok to call often; it’s cheap).
void mqtt_poll(void);
