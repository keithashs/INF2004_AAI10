// mqtt_client.h
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Boot Wi-Fi + connect to AP, then connect to MQTT broker.
// Uses compile-time defaults if args == NULL/0.
void mqtt_start(const char* ssid,
                const char* password,
                const char* broker_ip,
                int          broker_port);

bool mqtt_is_connected(void);

// Basic publish (non-blocking). qos: 0/1. retain: false/true
bool mqtt_send(const char* topic,
               const uint8_t* payload, size_t len,
               int qos, bool retain);

// Convenience printf-style publish to a topic.
int mqtt_publish_printf(const char* topic, const char* fmt, ...);

// Optional: call periodically if you want to force reconnect checks (not required
// with threadsafe background arch, but harmless).
void mqtt_pump(void);

bool mqtt_is_ready(void);   // compat: maps to mqtt_is_connected()
void mqtt_poll(void);       // compat: maps to mqtt_pump()
