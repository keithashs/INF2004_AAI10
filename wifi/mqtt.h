#ifndef MQTT_H
#define MQTT_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"

/* Callback for incoming subscribed messages (e.g., commands) */
typedef void (*mqtt_msg_cb_t)(const char *topic, const uint8_t *payload, size_t len);

/* Init MQTT client; set message callback (may be NULL). */
void mqtt_init(mqtt_msg_cb_t on_msg);

/* Start a low-priority task that resolves the broker, connects,
   resubscribes on reconnect, and keeps the session alive. */
void mqtt_start_service_task(UBaseType_t priority, uint16_t stack_words);

/* True if we have an active MQTT connection. */
bool mqtt_is_connected(void);

/* QoS 0 publish helpers (return true on success). */
bool mqtt_publish_text(const char *topic, const char *text);
bool mqtt_publish_raw (const char *topic, const void *buf, size_t len);

#endif 