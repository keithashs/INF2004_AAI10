#ifndef WIFI_H
#define WIFI_H

#include <stdbool.h>
#include <stdint.h>
#include "FreeRTOS.h"

/* Initialize CYW43 + enable STA mode.
   Call this once, from a FreeRTOS task (not before scheduler). */
bool wifi_start(void);

/* Connect to an AP using the credentials you passed via CMake:
   WIFI_SSID and WIFI_PASS. Returns true on success. */
bool wifi_connect_blocking(uint32_t timeout_ms);

/* Returns true if STA is up and associated. */
bool wifi_is_connected(void);

/* Fill a buffer with the IPv4 string (e.g. "192.168.1.23"). Returns true if ok. */
bool wifi_get_ip(char *out, int out_sz);

/* (Optional) Launch a low-priority task that keeps Wi-Fi connected.
   It will retry every few seconds if disconnected. */
void wifi_start_reconnect_task(UBaseType_t priority, uint16_t stack_words);

#endif 