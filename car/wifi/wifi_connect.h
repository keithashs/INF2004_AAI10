#pragma once
#include <stdbool.h>

// Call once near boot. Returns true if Wi-Fi connected, false if not.
// On failure, the function prints the error and leaves Wi-Fi disabled so
// the rest of your modules can continue normally.
bool wifi_try_connect_once(unsigned timeout_ms);

// If you want to query later:
bool wifi_is_connected(void);

// Optional: get a string to the assigned IP (valid after success).
const char* wifi_ip_str(void);
