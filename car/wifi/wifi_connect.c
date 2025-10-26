#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "wifi_connect.h"

// lwIP helpers for IP access + printing
#include "lwip/netif.h"
#include "lwip/ip_addr.h"
#include "lwip/ip4_addr.h"   // ip4addr_ntoa, netif_ip4_addr (when LWIP_IPV4)

#ifndef WIFI_SSID
#define WIFI_SSID "Keithiphone"
#endif
#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD "testong1"
#endif

static bool s_wifi_connected = false;
static char s_ip_buf[32];

bool wifi_try_connect_once(unsigned timeout_ms) {
    // Init CYW43 block
    if (cyw43_arch_init()) {
        printf("[WIFI] CYW43 init failed\n");
        s_wifi_connected = false;
        return false;
    }

    // Station mode only (no AP)
    cyw43_arch_enable_sta_mode();

    printf("[WIFI] Connecting to SSID: %s ...\n", WIFI_SSID);
    int err = cyw43_arch_wifi_connect_timeout_ms(
        WIFI_SSID, WIFI_PASSWORD,
        CYW43_AUTH_WPA2_AES_PSK, (int)timeout_ms
    );

    if (err) {
        // Cleanly deinit so other modules can use clocks/IRQs without Wi-Fi noise.
        printf("[WIFI] Connect FAILED (err=%d). Continuing without Wi-Fi.\n", err);
        cyw43_arch_deinit();
        s_wifi_connected = false;
        s_ip_buf[0] = '\0';
        return false;
    }

    // ---- Read the assigned IP address in a way that works for IPv4-only lwIP ----
    struct netif *nif = &cyw43_state.netif[CYW43_ITF_STA];
    const ip4_addr_t *ip4 = netif_ip4_addr(nif);
    
    if (ip4 && ip4->addr != 0) {
        snprintf(s_ip_buf, sizeof s_ip_buf, "%s", ip4addr_ntoa(ip4));
    } else {
        // fallback (no IPv4 address assigned yet)
        snprintf(s_ip_buf, sizeof s_ip_buf, "0.0.0.0");
    }

    printf("[WIFI] Connected. IP: %s\n", s_ip_buf);
    s_wifi_connected = true;
    return true;
}

bool wifi_is_connected(void) {
    return s_wifi_connected;
}

const char* wifi_ip_str(void) {
    return s_ip_buf;
}
