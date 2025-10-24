#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/ip4_addr.h"
#include "lwip/netif.h"
#include "lwip/dhcp.h"
#include "FreeRTOS.h"
#include "task.h"

#ifndef WIFI_SSID
# error "WIFI_SSID not defined (set via target_compile_definitions)"
#endif
#ifndef WIFI_PASSWORD
# error "WIFI_PASSWORD not defined (set via target_compile_definitions)"
#endif

static TaskHandle_t s_reconnect_task = NULL;

bool wifi_start(void) {
    /* Must be called from a task when using _lwip_sys_freertos arch */
    if (cyw43_arch_init()) {
        printf("[WIFI] cyw43_arch_init failed\n");
        return false;
    }
    cyw43_arch_enable_sta_mode();
    printf("[WIFI] STA mode enabled\n");
    return true;
}

bool wifi_connect_blocking(uint32_t timeout_ms) {
    printf("[WIFI] Connecting to SSID: %s ...\n", WIFI_SSID);
    int rc = cyw43_arch_wifi_connect_timeout_ms(
        WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, (int)timeout_ms);
    if (rc) {
        printf("[WIFI] Connect FAILED (rc=%d)\n", rc);
        return false;
    }
    printf("[WIFI] Connected\n");
    return true;
}

bool wifi_is_connected(void) {
    /* Link state check */
    return cyw43_wifi_link_status(&cyw43_state, CYW43_ITF_STA) == CYW43_LINK_UP;
}

bool wifi_get_ip(char *out, int out_sz) {
    if (!out || out_sz < 8) return false;

    struct netif *n = &cyw43_state.netif[CYW43_ITF_STA];   // <-- pointer
    if (!n || !netif_is_up(n)) return false;

    const ip4_addr_t *ip = netif_ip4_addr(n);
    if (!ip || ip4_addr_isany_val(*ip)) return false;

    ip4addr_ntoa_r(ip, out, out_sz);
    return true;
}

/* -------- Optional auto-reconnect task -------- */

static void wifi_reconnect_task(void *arg) {
    (void)arg;
    for (;;) {
        if (!wifi_is_connected()) {
            printf("[WIFI] Disconnected, retrying...\n");
            /* Try to connect for up to 20s; if it fails, wait 3s and try again */
            (void)wifi_connect_blocking(20000);
        }
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

void wifi_start_reconnect_task(UBaseType_t priority, uint16_t stack_words) {
    if (s_reconnect_task) return;
    xTaskCreate(wifi_reconnect_task, "wifi_recon", stack_words, NULL, priority, &s_reconnect_task);
}