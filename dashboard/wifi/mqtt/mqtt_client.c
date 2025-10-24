// mqtt_client.c
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/ip_addr.h"
#include "lwip/dns.h"
#include "lwip/apps/mqtt.h"
#include "lwip/altcp_tcp.h"
#include "mqtt_client.h"

static mqtt_client_t *s_client = NULL;
static volatile bool  s_mqtt_connected = false;
static ip_addr_t      s_broker_addr;
static absolute_time_t s_next_retry;
static int            s_broker_port = MQTT_BROKER_PORT;

static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) {
    (void)arg;
    if (status == MQTT_CONNECT_ACCEPTED) {
        s_mqtt_connected = true;
        printf("[MQTT] connected\n");
    } else {
        s_mqtt_connected = false;
        printf("[MQTT] disconnected status=%d\n", status);
        // schedule reconnect after 2s
        s_next_retry = make_timeout_time_ms(2000);
    }
}

static void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len) {
    (void)arg;
    printf("[MQTT] incoming topic=%s len=%" PRIu32 "\n", topic, tot_len);
}

static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags) {
    (void)arg; (void)flags;
    // Drop data (not subscribing right now)
    if (len && data) {
        // printf("[MQTT] data: %.*s\n", len, (const char*)data);
    }
}

static void mqtt_sub_request_cb(void *arg, err_t result) {
    (void)arg;
    printf("[MQTT] sub result=%d\n", result);
}

static void do_mqtt_connect(void) {
    if (!s_client) s_client = mqtt_client_new();
    if (!s_client) {
        printf("[MQTT] client_new failed\n");
        return;
    }
    mqtt_set_inpub_callback(s_client, mqtt_incoming_publish_cb, mqtt_incoming_data_cb, NULL);

    struct mqtt_connect_client_info_t ci = { 0 };
    ci.client_id   = "pico-car";
    ci.keep_alive  = 30;
    ci.will_topic  = NULL;
    ci.will_msg    = NULL;
    ci.will_qos    = 0;
    ci.will_retain = 0;
    ci.client_user = NULL;
    ci.client_pass = NULL;

    printf("[MQTT] connecting to %s:%d ...\n", ipaddr_ntoa(&s_broker_addr), s_broker_port);
    err_t err = mqtt_client_connect(s_client, &s_broker_addr, s_broker_port,
                                    mqtt_connection_cb, NULL, &ci);
    if (err != ERR_OK) {
        printf("[MQTT] connect err=%d\n", err);
        s_next_retry = make_timeout_time_ms(2000);
    }
}

static void ensure_mqtt_connected(void) {
    if (s_mqtt_connected) return;
    if (absolute_time_diff_us(get_absolute_time(), s_next_retry) > 0) return;
    do_mqtt_connect();
}

static bool connect_wifi(const char* ssid, const char* pass) {
    if (cyw43_arch_init()) {
        printf("[WiFi] cyw43 init failed\n");
        return false;
    }
    cyw43_arch_enable_sta_mode();
    printf("[WiFi] connecting to '%s'...\n", ssid);
    int r = cyw43_arch_wifi_connect_timeout_ms(ssid, pass, CYW43_AUTH_WPA2_AES_PSK, 20000);
    if (r) {
        printf("[WiFi] connect failed (%d)\n", r);
        return false;
    }
    printf("[WiFi] connected, got IP via DHCP\n");
    return true;
}

void mqtt_start(const char* ssid, const char* password, const char* broker_ip, int broker_port) {
    const char* use_ssid = (ssid && *ssid) ? ssid : WIFI_SSID;
    const char* use_pwd  = (password && *password) ? password : WIFI_PASSWORD;
    const char* use_bip  = (broker_ip && *broker_ip) ? broker_ip : MQTT_BROKER_IP;
    s_broker_port = broker_port > 0 ? broker_port : MQTT_BROKER_PORT;

    if (!connect_wifi(use_ssid, use_pwd)) {
        printf("[WiFi] start failed\n");
        return;
    }

    // Resolve/parse broker address
    if (!ipaddr_aton(use_bip, &s_broker_addr)) {
        // DNS resolve
        err_t e = dns_gethostbyname(use_bip, &s_broker_addr, NULL, NULL);
        if (e != ERR_OK && e != ERR_INPROGRESS) {
            printf("[MQTT] DNS failed err=%d\n", e);
            return;
        }
    }

    s_mqtt_connected = false;
    s_next_retry = get_absolute_time();
    ensure_mqtt_connected();
}

bool mqtt_is_connected(void) {
    return s_mqtt_connected;
}

bool mqtt_send(const char* topic, const uint8_t* payload, size_t len, int qos, bool retain) {
    if (!s_client || !s_mqtt_connected) return false;
    err_t e = mqtt_publish(s_client,
                           topic,
                           payload,
                           (u16_t)len,
                           qos ? 1 : 0,
                           retain ? 1 : 0,
                           NULL, NULL); // no completion callback
    return e == ERR_OK;
}


int mqtt_publish_printf(const char* topic, const char* fmt, ...) {
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n < 0) return n;
    size_t send_len = (n < (int)sizeof(buf)) ? (size_t)n : sizeof(buf);
    return mqtt_send(topic, (const uint8_t*)buf, send_len, 0, false) ? (int)send_len : -1;
}

void mqtt_pump(void) {
    // With cyw43_arch_lwip_threadsafe_background the stack runs in the SDK’s bg thread,
    // so we only ensure reconnects here.
    ensure_mqtt_connected();
}

// --------- compat with your existing main.c ----------
bool mqtt_is_ready(void) {
    return mqtt_is_connected();
}

void mqtt_poll(void) {
    mqtt_pump();
}
