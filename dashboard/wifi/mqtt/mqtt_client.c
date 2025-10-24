// dashboard/wifi/mqtt/mqtt_client.c
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/apps/mqtt.h"
#include "lwip/ip4_addr.h"
#include "mqtt_client.h"


static mqtt_client_t* s_client = NULL;
static ip4_addr_t     s_broker_ip;
static uint16_t       s_broker_port = 1883;
static volatile bool  s_wifi_ok = false;
static volatile bool  s_mqtt_ok = false;
static absolute_time_t s_next_reconnect;

static void mqtt_conn_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) {
    (void)arg;
    if (status == MQTT_CONNECT_ACCEPTED) {
        s_mqtt_ok = true;
        printf("[mqtt] connected\n");
    } else {
        s_mqtt_ok = false;
        printf("[mqtt] disconnected status=%d\n", status);
        s_next_reconnect = make_timeout_time_ms(2000);
    }
}

static void wifi_connect_blocking(const char* ssid, const char* pass) {
    if (cyw43_arch_init()) { printf("[wifi] cyw43 init failed\n"); return; }
    cyw43_arch_enable_sta_mode();
    printf("[wifi] Connecting to AP '%s'...\n", ssid);
    int rc = cyw43_arch_wifi_connect_timeout_ms(ssid, pass, CYW43_AUTH_WPA2_AES_PSK, 30000);
    if (rc) { printf("[wifi] Connect failed (%d)\n", rc); return; }
    s_wifi_ok = true;
    printf("[wifi] Connected.\n");
}

static void mqtt_do_connect(void) {
    if (!s_wifi_ok) return;
    if (!s_client) s_client = mqtt_client_new();
    if (!s_client) { printf("[mqtt] no client\n"); return; }

    struct mqtt_connect_client_info_t ci = {
        .client_id   = "pico-car",
        .keep_alive  = 30,
        .client_user = NULL, .client_pass = NULL,
        .will_topic  = NULL, .will_msg    = NULL,
        .will_qos    = 0,    .will_retain = 0
    };

    printf("[mqtt] connecting to %s:%u\n", ip4addr_ntoa(&s_broker_ip), s_broker_port);
    err_t err = mqtt_client_connect(s_client, &s_broker_ip, s_broker_port, mqtt_conn_cb, NULL, &ci);
    if (err != ERR_OK) { printf("[mqtt] connect err=%d\n", err); s_next_reconnect = make_timeout_time_ms(2000); }
}

void mqtt_start(const char* ssid, const char* pass, const char* broker, uint16_t port) {
    if (!ssid) ssid = WIFI_SSID;
    if (!pass) pass = WIFI_PASSWORD;
    if (port)  s_broker_port = port;
    ip4addr_aton(broker, &s_broker_ip);
    wifi_connect_blocking(ssid, pass);
    mqtt_do_connect();
}

bool mqtt_is_ready(void) {
    return s_wifi_ok && s_mqtt_ok;
}

static void mqtt_pub_request_cb(void *arg, err_t result) {
    (void)arg;
    if (result != ERR_OK) printf("[mqtt] publish err=%d\n", result);
}
bool mqtt_publish_text(const char* topic, const char* payload, int qos, bool retain) {
    if (!mqtt_is_ready() || !topic || !payload) return false;
    err_t err = mqtt_publish(s_client, topic, payload, (u16_t)strlen(payload),
                             qos ? 1 : 0, retain ? 1 : 0, mqtt_pub_request_cb, NULL);
    if (err != ERR_OK) { printf("[mqtt] publish err=%d\n", err); return false; }
    return true;
}

bool mqtt_publish_printf(const char* topic, int qos, bool retain, const char* fmt, ...) {
    char buf[384];
    va_list ap; va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n < 0) return false;
    if (n >= (int)sizeof(buf)) buf[sizeof(buf)-1] = '\0'; // truncated
    return mqtt_publish_text(topic, buf, qos, retain);
}

void mqtt_poll(void) {
    if (s_wifi_ok && !s_mqtt_ok && absolute_time_diff_us(get_absolute_time(), s_next_reconnect) <= 0) {
        mqtt_do_connect();
    }
}
