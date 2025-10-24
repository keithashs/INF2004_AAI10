#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "mqtt.h"
#include "wifi.h"  // for wifi_is_connected()
#include "FreeRTOS.h"
#include "task.h"
#include "lwip/apps/mqtt.h"
#include "lwip/dns.h"
#include "lwip/ip4_addr.h"


#ifndef MQTT_HOST
#error "MQTT_HOST not defined (set via target_compile_definitions)"
#endif
#ifndef MQTT_PORT
# define MQTT_PORT 1883
#endif
#ifndef MQTT_TOPIC_PUB
# define MQTT_TOPIC_PUB "car/telemetry"
#endif
#ifndef MQTT_TOPIC_SUB
# define MQTT_TOPIC_SUB "car/cmd"
#endif

/* ---- Internal state ---- */
static mqtt_client_t *s_client = NULL;
static mqtt_msg_cb_t  s_on_msg = NULL;
static ip_addr_t      s_broker_ip;
static bool           s_dns_ok = false;
static bool           s_connected = false;
static TaskHandle_t   s_task = NULL;

/* ---- Helpers / callbacks ---- */
static void on_pub_done(void *arg, err_t err) {
    (void)arg;
    if (err != ERR_OK) printf("[MQTT] publish err=%d\n", err);
}

static void on_incoming_data(void *arg, const u8_t *data, u16_t len, u8_t flags) {
    (void)arg; (void)flags;
    if (s_on_msg) s_on_msg(MQTT_TOPIC_SUB, data, (size_t)len);
}

static void on_incoming_publish(void *arg, const char *topic, u32_t tot_len) {
    (void)arg; (void)tot_len;
    printf("[MQTT] incoming topic: %s\n", topic);
}

static void on_connection(mqtt_client_t *c, void *arg, mqtt_connection_status_t status) {
    (void)arg;
    if (status == MQTT_CONNECT_ACCEPTED) {
        s_connected = true;
        printf("[MQTT] CONNECTED\n");
        err_t e = mqtt_subscribe(c, MQTT_TOPIC_SUB, 0, NULL, NULL);
        if (e == ERR_OK) printf("[MQTT] subscribed: %s\n", MQTT_TOPIC_SUB);
        else             printf("[MQTT] subscribe failed err=%d\n", e);
    } else {
        s_connected = false;
        printf("[MQTT] disconnected (status=%d)\n", status);
    }
}

static void on_dns_found(const char *name, const ip_addr_t *ipaddr, void *arg) {
    (void)name; (void)arg;
    if (ipaddr) {
        s_broker_ip = *ipaddr;
        s_dns_ok = true;
        char buf[16];
        ip4addr_ntoa_r(ip_2_ip4(&s_broker_ip), buf, sizeof buf);
        printf("[MQTT] broker %s\n", buf);
    } else {
        s_dns_ok = false;
        printf("[MQTT] DNS failed\n");
    }
}

static void try_connect(void) {
    if (!s_dns_ok) return;
    if (!s_client) s_client = mqtt_client_new();
    if (!s_client) { printf("[MQTT] mqtt_client_new failed\n"); return; }

    struct mqtt_connect_client_info_t ci = {0};
    ci.client_id  = "pico_w";
    ci.keep_alive = 30;
    /* If you need auth:
       ci.client_user = "user";
       ci.client_pass = "pass";
    */

    mqtt_set_inpub_callback(s_client, on_incoming_publish, on_incoming_data, NULL);
    err_t e = mqtt_client_connect(s_client, &s_broker_ip, MQTT_PORT, on_connection, NULL, &ci);
    if (e == ERR_OK) printf("[MQTT] connecting...\n");
    else             printf("[MQTT] connect start err=%d\n", e);
}

/* ---- Public API ---- */
void mqtt_init(mqtt_msg_cb_t on_msg) {
    s_on_msg = on_msg;

    /* Resolve host → IP (supports literal IP) */
    ip_addr_t ip;
    if (ipaddr_aton(MQTT_HOST, &ip)) {
        s_broker_ip = ip; s_dns_ok = true;
    } else {
        err_t de = dns_gethostbyname(MQTT_HOST, &s_broker_ip, on_dns_found, NULL);
        if (de == ERR_OK) s_dns_ok = true;
        else if (de != ERR_INPROGRESS) {
            printf("[MQTT] dns_gethostbyname err=%d\n", de);
        }
    }
}

bool mqtt_is_connected(void) {
    return s_connected && s_client && (s_client->conn != NULL);
}

bool mqtt_publish_text(const char *topic, const char *text) {
    return mqtt_publish_raw(topic, text, text ? strlen(text) : 0);
}

bool mqtt_publish_raw(const char *topic, const void *buf, size_t len) {
    if (!mqtt_is_connected()) return false;
    err_t e = mqtt_publish(s_client, topic, (const u8_t *)buf, (u16_t)len,
                           0 /*QoS*/, 0 /*retain*/, on_pub_done, NULL);
    return (e == ERR_OK);
}

/* ---- Service task: reconnect loop ---- */
static void mqtt_task(void *arg) {
    (void)arg;
    TickType_t last = xTaskGetTickCount();

    for (;;) {
        if (!wifi_is_connected()) {
            s_connected = false;
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }

        if (!s_dns_ok) {
            /* retry DNS every 3s */
            ip_addr_t ip;
            if (ipaddr_aton(MQTT_HOST, &ip)) {
                s_broker_ip = ip; s_dns_ok = true;
            } else {
                err_t de = dns_gethostbyname(MQTT_HOST, &s_broker_ip, on_dns_found, NULL);
                if (de == ERR_OK) s_dns_ok = true;
            }
        }

        if (!mqtt_is_connected() && s_dns_ok) {
            try_connect();
        }

        vTaskDelayUntil(&last, pdMS_TO_TICKS(500));  // run twice per second
    }
}

void mqtt_start_service_task(UBaseType_t priority, uint16_t stack_words) {
    if (s_task) return;
    xTaskCreate(mqtt_task, "mqtt", stack_words, NULL, priority, &s_task);
}