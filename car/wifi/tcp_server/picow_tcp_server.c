#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/pbuf.h"
#include "lwip/tcp.h"
#include "lwip/ip_addr.h"
#include "lwip/netif.h"
// #include "FreeRTOS.h"
// #include "task.h"
// #include "queue.h"
// #include "barcode.h"

#ifndef WIFI_SSID
#define WIFI_SSID "Keithiphone"
#endif
#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD "testong1"
#endif

#ifndef TCP_SERVER_PORT
#define TCP_SERVER_PORT 5000
#endif

// --- Simple TCP server state ---
static struct tcp_pcb *server_pcb = NULL;
static struct tcp_pcb *client_pcb = NULL;

// Forward declarations of callbacks
static err_t on_tcp_accept(void *arg, struct tcp_pcb *newpcb, err_t err);
static err_t on_tcp_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);
static void  on_tcp_err(void *arg, err_t err);
static err_t on_tcp_sent(void *arg, struct tcp_pcb *tpcb, u16_t len);
static void  close_client(struct tcp_pcb *tpcb);

// --- Helpers to send strings safely from this file ---
static void tcp_send_str(struct tcp_pcb *tpcb, const char *s) {
    if (!tpcb || !s) return;
    u16_t len = (u16_t)strlen(s);
    // queue data into lwIP buffers
    if (tcp_write(tpcb, s, len, TCP_WRITE_FLAG_COPY) == ERR_OK) {
        tcp_output(tpcb); // flush if possible
    }
}

// --- TCP callbacks ---

static err_t on_tcp_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
    (void)arg;
    if (err != ERR_OK) {
        return err;
    }
    if (p == NULL) {
        // Remote closed connection
        printf("TCP: client closed\n");
        close_client(tpcb);
        return ERR_OK;
    }

    // Acknowledge that we have received the data
    tcp_recved(tpcb, p->tot_len);

    // Echo payload back and print to USB
    struct pbuf *q = p;
    while (q) {
        // Print to serial
        fwrite(q->payload, 1, q->len, stdout);
        // Echo back exactly what we got
        tcp_write(tpcb, q->payload, q->len, TCP_WRITE_FLAG_COPY);
        q = q->next;
    }
    tcp_output(tpcb);

    // Free the pbuf chain
    pbuf_free(p);
    return ERR_OK;
}

static err_t on_tcp_sent(void *arg, struct tcp_pcb *tpcb, u16_t len) {
    (void)arg; (void)tpcb; (void)len;
    return ERR_OK;
}

static void on_tcp_err(void *arg, err_t err) {
    (void)arg;
    printf("TCP: error cb, err=%d\n", err);
    client_pcb = NULL; // lwIP already freed pcb
}

static err_t on_tcp_accept(void *arg, struct tcp_pcb *newpcb, err_t err) {
    (void)arg;
    if (err != ERR_OK || newpcb == NULL) {
        return ERR_VAL;
    }

    // Only keep one client in this simple demo
    if (client_pcb) {
        printf("TCP: busy, rejecting extra client\n");
        tcp_abort(newpcb);
        return ERR_ABRT;
    }

    client_pcb = newpcb;
    tcp_arg(client_pcb, NULL);
    tcp_recv(client_pcb, on_tcp_recv);
    tcp_sent(client_pcb, on_tcp_sent);
    tcp_err(client_pcb, on_tcp_err);
    tcp_nagle_disable(client_pcb);

    ip_addr_t ip = client_pcb->remote_ip;
    printf("TCP: client connected from %s:%u\n", ipaddr_ntoa(&ip), client_pcb->remote_port);
    tcp_send_str(client_pcb, "Hello from Pico W TCP server!\r\n");
    return ERR_OK;
}

static void close_client(struct tcp_pcb *tpcb) {
    if (!tpcb) return;
    tcp_arg(tpcb, NULL);
    tcp_sent(tpcb, NULL);
    tcp_recv(tpcb, NULL);
    tcp_err(tpcb, NULL);
    tcp_close(tpcb); // best-effort; may return ERR_MEM, but fine for demo
    if (tpcb == client_pcb) client_pcb = NULL;
}

// --- Server bring-up ---

static bool tcp_server_start(void) {
    server_pcb = tcp_new_ip_type(IPADDR_TYPE_ANY);
    if (!server_pcb) {
        printf("TCP: tcp_new failed\n");
        return false;
    }
    err_t err = tcp_bind(server_pcb, IP_ANY_TYPE, TCP_SERVER_PORT);
    if (err != ERR_OK) {
        printf("TCP: bind failed (%d)\n", err);
        tcp_close(server_pcb);
        server_pcb = NULL;
        return false;
    }

    server_pcb = tcp_listen_with_backlog(server_pcb, 1);
    if (!server_pcb) {
        printf("TCP: listen failed\n");
        return false;
    }

    tcp_accept(server_pcb, on_tcp_accept);
    printf("TCP: listening on port %d\n", TCP_SERVER_PORT);
    return true;
}

int main() {
    stdio_init_all();
    sleep_ms(500); // allow USB CDC to enumerate
    printf("\r\n--- Pico W TCP server (no FreeRTOS) ---\r\n");

#if defined(PICO_CYW43_ARCH_THREADSAFE_BACKGROUND)
    printf("Arch: threadsafe_background\r\n");
#elif defined(PICO_CYW43_ARCH_POLL)
    printf("Arch: poll\r\n");
#endif

    if (cyw43_arch_init()) {
        printf("CYW43 init failed\n");
        while (true) sleep_ms(1000);
    }

    cyw43_arch_enable_sta_mode();

    printf("Connecting to WiFi SSID: %s ...\n", WIFI_SSID);
    int res = cyw43_arch_wifi_connect_timeout_ms(
        WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000
    );
    if (res != 0) {
        printf("WiFi connect failed, err=%d\n", res);
        while (true) sleep_ms(2000);
    }

    printf("WiFi connected. Starting TCP server...\n");
    if (!tcp_server_start()) {
        printf("TCP server failed to start\n");
        while (true) sleep_ms(2000);
    }

    // Main idle loop. For threadsafe_background, lwIP runs in its own core/thread.
    while (true) {
#if defined(PICO_CYW43_ARCH_POLL)
        // If using the poll arch, you must regularly poll to drive lwIP
        cyw43_arch_poll();
#endif
        sleep_ms(100); // don’t spin
    }

    // Not reached
    // cyw43_arch_deinit();
    // return 0;
}