#ifndef _LWIPOPTS_H
#define _LWIPOPTS_H

#include "lwipopts_examples_common.h"

// ==== Core System Mode ====
#define NO_SYS 0                       // ✅ using FreeRTOS
#define LWIP_TCPIP_CORE_LOCKING_INPUT 1

// ==== MQTT Configuration ====
#define LWIP_MQTT 1                    // Enable MQTT
#define MEMP_NUM_MQTT_CLIENT 1         // Number of MQTT clients
#define MQTT_REQ_MAX_IN_FLIGHT 4       // Max simultaneous requests
#define MQTT_OUTPUT_RINGBUF_SIZE 512   // Output buffer size

// ==== DHCP & IPv4 ====
#define LWIP_IPV4 1                    // enable IPv4 stack
#define LWIP_DHCP 1                    // ✅ get IP from router/hotspot
#define LWIP_NETIF_API 1               // for netif_* calls

// ==== TCP / MQTT ====
#define LWIP_TCP 1
#define LWIP_RAW 1
#define LWIP_SOCKET 0                  // no POSIX sockets needed for lwIP MQTT
#define LWIP_NETIF_HOSTNAME 1

// ==== Thread Settings ====
#define TCPIP_THREAD_STACKSIZE 2048
#define DEFAULT_THREAD_STACKSIZE 1024
#define DEFAULT_RAW_RECVMBOX_SIZE 8
#define TCPIP_MBOX_SIZE 8
#define LWIP_TIMEVAL_PRIVATE 0

#define MEMP_NUM_SYS_TIMEOUT   20
#define MEMP_NUM_NETBUF        4
#define MEMP_NUM_NETCONN       4 

// ==== Optional Debugging ====
#define LWIP_DEBUG 0                   // set to 1 to enable lwIP debug
#define DHCP_DEBUG LWIP_DBG_ON         // optional
#define NETIF_DEBUG LWIP_DBG_ON        // optional

// ==== Optional timeouts ====
#define LWIP_SO_RCVTIMEO 1

#endif /* _LWIPOPTS_H */
