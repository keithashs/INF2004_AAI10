// dashboard/wifi/mqtt/lwipopts.h
#ifndef _LWIPOPTS_H
#define _LWIPOPTS_H
#include "lwipopts_examples_common.h"

// lwIP MQTT client requires ALTCP
#ifndef LWIP_ALTCP
#define LWIP_ALTCP 1
#endif
#ifndef LWIP_ALTCP_TLS
#define LWIP_ALTCP_TLS 0
#endif

#endif
