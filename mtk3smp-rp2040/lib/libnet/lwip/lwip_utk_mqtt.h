
#ifndef LWIP_UTK_MQTT_H
#define LWIP_UTK_MQTT_H

#include <stdint.h>
#include "lwip/netif.h"

typedef enum {
    MQTT_STATE_WAIT_NETWORK = 0,
    MQTT_STATE_RESOLVING,
    MQTT_STATE_BROKER_RESOLVED,
    MQTT_STATE_DNS_FAILED,
    MQTT_STATE_CONNECTING,
    MQTT_STATE_CONNECTED,
    MQTT_STATE_CONNECT_FAILED
} mqtt_state_t;

typedef struct {
    mqtt_state_t state;
    int dns_result;
    uint8_t broker_ip[4];
} mqtt_status_t;

void lwip_utk_mqtt_poll(struct netif *netif);
void lwip_utk_mqtt_get_status(mqtt_status_t *status);

int lwip_utk_mqtt_get_command(char *buffer, uint16_t buffer_size);

#endif
