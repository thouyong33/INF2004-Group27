/**
 * @file mqtt_tel.h
 * @brief Buddy 1: MQTT telemetry to the laptop broker (WIFI_MQTT=1 builds).
 *
 * Topics (payloads in tel_source.h):
 *   car27/online  "1" on connect, "0" as the last will; retained
 *   car27/status  the status snapshot every MQTT_TEL_STATUS_MS
 *   car27/event   each new controller event of the run, in order
 *
 * The client runs inside the radio task through cyw43_utk_app_poll(), as
 * lwIP has no other context in this port. It never blocks: if the link
 * or the broker is gone it retries every MQTT_TEL_RETRY_MS and the car
 * drives on regardless. Plain C types only (see tel_source.h).
 */

#ifndef BUDDY1_MQTT_TEL_H
#define BUDDY1_MQTT_TEL_H

#include <stdint.h>

/** mqtt_tel_info_t.state */
#define MQTT_TEL_ST_NO_IP       0u   /**< waiting for the hotspot address */
#define MQTT_TEL_ST_CONNECTING  1u
#define MQTT_TEL_ST_UP          2u   /**< connected, publishing */
#define MQTT_TEL_ST_RETRY       3u   /**< failed or lost, retrying */

/** Counters for the console (read from any task) */
typedef struct {
    uint8_t state;           /**< MQTT_TEL_ST_* */
    int32_t last_result;     /**< last connect status or lwIP error */
    uint32_t connects;       /**< accepted connections since boot */
    uint32_t status_sent;
    uint32_t events_sent;
    uint32_t drops;          /**< status publishes refused (buffer full) */
} mqtt_tel_info_t;

/** @brief Copy the counters. */
void mqtt_tel_get_info(mqtt_tel_info_t *p_info);

#endif /* BUDDY1_MQTT_TEL_H */
