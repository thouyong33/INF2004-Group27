#ifndef BUDDY1_HOOKS_H
#define BUDDY1_HOOKS_H

/*
 * Buddy 1: what the MQTT client (lwip_utk_mqtt.c) needs from the car.
 *
 * Implemented by the application (app_program/buddy1_telemetry/
 * tel_source.c), called from the radio task. Plain C types only: the
 * application side includes tk/tkernel.h and the lwIP side must not
 * (see the size_t note in cyw43_utk.h).
 */

#include <stdint.h>
#include <stdbool.h>
#include "telemetry_codec.h"

/* Event payload: t_ms (uint32), d_mm (int32), type, arg, val (int16),
 * index in the run (uint16), little-endian */
#define BUDDY1_EVENT_LEN 14U
/* Timing payload, once a second: motion loops/s (uint16), motion busy
 * us/s (uint32), longest pass us (uint16), controller samples/s
 * (uint16), longest sample gap ms (uint16), unused (uint16) */
#define BUDDY1_SYS_LEN   14U

/* Fill the live fields (all but version and heartbeat). */
void buddy1_fill_telemetry(telemetry_data_t *p_data);

/* The next controller event not yet sent; buddy1_event_done() once it
 * has been handed to MQTT. */
bool buddy1_event_peek(uint8_t p_out[BUDDY1_EVENT_LEN]);
void buddy1_event_done(void);

void buddy1_sys_pack(uint8_t p_out[BUDDY1_SYS_LEN]);

/* A validated command byte from the laptop (see lwip_utk_mqtt.c). */
void buddy1_command(uint8_t command);

/* The broker has been unreachable for a while (true) or is back. */
void buddy1_link_lost(bool b_lost);

#endif
