/**
 * @file tel_source.h
 * @brief What Buddy 1 publishes, packed into fixed little-endian payloads.
 *
 * The kernel-facing half of the telemetry: it reads the controller and
 * packs the bytes, so the lwIP half (mqtt_tel.c) never includes
 * tk/tkernel.h (see the size_t note in cyw43_utk.h). Plain C types only.
 *
 * Status, TEL_STATUS_LEN bytes (robot_status_t, controller/vehicle_ctrl.h):
 *   0 state, 1 faults, 2 last_evt, 3 flags,
 *   4 dist_mm, 6 heading_cdeg, 8 roll_cdeg, 10 obstacle_mm,
 *   12 obstacle_width_mm (int16), 14 max_ramp_mm, 15 spare,
 *   16 total_cm (uint16)
 * Event, TEL_EVENT_LEN bytes (ctrl_log_t):
 *   0 t_ms (uint32), 4 d_mm (int32), 8 type, 9 arg, 10 val (int16),
 *   12 index in the run (uint16)
 */

#ifndef BUDDY1_TEL_SOURCE_H
#define BUDDY1_TEL_SOURCE_H

#include <stdint.h>
#include <stdbool.h>

#define TEL_STATUS_LEN    18u
#define TEL_EVENT_LEN     14u
#define TEL_EVENT_KEPT    64u   /**< = CTRL_LOG_LEN */

/** @brief Pack the controller's status snapshot. */
void tel_status_pack(uint8_t p_out[TEL_STATUS_LEN]);

/** @brief Events logged this run (restarts at 0 with every run). */
uint32_t tel_event_count(void);

/** @brief Pack event idx of this run; false if not available (not logged
 * yet, or overwritten in the rolling log). */
bool tel_event_pack(uint32_t idx, uint8_t p_out[TEL_EVENT_LEN]);

#endif /* BUDDY1_TEL_SOURCE_H */
