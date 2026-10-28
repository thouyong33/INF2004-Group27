/**
 * @file obstacle.h
 * @brief Buddy 5: obstacle watch task (Tribunal 1 decisions).
 *
 * A task below the motion task's priority pings the HC-SR04 (servo held at
 * centre) every OBS_PING_MS while watching is on, so the busy-wait on the
 * echo can never delay the motor loop. It is driven by an event flag
 * (scan_flg) with one bit per request:
 *   OBS_FLG_WATCH  ping and keep the near/far history (left set)
 *   OBS_FLG_SCAN   run one obstacle scan (scan.c) in this task
 *   OBS_FLG_ABORT  reserved (abort is a byte the scan checks every step)
 *   OBS_FLG_DONE   set when the scan has finished or stopped
 *
 * Scan safety (Tribunal 1, R2): the scan checks an abort byte at every
 * step and returns with the motors stopped as soon as it is set. The
 * owner waits with obstacle_scan_wait(): if the scan overruns the timeout
 * it raises FAULT_SCAN_TIMEOUT, stops the wheels and aborts the scan.
 * motion_stop() still works at any time, whoever owns the motors.
 *
 * Stop rule: an obstacle is "blocked" when at least 2 of the last 3 pings
 * read OBS_STOP_MM or less (raw sensor reading, i.e. from the sensor face;
 * the IR bracket tip is ~60 mm further forward). A missed echo counts as
 * "not near" but no longer clears the history, so one dropped echo from an
 * angled or soft face cannot hide the obstacle. The history is 3 bits.
 *
 * Faults: FAULT_NO_ECHO is raised only after OBS_DEAD_PINGS pings in a row
 * whose echo pulse never started (sensor unplugged or dead); "nothing
 * within range" is normal. It clears on the next good echo.
 *
 * Watching drivers keep to OBS_WATCH_MAX_MM_S (300 mm/s): two near pings
 * (~130 ms) plus braking must fit inside the stop distance.
 */

#ifndef BUDDY5_OBSTACLE_H
#define BUDDY5_OBSTACLE_H

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>

#include "scan.h"

#define OBS_FLG_WATCH        (1u << 0)
#define OBS_FLG_SCAN         (1u << 1)
#define OBS_FLG_ABORT        (1u << 2)
#define OBS_FLG_DONE         (1u << 3)

#define OBS_STOP_MM          300     /**< raw reading at or below: near */
#define OBS_WATCH_MAX_MM_S   300     /**< speed cap while watching */
#define OBS_SCAN_TIMEOUT_MS  30000u  /**< measured full scans take 20-25 s */
#define OBS_ABORT_WAIT_MS    3000u   /**< scan step limit after an abort */

/** @brief Start the ultrasonic, centre the servo, create the task and flag. */
ER obstacle_task_create(void);

/** @brief Turn watching on (clears the history first) or off. */
void obstacle_watch(bool b_on);

/** @brief True when at least 2 of the last 3 pings were near. */
bool obstacle_blocked(void);

/** @brief Last good reading in mm (raw), or -1 if the last ping had no echo. */
int16_t obstacle_last_mm(void);

/** @brief Pings taken since watching was turned on (for tests). */
uint32_t obstacle_ping_count(void);

/** One ping as recorded by the trace (tests only). */
typedef struct {
    int16_t d_mm;            /**< distance travelled (encoders) at the ping */
    /* *< reading; -1 no echo / beyond 1 m, -2 echo never started */
    int16_t mm;
    uint8_t hist;            /**< near history after this ping, bit 0 newest */
} obstacle_trace_t;

#define OBS_TRACE_LEN        64u

/** @brief Start recording every ping (overwrites the oldest past
   OBS_TRACE_LEN). */
void obstacle_trace_start(void);

/**
 * @brief Copy the recorded pings, oldest first.
 * @return number of entries copied (at most OBS_TRACE_LEN)
 */
uint32_t obstacle_trace_get(obstacle_trace_t *p_out);

/**
 * @brief Start one obstacle scan in the obstacle task (watching pauses).
 * The car must be stopped with room to turn on the spot.
 */
ER obstacle_scan_start(int16_t half_deg, int16_t detect_mm);

/**
 * @brief Wait for the scan started by obstacle_scan_start().
 * @return E_OK with *p_prof filled; E_RLWAI if it was aborted;
 *         E_TMOUT if it overran timeout_ms (FAULT_SCAN_TIMEOUT raised,
 *         wheels stopped, scan aborted)
 */
ER obstacle_scan_wait(uint32_t timeout_ms, obstacle_profile_t *p_prof);

/** @brief Ask a running scan to stop at its next step (any task). */
void obstacle_scan_abort(void);

#endif /* BUDDY5_OBSTACLE_H */
