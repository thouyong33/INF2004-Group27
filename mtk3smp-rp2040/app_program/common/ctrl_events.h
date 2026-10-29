/**
 * @file ctrl_events.h
 * @brief Event queue to the vehicle controller (ctrl_evt_mbf, Tribunal 1).
 *
 * The buddy tasks post what they see; only the controller decides what
 * the car does about it. Posting never blocks: if the queue is full the
 * event is dropped and counted (the controller also polls the state it
 * depends on, so a dropped event delays a decision by one period at most).
 * Post from tasks only, never from an interrupt handler.
 */

#ifndef COMMON_CTRL_EVENTS_H
#define COMMON_CTRL_EVENTS_H

#include <stdint.h>
#include <tk/tkernel.h>

/** Event types */
#define EVT_NONE             0u
#define EVT_FAULT            1u   /**< arg = newly set FAULT_* bits */
#define EVT_OBSTACLE         2u   /**< val = sensor reading, mm */
#define EVT_LINE_END         3u   /**< arg = LINE_END_* */
#define EVT_RAMP_ON          4u   /**< controller: on the one-sided ramp */
/** controller: val = height, mm (0 if unknown) */
#define EVT_RAMP_OFF         5u
#define EVT_IMPACT           6u   /**< controller: val = jolt, mg */
/** controller: val = jolt, mg (wheels kept speed) */
#define EVT_JOLT_IGNORED     7u
#define EVT_STOP_REQ         8u   /**< someone asked the controller to stop */
/** arg = character (0 = no read), val = 1 if reversed */
#define EVT_BARCODE          9u
/** line task: junction arm under C and R, val = mm */
#define EVT_JUNCTION         10u  /* arg 0 = right IR, 1 = left IR */
/** controller: command done, arg = CTRL_CMD_* */
#define EVT_COMMAND          11u
/** controller: arg = 1 found, 2 found but the face ran past the sweep
 * (val is a lower bound), 0 none; val = width, mm */
#define EVT_OBST_WIDTH       12u
/** controller: crept in before the scan, val = sonar mm (-1 = none) */
#define EVT_OBST_CREEP       13u
/** controller: obstacle edge, val = mm left of the start; arg 0 right,
 * 1 left, 2 / 3 the same but still on the face at the probe's limit */
#define EVT_OBST_EDGE        14u
/** controller: went round the obstacle, val = mm left of the start where
 * it stopped (0 = back on the line's position) */
#define EVT_OBST_AVOID       15u
/** controller: obstacle sequence over (the one-off report), arg 1 = went
 * round and rejoined the line, 0 = failed; val = time taken, x0.1 s */
#define EVT_OBST_DONE        16u

/** One event, 4 bytes */
typedef struct {
    uint8_t type;            /**< EVT_* */
    uint8_t arg;             /**< type-specific small value */
    int16_t val;             /**< type-specific value */
} ctrl_evt_t;

/** @brief Create the queue (safe to call more than once). */
ER ctrl_evt_init(void);

/** @brief Post an event without waiting; dropped (and counted) if full. */
void ctrl_evt_post(uint8_t type, uint8_t arg, int16_t val);

/** @brief Wait up to tmo for an event; E_OK, E_TMOUT or an error. */
ER ctrl_evt_wait(ctrl_evt_t *p_evt, TMO tmo);

/** @brief Events dropped because the queue was full. */
uint32_t ctrl_evt_dropped(void);

#endif /* COMMON_CTRL_EVENTS_H */
