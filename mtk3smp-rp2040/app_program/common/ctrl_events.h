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
#define EVT_RAMP_OFF         5u   /**< controller: val = height, mm (0 if unknown) */
#define EVT_IMPACT           6u   /**< controller: val = jolt, mg */
#define EVT_JOLT_IGNORED     7u   /**< controller: val = jolt, mg (wheels kept speed) */
#define EVT_STOP_REQ         8u   /**< someone asked the controller to stop */
#define EVT_BARCODE          9u   /**< arg = character (0 = no read), val = 1 if reversed */
#define EVT_JUNCTION         10u  /**< line task: junction arm under C and R, val = mm */
#define EVT_COMMAND          11u  /**< controller: command done, arg = CTRL_CMD_* */
#define EVT_OBST_WIDTH       12u  /**< controller: arg = 1 if found, val = width, mm */

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
