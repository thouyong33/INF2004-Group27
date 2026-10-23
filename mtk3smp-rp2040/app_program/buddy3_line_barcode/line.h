/**
 * @file line.h
 * @brief Buddy 3: line following on the centre and right IR modules.
 *
 * The centre (GP4) and right (GP26) modules sit side by side on the front
 * bracket, but their spots are too far apart to both lie on the 18 mm line
 * (user, 2026-10-23). So the centre follows the line and the right one
 * guards its right edge:
 *   C R
 *   1 0  on track
 *   1 1  line moving right (car drifting left)  -> steer right
 *   0 1  line well to the right                 -> steer right harder
 *   0 0  line to the left (car drifting right), or lost: steer left for
 *        up to 30 mm of travel, then treat it as lost
 *
 * Steering is a speed difference between the wheels on top of the base
 * speed (Buddy 2 PI): a proportional kick while off to one side, plus a
 * trim that builds up the longer the car stays off to that side and is
 * held while centred, so a curve's steady difference is kept.
 *
 * Course mats are raised (2026-10-23), so the task never drives to look
 * for the line: a lost line stops the car and searches by turning on the
 * spot only, first toward the side it was last seen, then the other way.
 * If that fails it stops and raises FAULT_LINE_LOST. Following also stops
 * after max_mm of travel (test safety on short mats).
 */

#ifndef BUDDY3_LINE_H
#define BUDDY3_LINE_H

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>

/** Why following ended (2 bits) */
#define LINE_END_NONE        0u      /**< still running */
#define LINE_END_DISTANCE    1u      /**< reached max_mm */
#define LINE_END_LOST        2u      /**< line lost, on-spot search failed */
#define LINE_END_STOPPED     3u      /**< line_follow_stop() or a motion fault */

/** One 20 ms control period, recorded for tests */
typedef struct {
    int16_t d_mm;            /**< distance travelled since the start */
    int16_t corr;            /**< wheel speed difference, mm/s, + = left */
    int16_t trim;            /**< held part of it */
    uint8_t bits;            /**< IR_BIT_LEFT/CENTRE/RIGHT at this period */
    uint8_t searching;       /**< 1 while searching on the spot */
} line_trace_t;

#define LINE_TRACE_LEN       400u    /**< 8 s at 20 ms */

/** @brief Set up the IR pins and create the line task. */
ER line_task_create(void);

/**
 * @brief Start following at base_mm_s; stops after max_mm of travel.
 * Clears the trace and starts recording.
 */
ER line_follow_start(int16_t base_mm_s, int16_t max_mm);

/** @brief Stop following now (the wheels are stopped). */
void line_follow_stop(void);

/** @brief LINE_END_* (LINE_END_NONE while following). */
uint8_t line_follow_end(void);

/** @brief Number of times the line was lost and found again by searching. */
uint32_t line_recoveries(void);

/** @brief Copy the trace (oldest first); returns the number of entries. */
uint32_t line_trace_get(line_trace_t *p_out);

#endif /* BUDDY3_LINE_H */
