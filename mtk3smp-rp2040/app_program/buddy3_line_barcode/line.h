/**
 * @file line.h
 * @brief Buddy 3: line following on the centre and right IR modules.
 *
 * The centre (GP4) and right (GP26) spots are 20 mm apart, wider than the
 * 18 mm line (user, 2026-10-23), so the centre rides the line's RIGHT EDGE
 * and the right one only ever sees black on a junction arm:
 *   C R
 *   1 0  centre on the line  -> steer right, toward the edge
 *   0 0  centre on the white -> steer left, toward the edge
 *   0 1  line under the right spot (car far left) -> steer right hard
 *   1 1  a junction arm -> drive straight on for 30 mm
 * The steering grows while it stays on one side (curves) and restarts
 * small whenever the side changes. The car's centre runs ~1-2 cm to the
 * right of the line's centre.
 *
 * Course mats are raised (2026-10-23), so the task never drives to look
 * for the line: a lost line stops the car and searches by turning on the
 * spot only, first toward the side it was last seen, then the other way,
 * within +/-60 deg of the line's direction (the encoder heading averaged
 * over the last ~100 mm followed), so a junction arm is never taken.
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
    int16_t trim;            /**< periods steering to the same side */
    uint8_t bits;            /**< IR_BIT_LEFT/CENTRE/RIGHT at this period */
    uint8_t searching;       /**< LINE_TR_FOLLOW / SEARCH / HOLD */
} line_trace_t;

#define LINE_TR_FOLLOW       0u      /**< following the line */
#define LINE_TR_SEARCH       1u      /**< searching on the spot (every 200 ms) */
#define LINE_TR_HOLD         2u      /**< ramp hold: straight on the encoders */

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

/**
 * @brief Ramp hold (Buddy 4 says the car is on the one-sided ramp). While
 * set, the line task ignores the IR (tilted, it reads false black) and
 * drives straight along the line's direction on the encoders. 50 mm after
 * the hold is released it follows the line again; if neither the centre
 * nor the right IR sees it then, it searches on the spot within +/-45 deg
 * of that direction (FAULT_LINE_LOST if not found).
 * The line need not be followed on the ramp (course rule, user 2026-10-24).
 */
void line_set_hold(bool hold);

/**
 * @brief The line's direction: the encoder heading averaged over the last
 * ~100 mm followed, x0.01 deg, left positive, since following started.
 */
int32_t line_direction_cdeg(void);

/** @brief LINE_END_* (LINE_END_NONE while following). */
uint8_t line_follow_end(void);

/** @brief Number of times the line was lost and found again by searching. */
uint32_t line_recoveries(void);

/** @brief Copy the trace (oldest first); returns the number of entries. */
uint32_t line_trace_get(line_trace_t *p_out);

#endif /* BUDDY3_LINE_H */
