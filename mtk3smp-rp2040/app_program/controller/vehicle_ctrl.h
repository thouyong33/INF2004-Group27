/**
 * @file vehicle_ctrl.h
 * @brief Vehicle controller: the one task that decides what the car does.
 *
 * The buddy tasks report through ctrl_evt_mbf (common/ctrl_events.h):
 * the obstacle task posts EVT_OBSTACLE when its stop rule is first met,
 * the line task EVT_LINE_END when following ends, and fault_set() posts
 * EVT_FAULT for every newly raised fault bit. The controller also owns
 * Buddy 4 at run time: every CTRL_PERIOD_MS it reads the IMU, runs the
 * terrain detectors and, while the car is on the one-sided ramp, sets the
 * motion task's hump mask and the line task's ramp hold. It is the only
 * caller of those two, and of line_follow_start/stop during a run.
 *
 * A run follows the line at a set speed until the distance limit, an
 * obstacle (stop with the >= 5 cm gap, then a scan to estimate its width:
 * EVT_OBST_WIDTH and robot_status.obstacle_width_mm), a fault, a lost
 * line, or ctrl_stop(). Avoiding the obstacle comes later. The highest
 * ramp of the run is kept in robot_status.max_ramp_mm.
 *
 * Barcodes (CTRL_OPT_BARCODE): the barcode task reads the left IR while
 * following and posts EVT_BARCODE; the controller keeps the command and
 * carries it out at the next junction (EVT_JUNCTION from the line task).
 * User's mapping (24 Oct): "A" = left, "A reversed" = right, "Z" =
 * straight, "Z reversed" = U-turn: a code read forward in the car's
 * direction of travel is the plain letter, one read backwards the
 * "reversed" one (confirmed with A and Z samples, 24 Oct). For a turn the car
 * stops, drives on until the axle is over the junction, turns on the
 * spot on the encoders, then follows the line again for after_mm (0 ends
 * the run there: the test mats have nothing beyond the cross).
 *
 * Impacts (Tribunal 1): a jolt counts only if it comes while the motion
 * task is at a steady speed, not on the ramp, and the wheels then drop
 * below CTRL_JOLT_SLOW_PCT of the run speed within CTRL_JOLT_CONFIRM_MS.
 * Whether an impact stops the car is a run option (CTRL_OPT_IMPACT_STOP)
 * until the IR bracket's ramp strike has been checked against the rule.
 *
 * robot_status is a small snapshot, copied under a lock, that Buddy 1's
 * telemetry reads (every 500 ms per Tribunal 1).
 */

#ifndef CONTROLLER_VEHICLE_CTRL_H
#define CONTROLLER_VEHICLE_CTRL_H

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>

/** Controller states */
#define CTRL_ST_IDLE         0u   /**< no run yet */
#define CTRL_ST_START        1u   /**< measuring the rest attitude */
#define CTRL_ST_FOLLOW       2u   /**< following the line */
#define CTRL_ST_RAMP         3u   /**< on the ramp: driving straight */
#define CTRL_ST_DONE         4u   /**< ended: distance limit or ctrl_stop() */
#define CTRL_ST_OBSTACLE     5u   /**< ended: stopped at an obstacle */
#define CTRL_ST_IMPACT       6u   /**< ended: confirmed impact */
#define CTRL_ST_FAULT        7u   /**< ended: fault or line lost */
#define CTRL_ST_TURN         8u   /**< carrying out a barcode command */

/** Barcode commands (3 bits, in robot_status.flags) */
#define CTRL_CMD_NONE        0u
#define CTRL_CMD_LEFT        1u
#define CTRL_CMD_RIGHT       2u
#define CTRL_CMD_STRAIGHT    3u
#define CTRL_CMD_UTURN       4u

/** robot_status.flags */
#define CTRL_FLAG_ON_RAMP    (1u << 0)
#define CTRL_FLAG_IMU_OK     (1u << 1)
#define CTRL_FLAG_WATCHING   (1u << 2)
#define CTRL_FLAG_CMD_SHIFT  4u          /**< bits 4-6: pending CTRL_CMD_* */

/** Run options */
#define CTRL_OPT_OBSTACLE     (1u << 0)  /**< watch for obstacles, stop at one */
#define CTRL_OPT_IMPACT_STOP  (1u << 1)  /**< stop on a confirmed impact */
#define CTRL_OPT_BARCODE      (1u << 2)  /**< read barcodes, act at the next junction */

/** Snapshot for telemetry and tests, 16 bytes */
typedef struct {
    uint8_t state;           /**< CTRL_ST_* */
    uint8_t faults;          /**< FAULT_* bits */
    uint8_t last_evt;        /**< EVT_* last handled or raised */
    uint8_t flags;           /**< CTRL_FLAG_* */
    int16_t dist_mm;         /**< since the run started */
    int16_t heading_cdeg;    /**< encoders, since the run started, left + */
    int16_t roll_cdeg;       /**< from rest, x0.01 deg */
    int16_t obstacle_mm;     /**< last sonar reading, -1 = none */
    int16_t obstacle_width_mm; /**< width from the scan after a stop, 0 = none */
    uint8_t max_ramp_mm;     /**< highest ramp (hump) this run, mm */
    uint8_t spare;
} robot_status_t;

/** One logged event (tests) */
typedef struct {
    uint16_t t_ms;           /**< since the run started */
    int16_t d_mm;
    uint8_t type;            /**< EVT_* */
    uint8_t arg;
    int16_t val;
} ctrl_log_t;

#define CTRL_LOG_LEN         32u

/**
 * @brief Create the event queue, the controller, line and obstacle tasks,
 * and start the IMU (the run carries on without it if it does not answer).
 */
ER ctrl_task_create(void);

/** @brief Start a run (motion faults and the line-lost fault are cleared). */
ER ctrl_run(int16_t speed_mm_s, int16_t max_mm, uint8_t opts);

/** @brief Distance to follow after a junction turn (0 = end the run there). */
void ctrl_set_after_turn_mm(int16_t after_mm);

/** @brief Ask the controller to end the run (from any task). */
void ctrl_stop(void);

/** @brief True from ctrl_run() until the run has ended. */
bool ctrl_running(void);

/** @brief Copy the status snapshot. */
void ctrl_get_status(robot_status_t *p_st);

/** @brief Copy this run's event log (oldest first); returns the count. */
uint32_t ctrl_log_get(ctrl_log_t *p_out);

#endif /* CONTROLLER_VEHICLE_CTRL_H */
