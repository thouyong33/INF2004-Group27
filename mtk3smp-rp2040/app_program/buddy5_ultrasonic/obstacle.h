/**
 * @file obstacle.h
 * @brief Buddy 5: obstacle watch task (Tribunal 1 decisions).
 *
 * A task below the motion task's priority pings the HC-SR04 (servo held at
 * centre) every OBS_PING_MS while watching is on, so the busy-wait on the
 * echo can never delay the motor loop. Watching is an event flag bit
 * (OBS_FLG_WATCH) that stays set while it is on. The obstacle's width is
 * measured by the controller (vehicle_ctrl.c), which steps the car
 * sideways and pings with ultrasonic.c directly.
 *
 * Stop rule: the controller is told (EVT_OBSTACLE) when 2 of the last 3 pings
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

#define OBS_FLG_WATCH        (1u << 0)

#define OBS_STOP_MM          300     /**< raw reading at or below: near */
#define OBS_WATCH_MAX_MM_S   300     /**< speed cap while watching */

/** @brief Start the ultrasonic, centre the servo, create the task and flag. */
ER obstacle_task_create(void);

/** @brief Turn watching on (clears the history first) or off. */
void obstacle_watch(bool b_on);

/** @brief Last good reading in mm (raw), or -1 if the last ping had no echo. */
int16_t obstacle_last_mm(void);

#endif /* BUDDY5_OBSTACLE_H */
