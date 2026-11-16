/**
 * @file terrain.h
 * @brief Buddy 4: real-time hump and collision detection from the IMU.
 *
 * Feed one attitude sample per control period with the distance travelled
 * (from the wheel encoders) and whether the car is at steady speed. The
 * detector reports a one-sided (side) hump and a full-width hump once the
 * car has come off them, with their height, and a collision on a sharp
 * horizontal jolt. The course uses the side hump: the car rolls, and
 * height = 114 mm x sin(most-tilted 30 mm roll mean) (terrain.c).
 * The pitch (full-width) hump is the earlier method, kept for
 * terrain_on_hump(): height = 80 mm x sin(most nose-down 30 mm mean), on
 * pitch smoothed over 7 samples (2026-10-09: 1.0 / 2.4 cm ramps read
 * 12 / 24 mm).
 */

#ifndef BUDDY4_TERRAIN_H
#define BUDDY4_TERRAIN_H

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>

#include "imu.h"

#define TERRAIN_EVT_HUMP        (1u << 0)
#define TERRAIN_EVT_COLLISION   (1u << 1)
#define TERRAIN_EVT_SIDE_HUMP   (1u << 2)

typedef struct {
    int32_t start_mm;        /**< where the nose-down stretch began */
    int32_t end_mm;          /**< where it ended */
    int32_t plateau_mm;      /**< start of the most nose-down 30 mm */
    int16_t plateau_cdeg;    /**< that window's mean pitch change */
    int16_t height_mm;       /**< estimated hump height */
} hump_event_t;

/** One-sided hump: one wheel went over it and the car rolled. */
typedef struct {
    int32_t start_mm;        /**< where the roll stretch began */
    int32_t end_mm;          /**< where it ended */
    int32_t plateau_mm;      /**< start of the most-tilted 30 mm */
    int16_t plateau_cdeg;    /**< that window's mean roll, right side down + */
    int16_t height_mm;       /**< 114 mm track x sin(|plateau roll|) */
    uint8_t right_side;      /**< 1 = right wheel went over it */
} side_hump_event_t;

typedef struct {
    int32_t dist_mm;         /**< where the jolt happened */
    int16_t jolt_mg;         /**< horizontal accel jump that triggered it */
} collision_event_t;

/**
 * @brief Start detection; call with the car at rest and level.
 * @param rest_pitch_cdeg pitch measured at rest (the zero reference)
 * @param rest_roll_cdeg  roll measured at rest (the zero reference)
 */
void terrain_reset(int16_t rest_pitch_cdeg, int16_t rest_roll_cdeg);

/**
 * @brief Process one sample.
 * @param dist_mm   distance travelled (encoders), increasing forwards
 * @param p_att     attitude from imu_read_attitude()
 * @param p_cal     calibrated sample from the same read (for the jolt)
 * @param steady    true while driving at constant speed: speeding up or
 *                  braking reads as pitch, so only pitch humps wait for it
 * @param p_hump    filled when TERRAIN_EVT_HUMP is returned
 * @param p_coll    filled when TERRAIN_EVT_COLLISION is returned
 * @return TERRAIN_EVT_* flags for events completed by this sample
 *         (for TERRAIN_EVT_SIDE_HUMP read it with terrain_last_side_hump)
 */
uint32_t terrain_update(int32_t dist_mm, const imu_attitude_t *p_att,
                        const imu_raw_t *p_cal, bool b_steady,
                        hump_event_t *p_hump, collision_event_t *p_coll);

/** @brief True while inside a hump: nose-down stretch or one-sided roll. */
bool terrain_on_hump(void);

/** @brief The side hump completed by the last TERRAIN_EVT_SIDE_HUMP. */
void terrain_last_side_hump(side_hump_event_t *p_ev);

/** @brief Largest horizontal jolt seen since terrain_reset (for tuning). */
int16_t terrain_max_jolt_mg(void);

#endif /* BUDDY4_TERRAIN_H */
