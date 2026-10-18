/**
 * @file terrain.h
 * @brief Buddy 4: real-time hump and collision detection from the IMU.
 *
 * Feed one attitude sample per control period with the distance travelled
 * (from the wheel encoders) and whether the car is at steady speed. The
 * detector reports a hump once the car has come off it, with its height,
 * and a collision on a sharp horizontal jolt.
 *
 * Hump method (validated 2026-10-09 on 1.0 / 2.4 cm ramps at 200 mm/s,
 * estimates 12 / 24 mm): the castor floats while the drive wheels climb,
 * then lands; while the wheels come down the far side the nose tips down
 * by asin(h / 80 mm). Height = 80 mm x sin(most nose-down mean over 30 mm
 * of travel), on pitch smoothed over 7 samples.
 */

#ifndef BUDDY4_TERRAIN_H
#define BUDDY4_TERRAIN_H

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>

#include "imu.h"

#define TERRAIN_EVT_HUMP        (1u << 0)
#define TERRAIN_EVT_COLLISION   (1u << 1)

typedef struct {
    int32_t start_mm;        /**< where the nose-down stretch began */
    int32_t end_mm;          /**< where it ended */
    int32_t plateau_mm;      /**< start of the most nose-down 30 mm */
    int16_t plateau_cdeg;    /**< that window's mean pitch change */
    int16_t height_mm;       /**< estimated hump height */
} hump_event_t;

typedef struct {
    int32_t dist_mm;         /**< where the jolt happened */
    int16_t jolt_mg;         /**< horizontal accel jump that triggered it */
} collision_event_t;

/**
 * @brief Start detection; call with the car at rest and level.
 * @param rest_pitch_cdeg pitch measured at rest (the zero reference)
 */
void terrain_reset(int16_t rest_pitch_cdeg);

/**
 * @brief Process one sample.
 * @param dist_mm   distance travelled (encoders), increasing forwards
 * @param p_att     attitude from imu_read_attitude()
 * @param p_cal     calibrated sample from the same read (for the jolt)
 * @param steady    true while driving at constant speed: speeding up or
 *                  braking reads as tilt, so humps are only armed then
 * @param p_hump    filled when TERRAIN_EVT_HUMP is returned
 * @param p_coll    filled when TERRAIN_EVT_COLLISION is returned
 * @return TERRAIN_EVT_* flags for events completed by this sample
 */
uint32_t terrain_update(int32_t dist_mm, const imu_attitude_t *p_att,
                        const imu_raw_t *p_cal, bool steady,
                        hump_event_t *p_hump, collision_event_t *p_coll);

/** @brief Largest horizontal jolt seen since terrain_reset (for tuning). */
int16_t terrain_max_jolt_mg(void);

/** @brief Most nose-down filtered pitch change since reset, x0.01 deg. */
int16_t terrain_min_pitch_cdeg(void);

#endif /* BUDDY4_TERRAIN_H */
