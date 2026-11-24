/**
 * @file terrain.h
 * @brief Buddy 4: real-time hump and lift detection from the IMU.
 *
 * Feed one attitude sample per control period with the distance travelled
 * (from the wheel encoders) and whether the car is at steady speed. The
 * detector reports a one-sided (side) hump and a full-width hump once the
 * car has come off them, with their height. A separate check
 * (terrain_lift_update) tells when the car has been picked up, which ends
 * the demo (it replaced collision detection, 16 Nov). The course uses the
 * side hump: the car rolls, and
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
#define TERRAIN_EVT_SIDE_HUMP   (1u << 2)

/** What gave the lift away (terrain_lift_update) */
#define TERRAIN_LIFT_TILT       1u   /**< tilted past LIFT_TILT_CDEG */
#define TERRAIN_LIFT_ACCEL      2u   /**< |a| off 1 g by LIFT_ACCEL_MG */
/** all three IRs black for 1 s, no IMU cue needed (val = ms) */
#define TERRAIN_LIFT_NO_FLOOR   3u

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

/** A lift, for the log */
typedef struct {
    uint8_t cue;             /**< TERRAIN_LIFT_* that armed it */
    int16_t cue_val;         /**< tilt, x0.01 deg, or |a| - 1 g, mg */
} lift_event_t;

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
 * @param steady    true while driving at constant speed: speeding up or
 *                  braking reads as pitch, so only pitch humps wait for it
 * @param p_hump    filled when TERRAIN_EVT_HUMP is returned
 * @return TERRAIN_EVT_* flags for events completed by this sample
 *         (for TERRAIN_EVT_SIDE_HUMP read it with terrain_last_side_hump)
 */
uint32_t terrain_update(int32_t dist_mm, const imu_attitude_t *p_att,
                        bool b_steady, hump_event_t *p_hump);

/**
 * @brief Lift check (the end of the demo: picked up, the motors stop).
 * Call every control period during a run, with or without a new IMU
 * sample. All three IRs black (no floor under them) for 1 s is a lift.
 * An IMU cue (tilted past 25 deg from rest for 0.1 s, or the
 * acceleration's size off 1 g by 300 mg: a pull up or a drop) arms a
 * quicker check for 2 s: no floor for 0.5 s. A cross blackens all three
 * for about 18 mm only. On the ramp (and 1 s after it) the acceleration
 * cue is ignored, its bumps being normal there. A black ramp that
 * blackens all three IRs for 1 s would read as a lift (untested).
 * @param t_ms      now, ms
 * @param p_att     attitude, or NULL if no sample this period
 * @param p_cal     calibrated sample from the same read, or NULL
 * @param b_no_floor all three IRs black
 * @param b_on_ramp on the one-sided ramp
 * @param p_lift    filled when it returns true
 * @return true once, when the car has been lifted (until terrain_reset)
 */
bool terrain_lift_update(uint32_t t_ms, const imu_attitude_t *p_att,
                         const imu_raw_t *p_cal, bool b_no_floor,
                         bool b_on_ramp, lift_event_t *p_lift);

/**
 * @brief Normal (true) or long (false) "no floor alone is a lift" time:
 * LIFT_NO_FLOOR_ALONE_MS (1 s) or LIFT_NO_FLOOR_SEQ_MS (3 s); normal after
 * terrain_reset. The controller uses the long one during the obstacle
 * sequence (stops and turns on the spot over a cross arm), only while the
 * IMU works; an IMU cue still makes it 0.5 s.
 */
void terrain_lift_alone(bool b_on);

/** @brief Largest tilt from rest (x0.01 deg) and |a| - 1 g (mg) seen off
 * the ramp since terrain_reset: the margin a normal run keeps below the
 * lift cues (tuning and evidence). */
void terrain_lift_peaks(int16_t *p_tilt_cdeg, int16_t *p_accel_mg);

/** @brief True while inside a hump: nose-down stretch or one-sided roll. */
bool terrain_on_hump(void);

/** @brief The side hump completed by the last TERRAIN_EVT_SIDE_HUMP. */
void terrain_last_side_hump(side_hump_event_t *p_ev);

#endif /* BUDDY4_TERRAIN_H */
