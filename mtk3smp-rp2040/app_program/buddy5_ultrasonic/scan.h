/**
 * @file scan.h
 * @brief Buddy 5: obstacle scan by turning the car, coarse then fine.
 *
 * The HC-SR04 beam is wider than the servo can sweep (measured
 * 2026-10-09 by turning the car past a narrow target: 28 deg at 20 cm,
 * 33 deg at 35 cm, 54 deg at 50 cm, centred within 1 deg), so the servo
 * stays at centre and the car turns on the spot instead. Angles come from
 * the wheel encoders (spin test: encoders 920 deg vs compass 922 deg).
 *
 * Coarse: one continuous sweep across +/-half_deg finds the nearest
 * obstacle and roughly where its edges are. Fine: around each edge the
 * car stops every ~2 deg and takes a median of three pings, to place the
 * edge more precisely. The obstacle is seen over its own angular size plus
 * the beam width at its distance, so
 *     width ~= 2 x distance x tan((seen span - beam width) / 2).
 *
 * Blocking; drives the motors. Call from a task that may wait several
 * seconds, with the car stopped and clear space to turn on the spot.
 */

#ifndef BUDDY5_SCAN_H
#define BUDDY5_SCAN_H

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>

typedef struct {
    bool    found;           /**< anything within detect_mm */
    int16_t nearest_mm;      /**< closest reading on the obstacle */
    int16_t nearest_cdeg;    /**< car angle at that reading, left + */
    int16_t right_cdeg;      /**< right-most angle it was still seen */
    int16_t left_cdeg;       /**< left-most angle it was still seen */
    int16_t beam_cdeg;       /**< beam width used for the correction */
    int16_t width_mm;        /**< estimated width (0 = below resolution) */
    int16_t left_side_mm;    /**< obstacle's left side, mm left of centreline */
    int16_t right_side_mm;   /**< obstacle's right side, mm left of centreline
                                  (negative = to the right) */
} obstacle_profile_t;

/**
 * @brief Scan +/-half_deg around the current heading and profile the
 * nearest obstacle closer than detect_mm. Returns facing the start heading.
 * @param p_print  true to print the coarse and fine logs ([SCAN] lines)
 */
ER scan_obstacle(int16_t half_deg, int16_t detect_mm, bool p_print,
                 obstacle_profile_t *p_prof);

#endif /* BUDDY5_SCAN_H */
