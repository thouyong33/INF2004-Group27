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
 * obstacle and roughly where its edges are: the readings within 45 mm of
 * the nearest one (a range gate, so things behind or beside it don't
 * widen it). Fine: around each edge the
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

/** One reading of a scan: car angle (left +) and range (-1 = no echo) */
typedef struct {
    int16_t cdeg;
    int16_t mm;
} scan_pt_t;

#define SCAN_LOG_MAX         128u    /**< coarse readings kept */
#define SCAN_FINE_MAX        32u     /**< fine readings kept per edge */

typedef struct {
    bool    found;           /**< anything within detect_mm */
    int16_t nearest_mm;      /**< closest reading on the obstacle */
    int16_t nearest_cdeg;    /**< car angle at that reading, left + */
    int16_t right_cdeg;      /**< right-most angle it was still seen */
    int16_t left_cdeg;       /**< left-most angle it was still seen */
    int16_t beam_cdeg;       /**< beam width used for the correction */
    int16_t width_mm;        /**< estimated width (0 = below resolution) */
    int16_t width_old_mm;    /**< same, by the old "anything in range" edges */
    int16_t left_side_mm;    /**< obstacle's left side, mm left of centreline */
    int16_t right_side_mm;   /**< obstacle's right side, mm left of centreline
                                  (negative = to the right) */
} obstacle_profile_t;

/**
 * @brief Scan +/-half_deg around the current heading and profile the
 * nearest obstacle closer than detect_mm. Returns facing the start heading,
 * or E_RLWAI as soon as the abort flag (scan_set_abort_flag) is set; the
 * motors are left stopped and the car where it was.
 * @param p_print  true to print the coarse and fine logs ([SCAN] lines)
 */
ER scan_obstacle(int16_t half_deg, int16_t detect_mm, bool p_print,
                 obstacle_profile_t *p_prof);

/** Where the last scan spent its time (diagnostics). Index 0 = right
 *  edge, 1 = left edge. */
typedef struct {
    uint32_t t0_ms;              /**< scan start (kernel ms) */
    uint32_t start_ms;           /**< turn to the sweep start */
    uint32_t coarse_ms;          /**< continuous sweep */
    uint16_t coarse_samples;
    uint32_t fine_ms[2];         /**< fine look around each edge */
    uint16_t fine_steps[2];      /**< stop-and-look steps taken */
    int16_t  fine_span_cdeg[2];  /**< angle covered by the steps */
    uint32_t return_ms;          /**< turn back to the start heading */
} scan_timing_t;

/** @brief Timing of the last scan_obstacle() call. */
void scan_get_timing(scan_timing_t *p_t);

/** @brief Copy the last scan's coarse sweep (right to left); returns the count. */
uint32_t scan_get_coarse(scan_pt_t *p_out, uint32_t max);

/** @brief Copy the last scan's fine readings at one edge (0 = right, 1 = left). */
uint32_t scan_get_fine(uint32_t edge, scan_pt_t *p_out, uint32_t max);

/**
 * @brief Byte the scan checks at every step; non-zero aborts it.
 * Set by the scan's owner (the obstacle task); NULL disables aborting.
 */
void scan_set_abort_flag(const volatile uint8_t *p_flag);

#endif /* BUDDY5_SCAN_H */
