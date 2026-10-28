/**
 * @file barcode.h
 * @brief Buddy 3: Code 39 decoding from the left IR's analog profile.
 *
 * The left IR (slit-taped, AO on GP27) is sampled once per encoder step
 * while the car drives past the barcode. The profile is split into bars
 * and spaces with a threshold just above the white level (thin bars only
 * peak a few hundred mV above white, 24 Oct), then a window that reads
 * "*...*" is searched for, forwards and backwards. Bars and spaces are
 * sorted into narrow/wide separately: the spot's blur makes bars read
 * narrower and spaces wider than printed. The data characters must decode
 * exactly; each '*' may have one element wrong (a faint or merged bar at
 * the ends, e.g. next to the tape holding the paper down).
 */

#ifndef BUDDY3_BARCODE_H
#define BUDDY3_BARCODE_H

#include <stdint.h>
#include <stdbool.h>

#ifndef BC_ON_MV
/* 100/50 missed faint thin bars (6/10 driven passes, 24 Oct); replaying
 * the 18 logged passes, 60/30 decodes 17, all correct, none wrong */
#define BC_ON_MV             60u     /**< bar starts this far above white */
#endif
#ifndef BC_OFF_MV
#define BC_OFF_MV            30u     /**< ...and ends below this */
#endif
#ifndef BC_ON_PCT
/* Replays 26 Oct: the 1 mm slit at ~8 mm decodes 3/3 at 15-30 % of the
 * span (narrow spaces stay 200-330 mV); the faint 24 Oct set needs the
 * fixed margin, so barcode_read() tries 20 % first, then the fixed margin */
/** ...or this % of white-to-dark, if higher */
#define BC_ON_PCT            20u
#endif
#ifndef BC_OFF_PCT
#define BC_OFF_PCT           10u
#endif
#ifndef BC_DARK_PCTL
#define BC_DARK_PCTL         90u     /**< "dark" level percentile of a read */
#endif
#define BC_MAX_ELEMS         64u
#define BC_MAX_TEXT          6u      /**< data characters between the stars */

/** One bar or space */
typedef struct {
    uint8_t bar;             /**< 1 = bar (black), 0 = space */
    uint16_t w;              /**< width, samples */
    uint16_t peak;           /**< highest reading in it, mV */
} bc_elem_t;

/** Decode result */
typedef struct {
    char text[BC_MAX_TEXT + 1];  /**< data characters, without the stars */
    uint8_t len;
    uint8_t reversed;        /**< 1 = read backwards in the travel direction */
    uint8_t star_errs;       /**< elements corrected in the two stars */
    uint8_t first;           /**< first element of the code in the list */
} bc_result_t;

/**
 * @brief Split a profile into bars and spaces, without the white lead-in
 * and lead-out. Returns the number of elements (at most max).
 * @param p_white set to the white level used (10th percentile)
 * @param span_thr also use the BC_ON_PCT/BC_OFF_PCT share of the
 *        white-to-dark span when that is above the fixed margin
 */
uint32_t barcode_elements(const uint16_t *p_mv, uint32_t n, bc_elem_t *p_el,
                          uint32_t max, uint16_t *p_white, bool b_span_thr);

/**
 * @brief Split and decode a profile: span threshold first, then the fixed
 * margin. p_el/p_ne hold the elements of the decoded (or last) attempt.
 */
bool barcode_read(const uint16_t *p_mv, uint32_t n, bc_elem_t *p_el,
                  uint32_t max,
                  uint32_t *p_ne, uint16_t *p_white, bc_result_t *p_res);

/** @brief Find and decode "*...*" either way round; true if found. */
bool barcode_decode(const bc_elem_t *p_el, uint32_t n, bc_result_t *p_res);

#endif /* BUDDY3_BARCODE_H */
