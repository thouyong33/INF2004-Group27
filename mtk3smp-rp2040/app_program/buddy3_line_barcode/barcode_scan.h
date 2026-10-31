/**
 * @file barcode_scan.h
 * @brief Buddy 3: barcode scanning task (left IR AO while driving).
 *
 * While enabled, a low-priority task polls the left IR's analog output
 * every 2 ms and keeps one reading per encoder step (0.33 mm). It tracks
 * the white level; once a reading rises BC_ON_MV above it, it records
 * (with ~10 mm of white before) until BCS_QUIET_MM of white follow (more
 * for a larger-scale code: 4 x its narrowest bar, up to 30 mm), then
 * decodes (barcode.c) and posts EVT_BARCODE to the controller:
 *   arg = the character ('A', 'Z', ...) or 0 if nothing decoded,
 *   val = 1 if read backwards in the travel direction (0 forward), or,
 *         when nothing decoded, the number of bars and spaces seen.
 * A junction arm or the line under the left IR posts a failed read
 * (arg 0); the controller just logs it.
 */

#ifndef BUDDY3_BARCODE_SCAN_H
#define BUDDY3_BARCODE_SCAN_H

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>

#include "barcode.h"

/** white after the last bar: code ended. More than the widest space of
 * the largest code expected (A4 scale, 18 mm): the scaled rule below can
 * not be trusted on the first bar alone, which the IR may clip (A4 run
 * read its first narrow bar short and cut the code after it, 29 Oct) */
#define BCS_QUIET_MM         22

/** @brief Set up the ADC and create the (idle) scanning task. */
ER barcode_scan_create(void);

/** @brief Start or stop scanning (stopping drops a read in progress). */
void barcode_scan_enable(bool b_on);

/** @brief Reads finished since start-up (decoded or not). */
uint32_t barcode_scan_count(void);

/** @brief Bars and spaces of the last read with >= 9 of them (tests); returns
   the count. */
uint32_t barcode_scan_last(bc_elem_t *p_out, uint32_t max);

/** failed reads with this many elements or more are kept (tests) */
#define BCS_FAIL_MIN         20u
#define BCS_FAIL_KEEP        4u      /**< ...the last this many */

/** @brief Failed reads kept since start-up (the last BCS_FAIL_KEEP of them
   can be fetched). */
uint32_t barcode_scan_fail_count(void);

/** @brief Bars and spaces of failed read idx (0 = the first since
   start-up) and the distance where it ended; 0 if no longer kept. */
uint32_t barcode_scan_failed(uint32_t idx, bc_elem_t *p_out, uint32_t max,
                             int32_t *p_d_mm);

#endif /* BUDDY3_BARCODE_SCAN_H */
