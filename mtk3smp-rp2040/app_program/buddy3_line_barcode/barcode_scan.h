/**
 * @file barcode_scan.h
 * @brief Buddy 3: barcode scanning task (left IR AO while driving).
 *
 * While enabled, a low-priority task polls the left IR's analog output
 * every 2 ms and keeps one reading per encoder step (0.33 mm). It tracks
 * the white level; once a reading rises BC_ON_MV above it, it records
 * (with ~10 mm of white before) until BCS_QUIET_MM of white follow, then
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

#define BCS_QUIET_MM         15      /**< white after the last bar: code ended */

/** @brief Set up the ADC and create the (idle) scanning task. */
ER barcode_scan_create(void);

/** @brief Start or stop scanning (stopping drops a read in progress). */
void barcode_scan_enable(bool on);

/** @brief Reads finished since start-up (decoded or not). */
uint32_t barcode_scan_count(void);

#endif /* BUDDY3_BARCODE_SCAN_H */
