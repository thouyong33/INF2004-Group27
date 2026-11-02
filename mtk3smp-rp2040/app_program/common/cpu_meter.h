/**
 * @file cpu_meter.h
 * @brief Per-task CPU time, for the test log's TIMING line.
 *
 * Each measured task registers itself once and sleeps through cpu_dly()
 * (and brackets any other blocking wait with cpu_idle() / cpu_busy()), so
 * the time between waking and sleeping again is added to its slot, read
 * from the RP2040's 1 us timer. A function that sleeps on behalf of its
 * caller (a motion command, a probe step) uses cpu_dly() too and is
 * charged to whichever registered task called it; an unregistered caller
 * is not measured.
 *
 * The figure is an upper bound: if a higher-priority task preempts a pass,
 * that time is counted in both. The motion task, the highest priority, is
 * timed by its own loop and added with cpu_meter_add().
 */

#ifndef COMMON_CPU_METER_H
#define COMMON_CPU_METER_H

#include <stdint.h>
#include <tk/tkernel.h>

#define CPU_MOTION           0u
#define CPU_LINE             1u
#define CPU_BARCODE          2u
#define CPU_OBSTACLE         3u
#define CPU_CTRL             4u
#define CPU_SLOTS            5u

/** @brief Register the calling task for slot (call once, from the task). */
void cpu_meter_register(uint32_t slot);

/** @brief The calling task is about to block (ends its current pass). */
void cpu_idle(void);

/** @brief The calling task has woken (starts a pass). */
void cpu_busy(void);

/** @brief tk_dly_tsk() that does not count the sleep as busy time. */
void cpu_dly(RELTIM ms);

/** @brief Add one pass of us to slot (for a task that times itself). */
void cpu_meter_add(uint32_t slot, uint32_t us);

/** @brief Clear every slot and start a new measuring window. */
void cpu_meter_reset(void);

/**
 * @brief A slot's busy time and longest pass since cpu_meter_reset().
 * @return the window's length in us
 */
uint32_t cpu_meter_get(uint32_t slot, uint32_t *p_busy_us,
                       uint32_t *p_max_pass_us);

#endif /* COMMON_CPU_METER_H */
