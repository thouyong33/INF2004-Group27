/**
 * @file cpu_meter.c
 * @brief Per-task CPU time (see cpu_meter.h).
 */

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>
#include <sys/sysdef.h>

#include "cpu_meter.h"

/* One slot per measured task: only that task writes its busy time and
 * pass fields; the window start is written only by cpu_meter_reset(), run
 * by the test task between runs. */
typedef struct {
    ID tid;                  /* 0 = not registered */
    uint32_t since_us;       /* start of the current pass */
    bool b_busy;
    uint64_t busy_us;
    uint32_t max_pass_us;
} cpu_slot_t;

static cpu_slot_t g_slot[CPU_SLOTS];
static volatile uint32_t g_t0_us = 0u;

static uint32_t
now_us(void)
{
    return in_w(TIMER_TIMERAWL);
}

static cpu_slot_t *
mine(void)
{
    ID tid = tk_get_tid();
    uint32_t i;

    for (i = 0u; i < CPU_SLOTS; i++)
    {
        if ((0 != g_slot[i].tid) && (tid == g_slot[i].tid))
        {
            return &g_slot[i];
        }
    }
    return NULL;
}

void
cpu_meter_register(uint32_t slot)
{
    if (slot < CPU_SLOTS)
    {
        g_slot[slot].tid = tk_get_tid();
        g_slot[slot].since_us = now_us();
        g_slot[slot].b_busy = true;
    }
}

void
cpu_idle(void)
{
    cpu_slot_t *p_s = mine();
    uint32_t us;

    if ((NULL != p_s) && p_s->b_busy)
    {
        us = now_us() - p_s->since_us;
        p_s->busy_us += us;
        p_s->max_pass_us = (us > p_s->max_pass_us) ? us : p_s->max_pass_us;
        p_s->b_busy = false;
    }
}

void
cpu_busy(void)
{
    cpu_slot_t *p_s = mine();

    if (NULL != p_s)
    {
        p_s->since_us = now_us();
        p_s->b_busy = true;
    }
}

void
cpu_dly(RELTIM ms)
{
    cpu_idle();
    tk_dly_tsk(ms);
    cpu_busy();
}

void
cpu_meter_add(uint32_t slot, uint32_t us)
{
    if (slot < CPU_SLOTS)
    {
        g_slot[slot].busy_us += us;
        if (us > g_slot[slot].max_pass_us)
        {
            g_slot[slot].max_pass_us = us;
        }
    }
}

void
cpu_meter_reset(void)
{
    uint32_t i;

    for (i = 0u; i < CPU_SLOTS; i++)
    {
        g_slot[i].busy_us = 0u;
        g_slot[i].max_pass_us = 0u;
    }
    g_t0_us = now_us();
}

uint32_t
cpu_meter_get(uint32_t slot, uint32_t *p_busy_us, uint32_t *p_max_pass_us)
{
    if (slot < CPU_SLOTS)
    {
        if (NULL != p_busy_us)
        {
            *p_busy_us = (uint32_t)g_slot[slot].busy_us;
        }
        if (NULL != p_max_pass_us)
        {
            *p_max_pass_us = g_slot[slot].max_pass_us;
        }
    }
    return now_us() - g_t0_us;
}
