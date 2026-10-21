/**
 * @file faults.c
 * @brief Shared fault bitmask with locked read-modify-write.
 */

#include <stdint.h>
#include <tk/tkernel.h>

#include "critical.h"
#include "faults.h"

static volatile uint8_t g_faults = 0u;
static crit_lock_t g_fault_lock;

void
fault_set(uint8_t bits)
{
    UINT sts;

    CRIT_ENTER(&g_fault_lock, sts);
    g_faults = (uint8_t)(g_faults | bits);
    CRIT_EXIT(&g_fault_lock, sts);
}

void
fault_clear(uint8_t bits)
{
    UINT sts;

    CRIT_ENTER(&g_fault_lock, sts);
    g_faults = (uint8_t)(g_faults & (uint8_t)~bits);
    CRIT_EXIT(&g_fault_lock, sts);
}

uint8_t
fault_get(void)
{
    return g_faults;
}
