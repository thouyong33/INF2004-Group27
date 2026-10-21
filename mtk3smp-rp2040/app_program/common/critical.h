/**
 * @file critical.h
 * @brief Short critical section for data shared between tasks (and with a
 *        handler on the same core): interrupts off, plus a spinlock when the
 *        kernel is built for both cores, as the port's ADC driver does.
 *
 * Usage:
 *     UINT sts;
 *     CRIT_ENTER(&lock, sts);
 *     ... a few loads/stores ...
 *     CRIT_EXIT(&lock, sts);
 *
 * Keep the section to a few instructions: it delays every interrupt.
 */

#ifndef COMMON_CRITICAL_H
#define COMMON_CRITICAL_H

#include <tk/tkernel.h>

#if TK_SUPPORT_SMP
#include <tk/smp.h>
typedef T_SPLOCK crit_lock_t;
#define CRIT_ENTER(p_lock, sts)   ((void)ISpinLock((p_lock), &(sts)))
#define CRIT_EXIT(p_lock, sts)    ((void)ISpinUnlock((p_lock), (sts)))
#else
typedef UW crit_lock_t;          /* unused on one core */
#define CRIT_ENTER(p_lock, sts)   do { (void)(p_lock); DI(sts); } while (0)
#define CRIT_EXIT(p_lock, sts)    do { (void)(p_lock); EI(sts); } while (0)
#endif

#endif /* COMMON_CRITICAL_H */
