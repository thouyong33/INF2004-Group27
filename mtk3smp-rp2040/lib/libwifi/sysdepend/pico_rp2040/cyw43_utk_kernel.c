/*
 * Copyright (c) 2026 Muhamed Fauzi Bin Abbas
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Kernel-facing half of the CYW43 service.  Kept separate so the pico-sdk's
 * ISO C size_t does not collide with the inherited T-Kernel syslib typedef. */

#include <tk/tkernel.h>

IMPORT void cyw43_utk_task(INT stacd, void *exinf);

/* USB CDC drains at priority 3 on the same physical core.  Keep radio polling
 * below it so firmware download and the 2 ms poll cadence cannot overflow the
 * console ring, while remaining well above the qualification task.
 * Robot car (19 Nov): below the motion task (5) too. At 4 it preempted the
 * 2 ms motion loop, which fell to 469-472 passes/s once MQTT publishes
 * stopped failing (v59); the car's control must not wait on the radio. */
#define CYW43_TASK_PRIORITY  6
#define CYW43_TASK_STACK     (6 * 1024)

EXPORT ER cyw43_utk_start(void)
{
    T_CTSK ctsk = {0};
    ID task;

#if TK_SUPPORT_SMP
    /* Pin to processor 1 (RP2040 core 0): the radio owns PIO, DMA and the
       USBCTRL IRQ on that core, and must not migrate. */
    ctsk.tskatr = TA_HLNG | TA_RNG1 | TA_ASSPRC;
#else
    ctsk.tskatr = TA_HLNG | TA_RNG1;
#endif
	ctsk.task = (FP)cyw43_utk_task;
    ctsk.itskpri = CYW43_TASK_PRIORITY;
    ctsk.stksz = CYW43_TASK_STACK;
#if TK_SUPPORT_SMP
    ctsk.assprc = TP_PRC1;
#endif
#if USE_OBJECT_NAME
    ctsk.dsname = (UB*)"cyw43";
#endif

    task = tk_cre_tsk(&ctsk);
    if(task < E_OK) return task;
    return tk_sta_tsk(task, 0);
}
