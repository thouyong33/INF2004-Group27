/*
 * Copyright (c) 2026 Muhamed Fauzi Bin Abbas
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * app_main.c
 * INF2004 Group 27 robot car on micro T-Kernel 3.0 (RP2040, Robo Pico).
 *
 * usermain() is the kernel's initial task. It starts the health LED (and
 * the Wi-Fi status task on a WIFI=cyw43 build), creates the motion task,
 * then runs the test mode selected in test_modes/test_modes.h. The test
 * mode creates whatever else it needs (the course mode creates the vehicle
 * controller, which creates the line, barcode and obstacle tasks).
 *
 * The port's producer/consumer IPC demo and the retired test modes were
 * removed on 28 Oct 2026; they are in git history (commit 7e16bec).
 */

#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "common/blink.h"
#include "buddy1_telemetry/wifi_status.h"
#include "buddy2_motion/motion_task.h"
#include "test_modes/test_modes.h"

#define MAIN_MOTION_SETTLE_MS    2000u   /* motion task initialises */

EXPORT INT usermain(void)
{
    ER err;

    err = blink_task_create();
    if (E_OK != err)
    {
        tm_printf((UB *)"[MAIN] blink task failed: %d\n", (INT)err);
    }
#if TM_WIFI_CYW43
    err = wifi_status_create();
    if (E_OK != err)
    {
        tm_printf((UB *)"[MAIN] wifi status task failed: %d\n", (INT)err);
    }
#endif

    tm_printf((UB *)"\n=== Starting Motion Subsystem Test ===\n");
    err = motion_task_create();
    if (E_OK != err)
    {
        tm_printf((UB *)"[MAIN] Motion task creation failed: %d\n", (INT)err);
        return 1;
    }
    tk_dly_tsk(MAIN_MOTION_SETTLE_MS);

    test_mode_run();

    /* The initial task has nothing left to do. It must not return: on
       return the kernel shuts the system down. */
    (void)tk_slp_tsk(TMO_FEVR);
    return 0;
}
