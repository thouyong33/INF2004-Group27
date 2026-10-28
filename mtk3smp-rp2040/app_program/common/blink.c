/*
 * Copyright (c) 2026 Muhamed Fauzi Bin Abbas
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * blink.c
 * Health LED task. See blink.h.
 *
 * Since USB does not enumerate and therefore cannot report on itself, the
 * LED also reads out how far USB bring-up gets, as a long/short count:
 *   0  (dark)  tusb_init() failed
 *   1          tk_def_int() rejected the USB interrupt
 *   2          NVIC never unmasked USBCTRL_IRQ
 *   3          USB controller not enabled
 *   4          D+ pull-up not asserted, so no host can see the device
 *   5          pull-up on, but not one USB interrupt has ever fired
 *   6          interrupts firing, but no bus reset or connect from the host
 *   7          host talking, enumeration not completing
 *   8          fully enumerated (plain 2 Hz blink)
 * On the UART console build the level is always 8.
 * If any console call blocks, this task stops and the LED freezes entirely.
 */

#include <stdint.h>
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>

#include "usb_console_compat.h"
#include "blink.h"

#define BLINK_TASK_PRI       10u
#define BLINK_STACK_SIZE     4096u   /* as in the port: SMP kernel calls run deep */
#define BLINK_LONG_ON_MS     600u
#define BLINK_LONG_OFF_MS    300u
#define BLINK_SHORT_ON_MS    150u
#define BLINK_SHORT_OFF_MS   250u
#define BLINK_CODE_GAP_MS    2000u
#define BLINK_HALF_MS        250u    /* 2 Hz when fully up */
#define BLINK_LEVEL_UP       8
#define BLINK_LONG_COUNT     5       /* one long flash counts 5 */

static void
blink_code(int32_t n)
{
    int32_t i;

    for (i = 0; i < (n / BLINK_LONG_COUNT); i++)
    {
        gpio_set_val(BOARD_LED_PIN, 1);
        tk_dly_tsk(BLINK_LONG_ON_MS);
        gpio_set_val(BOARD_LED_PIN, 0);
        tk_dly_tsk(BLINK_LONG_OFF_MS);
    }
    for (i = 0; i < (n % BLINK_LONG_COUNT); i++)
    {
        gpio_set_val(BOARD_LED_PIN, 1);
        tk_dly_tsk(BLINK_SHORT_ON_MS);
        gpio_set_val(BOARD_LED_PIN, 0);
        tk_dly_tsk(BLINK_SHORT_OFF_MS);
    }
    gpio_set_val(BOARD_LED_PIN, 0);
    tk_dly_tsk(BLINK_CODE_GAP_MS);
}

static void
blink_task(INT stacd, void *exinf)
{
    int32_t level;

    (void)stacd;
    (void)exinf;

#if BOARD_DIAG_GPIO_SCAN
    knl_diag_pulse(8);      /* CP8: the blink task is being scheduled */
#endif

    for (;;)
    {
        level = (int32_t)tm_usb_diag_level();
        if (level >= BLINK_LEVEL_UP)
        {
            gpio_set_val(BOARD_LED_PIN, 1);
            tk_dly_tsk(BLINK_HALF_MS);
            gpio_set_val(BOARD_LED_PIN, 0);
            tk_dly_tsk(BLINK_HALF_MS);
        }
        else
        {
            blink_code(level);
        }
    }
}

ER
blink_task_create(void)
{
    T_CTSK ctsk = { 0 };
    ID tid;

    ctsk.tskatr = TA_HLNG | TA_RNG3;
    ctsk.task = (FP)blink_task;
    ctsk.itskpri = (PRI)BLINK_TASK_PRI;
    ctsk.stksz = (SZ)BLINK_STACK_SIZE;
    ctsk.exinf = NULL;

    tid = tk_cre_tsk(&ctsk);
    if (tid < E_OK)
    {
        return (ER)tid;
    }
    return tk_sta_tsk(tid, 0);
}
