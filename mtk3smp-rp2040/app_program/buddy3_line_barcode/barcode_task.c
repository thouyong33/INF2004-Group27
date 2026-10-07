/*
 * barcode_task.c
 * Test task for the left IR sensor (barcode reader).
 */

#include "barcode_task.h"
#include "common/pin_defs.h"
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>
#include <sys/sysdef.h>

void barcode_test_task(INT stacd, void *exinf)
{
    (void)stacd;
    (void)exinf;

    // Configure GP6 as GPIO input
    // GPIO_CTRL is already defined in sysdef.h
    out_w(GPIO_CTRL(IR_LEFT_DOUT), 5);  // Set to SIO function (5)
    
    // Clear output enable bit (make it input)
    // GPIO_OE is already defined in sysdef.h
    UW oe = in_w(GPIO_OE);
    out_w(GPIO_OE, oe & ~(1u << IR_LEFT_DOUT));

    tm_printf((UB *)"[barcode_test] Started. Monitoring GP%d (Left IR DOUT)\n", 
              IR_LEFT_DOUT);
    tm_printf((UB *)"[barcode_test] Expected: HIGH=1 (no object), LOW=0 (white surface)\n");

    UINT last_state = 0xFFFFFFFF;  // Force first report
    int count = 0;

    while (1) {
        // Read GPIO value using BSP function
        UINT state = gpio_get_val(IR_LEFT_DOUT);

        // Report state changes
        if (state != last_state) {
            count++;
            tm_printf((UB *)"[barcode_test] GP%d=%u  (transition #%d)\n", 
                      IR_LEFT_DOUT, state, count);
            last_state = state;
        }

        // Poll every 10ms
        tk_dly_tsk(10);
    }
}