/*
 * test_line_barcode.c
 * Buddy 3 test modes: IR log and driven barcode reads.
 * Built only when MOTION_TEST_MODE (test_modes.h) selects one of these modes;
 * moved out of app_main.c unchanged on 28 Oct 2026.
 */

#include "test_common.h"

#define IR_LOG_LEN               400   /* sensor changes per pass */
#define IR_SHORT_MM10            10    /* runs under 1 mm count as chatter */
#define SWEEP_UM_PER_EDGE        328  /* keep in step with UM_PER_EDGE */

#if MOTION_TEST_MODE == IR_TEST_LOG
/* Buddy 3 step 1: IR bring-up. Live readout to check polarity, then
   hand-pushed passes (motors off) that record every sensor change with
   the encoder distance, printed afterwards as black/white run widths. */
typedef struct {
    INT mm10;                    /* distance, x0.1 mm */
    UB  bits;                    /* ir_read() after the change */
} ir_evt_t;

LOCAL UW ir_e0;

/* Distance pushed since the pass started, x0.1 mm. hw_edges counts either
   way round, so push in one direction only. */
LOCAL INT ir_pos_mm10(void)
{
    enc_diag_t l, r;

    (void)motion_get_encoder_diag(&l, &r);
    return (INT)(((l.hw_edges + r.hw_edges - ir_e0) * SWEEP_UM_PER_EDGE)
                 / 200u);
}

LOCAL void ir_print_mm10(INT v)
{
    tm_printf((UB *)"%d.%d", v / 10, v % 10);
}

/* One sensor's runs between its first and last change in the pass */
LOCAL void ir_print_runs(const ir_evt_t *ev, INT n, UB bit, const char *name)
{
    INT i, k = 0;
    INT last_mm10 = -1;
    INT short_n = 0;
    BOOL b_was = FALSE;

    tm_printf((UB *)"[IR] %s runs, mm (B black, W white):\n[IR]  ", name);
    for(i = 0; i < n; i++) {
        BOOL b_now = (0u != (ev[i].bits & bit));

        if((i > 0) && (b_now == b_was)) {
            continue;                    /* another sensor changed */
        }
        if(last_mm10 >= 0) {
            tm_printf((UB *)" %s", b_was ? "B" : "W");
            ir_print_mm10(ev[i].mm10 - last_mm10);
            if((ev[i].mm10 - last_mm10) < IR_SHORT_MM10) {
                short_n++;
            }
            if((++k % 10) == 0) {
                tm_printf((UB *)"\n[IR]  ");
            }
        }
        last_mm10 = ev[i].mm10;
        b_was = b_now;
    }
    if(k == 0) {
        tm_printf((UB *)" (no change)");
    }
    tm_printf((UB *)"\n[IR]   %d runs, %d shorter than 1 mm (edge chatter)\n",
              k, short_n);
}

void ir_test_log(void)
{
    static ir_evt_t ev[IR_LOG_LEN];
    enc_diag_t l, r;
    INT pass, n, key;
    UB bits, prev;

    ir_init();
    tm_printf((UB *)"\n[IR] IR bring-up: L=GP%d C=GP%d R=GP%d, 1 = black "
                    "expected.\n",
              IR_LEFT_DOUT, IR_CENTER_DOUT, IR_RIGHT_DOUT);
    tm_printf((UB *)"[IR] Live readout: hold the car with each sensor over "
                    "white, then\n");
    tm_printf((UB *)"[IR] black (the line or tape). Press a key to go on to "
                    "the passes.\n");
    console_flush_input();
    key = -1;
    while(key < 0) {
        bits = (UB)ir_read();
        tm_printf((UB *)"[IR]   L=%d C=%d R=%d\n",
                  (0u != (bits & IR_BIT_LEFT)) ? 1 : 0,
                  (0u != (bits & IR_BIT_CENTRE)) ? 1 : 0,
                  (0u != (bits & IR_BIT_RIGHT)) ? 1 : 0);
        tk_dly_tsk(400);
        key = console_try_getc();
    }

    for(pass = 1; ; pass++) {
        tm_printf((UB *)"\n[IR] Pass %d: put the car ~3 cm before the line or "
                        "barcode,\n", pass);
        tm_printf((UB *)"[IR] press a key, push it slowly and steadily "
                        "forwards only across,\n");
        tm_printf((UB *)"[IR] then press a key to stop recording.\n");
        console_flush_input();
        while(console_try_getc() < 0) {
            tk_dly_tsk(50);
        }
        tm_printf((UB *)"[IR] recording...\n");
        tk_dly_tsk(100);
        console_flush_input();

        (void)motion_get_encoder_diag(&l, &r);
        ir_e0 = l.hw_edges + r.hw_edges;
        n = 0;
        prev = (UB)ir_read();
        ev[n].mm10 = 0;
        ev[n].bits = prev;
        n++;
        /* No printing here: a print would stretch the poll and blur edges */
        while((n < IR_LOG_LEN) && (console_try_getc() < 0)) {
            bits = (UB)ir_read();
            if(bits != prev) {
                ev[n].mm10 = ir_pos_mm10();
                ev[n].bits = bits;
                n++;
                prev = bits;
            }
            tk_dly_tsk(1);
        }

        tm_printf((UB *)"\n[IR] Pass %d: pushed ", pass);
        ir_print_mm10(ir_pos_mm10());
        tm_printf((UB *)" mm, %d changes%s\n", n - 1,
                  (n >= IR_LOG_LEN) ? " (log FULL, stopped early)" : "");
        ir_print_runs(ev, n, IR_BIT_LEFT, "LEFT  (barcode)");
        ir_print_runs(ev, n, IR_BIT_CENTRE, "CENTRE");
        ir_print_runs(ev, n, IR_BIT_RIGHT, "RIGHT ");
    }
}
#endif /* IR_TEST_LOG */

#if MOTION_TEST_MODE == BARCODE_TEST_DRIVE
/* Buddy 3 barcode step 2: driven scan with the slit-taped left IR. The car
   drives itself straight at BCD_MM_S for BCD_DIST_MM with the barcode
   under the left IR, storing the left AO once per encoder step (0.33 mm):
   constant speed and height, unlike the hand-pushed passes. Decoder v2
   (barcode.c, 24 Oct): threshold just above white, bars and spaces split
   into narrow/wide separately, "*...*" searched both ways. Printed after
   the car stops: levels, every bar (width, peak) and space, the decode,
   and the profile every 1 mm. */
#define BCD_MM_S                 50
#define BCD_DIST_MM              250
#define BCD_LEN                  800    /* steps: 262 mm */
/* profile values per line (1 mm each) */
#define BCD_ROW                  20

LOCAL INT bcd_steps(void)
{
    enc_diag_t l, r;

    (void)motion_get_encoder_diag(&l, &r);
    return (INT)((l.hw_count + r.hw_count) / 2);
}

LOCAL void bcd_print_mm(INT steps)
{
    INT mm10 = (steps * SWEEP_UM_PER_EDGE) / 100;

    tm_printf((UB *)"%d.%d", mm10 / 10, mm10 % 10);
}

void barcode_test_drive(void)
{
    static UH prof[BCD_LEN];
    static bc_elem_t el[BC_MAX_ELEMS];
    bc_result_t res;
    INT pass, n, i, k, s, last, ne, ok_n = 0, tries = 0;
    uint32_t une;
    BOOL b_dec_ok;
    UH white;

    ir_init();
    ir_ao_init();
    tm_printf((UB *)"\n[BCD] Driven barcode scan v2: left IR AO, %d mm/s for "
                    "%d mm.\n",
              BCD_MM_S, BCD_DIST_MM);
    tm_printf((UB *)"[BCD] Live readout first: check white, line, wide bar, "
                    "thin bar."
              " Press a key to go on.\n");
    console_flush_input();
    while(console_try_getc() < 0) {
        tm_printf((UB *)"[BCD]   AO=%d mV\n", (INT)ir_ao_left_mv());
        tk_dly_tsk(400);
    }

    for(pass = 1; ; pass++) {
        tm_printf((UB *)"\n[BCD] Pass %d: put the car so the left IR is ~3 cm "
                        "before the"
                  " barcode, square to it, press a key (any key while driving "
                  "stops it).\n",
                  pass);
        console_flush_input();
        while(console_try_getc() < 0) {
            tk_dly_tsk(50);
        }
        tk_dly_tsk(1000);
        console_flush_input();
        fault_clear(FAULT_MOTION_MASK);
        motion_reset_odometry();
        motion_set_velocity(BCD_MM_S, BCD_MM_S);

        /* No printing while driving */
        last = -1;
        while(console_try_getc() < 0) {
            s = bcd_steps();
            if((s >= BCD_LEN)
               || (((s * SWEEP_UM_PER_EDGE) / 1000) >= BCD_DIST_MM)
               || (0u != (fault_get() & FAULT_MOTION_MASK))) {
                break;
            }
            if(s >= 0) {
                UH mv = (UH)ir_ao_left_mv();

                for(k = last + 1; k <= s; k++) {
                    prof[k] = mv;
                }
                if(s > last) {
                    last = s;
                }
            }
            tk_dly_tsk(1);
        }
        motion_stop();
        tk_dly_tsk(500);
        n = last + 1;

        tm_printf((UB *)"\n[BCD] Pass %d: ", pass);
        bcd_print_mm(n);
        tm_printf((UB *)" mm recorded, faults 0x%x\n", (INT)fault_get());
        if(n < 30) {
            continue;
        }

        b_dec_ok = barcode_read(prof, (uint32_t)n, el, BC_MAX_ELEMS, &une,
                                &white,
                              &res);
        ne = (INT)une;
        tm_printf((UB *)"[BCD]   white %d mV (fixed threshold %d mV); %d "
                        "elements"
                  " (B width mm/peak mV, w width mm):\n[BCD]  ",
                  white, white + BC_ON_MV, ne);
        for(i = 0; i < ne; i++) {
            if(0u != el[i].bar) {
                tm_printf((UB *)" B");
                bcd_print_mm(el[i].w);
                tm_printf((UB *)"/%d", el[i].peak);
            } else {
                tm_printf((UB *)" w");
                bcd_print_mm(el[i].w);
            }
            if((i % 10) == 9) {
                tm_printf((UB *)"\n[BCD]  ");
            }
        }
        tm_printf((UB *)"\n");
        tries++;
        if(b_dec_ok) {
            ok_n++;
            tm_printf((UB *)"[BCD]   DECODED \"%s\" %s, from element %d, %d "
                            "star element(s)"
                      " corrected\n", res.text,
                          (0u != res.reversed) ? "REVERSED" : "forward",
                      (INT)res.first, (INT)res.star_errs);
        } else {
            tm_printf((UB *)"[BCD]   no Code 39 match\n");
        }
        tm_printf((UB *)"[BCD]   decoded %d of %d passes so far\n", ok_n,
                  tries);

        tm_printf((UB *)"[BCD]   profile, mV every 1 mm:\n");
        for(i = 0; i < n; i += 3 * BCD_ROW) {
            tm_printf((UB *)"[BCD] %d mm:", (i * SWEEP_UM_PER_EDGE) / 1000);
            for(k = i; (k < (i + (3 * BCD_ROW))) && (k < n); k += 3) {
                tm_printf((UB *)" %d", prof[k]);
            }
            tm_printf((UB *)"\n");
        }
    }
}
#endif /* BARCODE_TEST_DRIVE */
