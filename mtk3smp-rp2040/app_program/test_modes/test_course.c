/*
 * test_course.c
 * Vehicle controller course runs (l / o / b keys).
 * Built only when MOTION_TEST_MODE (test_modes.h) selects one of these modes;
 * moved out of app_main.c unchanged on 28 Oct 2026.
 */

#include "test_common.h"

#define SWEEP_UM_PER_EDGE        328  /* keep in step with UM_PER_EDGE */

#if MOTION_TEST_MODE == CTRL_TEST_COURSE
/* Vehicle controller (controller/vehicle_ctrl.c), key-paced course runs:
     l  line following + one-sided ramp (no obstacle watch)
     o  line following + obstacle stop (block on the line, >= 5 cm gap)
     b  line following + barcode on the straight, command at the cross
        (turns stop the run: the test mats have nothing past the cross)
   The controller runs everything; this task only starts runs, samples
   robot_status every 500 ms like Buddy 1's telemetry will (kept, printed
   after the run: no printing while the wheels turn), and prints the
   controller's event log. Impacts are logged, not acted on, until the
   bracket's ramp strike has been checked against the rule. */
#define CTRLT_MM_S               100
#define CTRLT_MAX_MM             450
#define CTRLT_READ_MM            450   /* r: the 2-mat test space */
#define CTRLT_SNAP_MS            500
#define CTRLT_SNAPS              24

/* The line task's 20 ms trace (since following last started, up to 8 s):
   distance, heading, which IRs saw black, the steering decision, the wheel
   speed difference; S = an on-spot search (every 200 ms), H = ramp hold */
LOCAL void ctrlt_print_line_trace(void)
{
    static line_trace_t tr[LINE_TRACE_LEN];
    INT n;
    INT i;

    n = (INT)line_trace_get(tr);
    tm_printf((UB *)"[CTRL]   line trace (%d periods; d_mm:heading x0.01:IRs"
                    " C/R:steer:corr):", n);
    for(i = 0; i < n; i++) {
        if((i % 8) == 0) {
            tm_printf((UB *)"\n[CTRL]    ");
        }
        if(LINE_TR_FOLLOW == tr[i].searching) {
            tm_printf((UB *)" %d:%d:%c%c:%d:%d", tr[i].d_mm,
                      tr[i].heading_cdeg,
                      (0u != (tr[i].bits & IR_BIT_CENTRE)) ? 'C' : '-',
                      (0u != (tr[i].bits & IR_BIT_RIGHT)) ? 'R' : '-',
                      (INT)tr[i].off, tr[i].corr);
        } else {
            tm_printf((UB *)" %d:%d:%c%c:%c", tr[i].d_mm, tr[i].heading_cdeg,
                      (0u != (tr[i].bits & IR_BIT_CENTRE)) ? 'C' : '-',
                      (0u != (tr[i].bits & IR_BIT_RIGHT)) ? 'R' : '-',
                      (LINE_TR_HOLD == tr[i].searching) ? 'H' : 'S');
        }
    }
    tm_printf((UB *)"\n");
}

/* The obstacle probe's side-steps (sonar position, median reading) */
LOCAL void ctrlt_print_probe(void)
{
    static ctrl_probe_pt_t pts[CTRL_PROBE_MAX];
    INT n;
    INT i;

    n = (INT)ctrl_probe_get(pts);
    tm_printf((UB *)"[CTRL]   probe (%d steps, mm left of the start,"
                    " reading mm):", n);
    for(i = 0; i < n; i++) {
        if((i % 10) == 0) {
            tm_printf((UB *)"\n[CTRL]    ");
        }
        tm_printf((UB *)" %d,%d", pts[i].y_mm, pts[i].mm);
    }
    tm_printf((UB *)"\n");
}

LOCAL const char *ctrlt_state(UB s)
{
    static const char *const txt[9] = {
        "idle", "starting", "following", "on ramp", "DONE (distance or stop)",
        "OBSTACLE: stopped", "IMPACT: stopped", "FAULT / line lost", "turning"
    };
    return (s < 9u) ? txt[s] : "?";
}

LOCAL const char *ctrlt_evt(UB e)
{
    static const char *const txt[17] = {
        "-", "FAULT", "OBSTACLE", "LINE_END", "RAMP_ON", "RAMP_OFF", "IMPACT",
        "jolt ignored", "STOP_REQ", "BARCODE", "JUNCTION", "COMMAND",
        "OBST_WIDTH", "OBST_CREEP", "OBST_EDGE", "OBST_AVOID", "OBST_DONE"
    };
    return (e < 17u) ? txt[e] : "?";
}

LOCAL const char *ctrlt_cmd(UB c)
{
    static const char *const txt[5] = {"none", "LEFT", "RIGHT", "STRAIGHT",
                                       "U-TURN"};
    return (c < 5u) ? txt[c] : "?";
}

void ctrl_test_course(void)
{
    static robot_status_t snaps[CTRLT_SNAPS];
    static ctrl_log_t lg[CTRL_LOG_LEN];
    static bc_elem_t bel[BC_MAX_ELEMS];
    robot_status_t st;
    INT run, c, n, i, k, ns;
    uint32_t rate, rate_min, pass, pass_max;
    UB opts;
    ER er;

    er = ctrl_task_create();
    if(E_OK != er) {
        tm_printf((UB *)"[CTRL] ctrl_task_create failed: %d\n", er);
        return;
    }
    ctrl_set_after_turn_mm(0);
    tm_printf((UB *)"\n[CTRL] Vehicle controller: %d mm/s, stops after %d mm. "
                    "IMU %s.\n",
              CTRLT_MM_S, CTRLT_MAX_MM,
              (0u != (fault_get() & FAULT_IMU)) ? "NOT answering (no ramp "
                                                  "handling)" : "ok");

    for(run = 1; ; run++) {
        tm_printf((UB *)"\n[CTRL] Run %d: set the car at the mat edge (centre "
                        "IR on the line's"
                  " right edge), keep it still, then press\n[CTRL]   l = line "
                  "+ ramp,"
                  "  o = line + obstacle stop,  b = line + barcode command at "
                  "the cross,\n[CTRL]   a = line + obstacle: measure, go round"
                  " on the left, rejoin the line,\n[CTRL]   c = continuous"
                  " course: everything on, no distance limit,\n[CTRL]   r ="
                  " line + barcode read only (no command)"
                  "\n[CTRL]   (any key while running stops it)\n", run);
        console_flush_input();
        do {
            c = console_try_getc();
            tk_dly_tsk(50);
        } while((c != 'l') && (c != 'o') && (c != 'b') && (c != 'a')
                && (c != 'c') && (c != 'r'));
        opts = (c == 'o') ? CTRL_OPT_OBSTACLE
               : ((c == 'a') ? (CTRL_OPT_OBSTACLE | CTRL_OPT_AVOID)
               : ((c == 'c') ? (CTRL_OPT_OBSTACLE | CTRL_OPT_AVOID
                                | CTRL_OPT_BARCODE | CTRL_OPT_NO_CMD)
               : ((c == 'r') ? (CTRL_OPT_BARCODE | CTRL_OPT_NO_CMD)
               : ((c == 'b') ? CTRL_OPT_BARCODE : 0u))));
        tk_dly_tsk(500);
        console_flush_input();

        er = ctrl_run(CTRLT_MM_S,
                      (c == 'c') ? 0 : ((c == 'r') ? CTRLT_READ_MM
                                                   : CTRLT_MAX_MM), opts);
        if(E_OK != er) {
            tm_printf((UB *)"[CTRL] ctrl_run failed: %d\n", er);
            continue;
        }
        ns = 0;
        rate_min = 0xFFFFFFFFu;
        pass_max = 0u;
        if(c == 'c') {
            tm_printf((UB *)"[CTRL] continuous run started: the serial wire can"
                            " be unplugged now. Press s to stop (or lift the"
                            " car: the line search fails and it stops).\n");
        }
        while(ctrl_running()) {
            for(i = 0; (i < CTRLT_SNAP_MS / 20) && ctrl_running(); i++) {
                k = console_try_getc();
                /* c runs: only 's' stops, so a loose serial wire's noise
                   cannot end the run */
                if((k >= 0) && ((c != 'c') || (k == 's'))) {
                    ctrl_stop();
                }
                tk_dly_tsk(20);
                /* timing evidence: worst motion second of the run */
                rate = motion_loop_rate();
                motion_get_cpu(NULL, &pass);
                rate_min = (rate < rate_min) ? rate : rate_min;
                pass_max = (pass > pass_max) ? pass : pass_max;
            }
            /* rolling: the latest CTRLT_SNAPS are printed */
            ctrl_get_status(&snaps[ns % CTRLT_SNAPS]);
            ns++;
        }
        ctrl_get_status(&st);
        if(c == 'c') {
            /* The log stays in RAM: reconnect the serial wire, then ask */
            tm_printf((UB *)"[CTRL] run ended; press p to print the log\n");
            do {
                k = console_try_getc();
                tk_dly_tsk(50);
            } while(k != 'p');
        }

        tm_printf((UB *)"[CTRL] run %d (%s): %s at %d mm, heading %d x0.01 deg,"
                  " faults 0x%x, events dropped %d\n", run,
                  (c == 'c') ? "continuous course"
                  : (c == 'r') ? "line + barcode read only"
                  : (0u != (opts & CTRL_OPT_OBSTACLE)) ? "line + obstacle"
                  : ((0u != (opts & CTRL_OPT_BARCODE)) ? "line + "
                                                         "barcode" : "line + "
                                                                     "ramp"),
                  ctrlt_state(st.state),
                  (INT)st.total_cm * 10, st.heading_cdeg, (INT)st.faults,
                      (INT)ctrl_evt_dropped());
        tm_printf((UB *)"[CTRL]   REPORT: highest ramp %d mm",
                  (INT)st.max_ramp_mm);
        if(0u != (opts & CTRL_OPT_OBSTACLE)) {
            tm_printf((UB *)", obstacle width %d mm (about +/-4 cm)",
                      st.obstacle_width_mm);
        }
        tm_printf((UB *)"\n");
        tm_printf((UB *)"[CTRL]   TIMING: motion loop at least %u/s"
                  " (1000 = every 1 ms), longest pass %u us;"
                  " controller longest sample gap %u ms (period 20)\n",
                  (UINT)rate_min, (UINT)pass_max,
                  (UINT)ctrl_sample_gap_max_ms());
        n = (INT)ctrl_log_get(lg);
        tm_printf((UB *)"[CTRL]   events (t_ms, d_mm, event, arg, val):\n");
        for(i = 0; i < n; i++) {
            tm_printf((UB *)"[CTRL]     %d, %d, %s, %d, %d", (INT)lg[i].t_ms,
                      (INT)lg[i].d_mm,
                      ctrlt_evt(lg[i].type), (INT)lg[i].arg, lg[i].val);
            if(EVT_BARCODE == lg[i].type) {
                if(0u != lg[i].arg) {
                    tm_printf((UB *)"   (read '%c' %s)", (INT)lg[i].arg,
                              (0 != lg[i].val) ? "REVERSED" : "forward");
                } else {
                    tm_printf((UB *)"   (no read, %d bars/spaces)", lg[i].val);
                }
            } else if(EVT_OBST_WIDTH == lg[i].type) {
                if(2u == lg[i].arg) {
                    tm_printf((UB *)"   (width at least %d mm: the face ran"
                              " past the sweep)", lg[i].val);
                } else if(0u != lg[i].arg) {
                    tm_printf((UB *)"   (width estimate %d mm)", lg[i].val);
                } else {
                    tm_printf((UB *)"   (scan failed or nothing found: %d)",
                              lg[i].val);
                }
            } else if(EVT_OBST_EDGE == lg[i].type) {
                tm_printf((UB *)"   (%s edge %d mm left of the start%s)",
                          (0u != (lg[i].arg & 1u)) ? "left" : "right",
                          lg[i].val,
                          (lg[i].arg >= 2u) ? ", still on the face at the"
                                              " limit" : "");
            } else if(EVT_OBST_DONE == lg[i].type) {
                tm_printf((UB *)"   (%s in %d.%d s)",
                          (0u != lg[i].arg) ? "went round, line rejoined"
                                            : "FAILED", lg[i].val / 10,
                          lg[i].val % 10);
            } else if(EVT_OBST_CREEP == lg[i].type) {
                tm_printf((UB *)"   (crept in to scan, sonar %d mm)",
                          lg[i].val);
            } else if(EVT_COMMAND == lg[i].type) {
                tm_printf((UB *)"   (%s done, turned %d x0.01 deg)",
                          ctrlt_cmd(lg[i].arg),
                          lg[i].val);
            }
            tm_printf((UB *)"\n");
        }
        if(0u != (opts & CTRL_OPT_BARCODE)) {
            n = (INT)barcode_scan_last(bel, BC_MAX_ELEMS);
            tm_printf((UB *)"[CTRL]   last barcode read, %d bars/spaces (B "
                            "mm/peak mV,"
                      " w mm):\n[CTRL]    ", n);
            for(i = 0; i < n; i++) {
                k = (INT)((bel[i].w * SWEEP_UM_PER_EDGE) / 100);
                if(0u != bel[i].bar) {
                    tm_printf((UB *)" B%d.%d/%d", k / 10, k % 10, bel[i].peak);
                } else {
                    tm_printf((UB *)" w%d.%d", k / 10, k % 10);
                }
                if((i % 10) == 9) {
                    tm_printf((UB *)"\n[CTRL]    ");
                }
            }
            tm_printf((UB *)"\n");
        }
        tm_printf((UB *)"[CTRL]   robot_status every %d ms (state, d_mm, "
                        "heading, roll,"
                  " sonar_mm, flags R/I/W, pending command, last event):\n",
                      CTRLT_SNAP_MS);
        for(k = (ns > CTRLT_SNAPS) ? (ns - CTRLT_SNAPS) : 0; k < ns; k++) {
            i = k % CTRLT_SNAPS;
            tm_printf((UB *)"[CTRL]     %s, %d, %d, %d, %d, %c%c%c, %s, %s\n",
                      ctrlt_state(snaps[i].state), snaps[i].dist_mm,
                          snaps[i].heading_cdeg,
                      snaps[i].roll_cdeg, snaps[i].obstacle_mm,
                      (0u != (snaps[i].flags & CTRL_FLAG_ON_RAMP)) ? 'R' : '-',
                      (0u != (snaps[i].flags & CTRL_FLAG_IMU_OK)) ? 'I' : '-',
                      (0u != (snaps[i].flags & CTRL_FLAG_WATCHING)) ? 'W' : '-',
                      ctrlt_cmd((UB)((snaps[i].flags >> CTRL_FLAG_CMD_SHIFT)
                                     & 7u)),
                      ctrlt_evt(snaps[i].last_evt));
        }
        if(0u != (opts & CTRL_OPT_OBSTACLE)) {
            ctrlt_print_probe();
        }
        if((c == 'l') || (c == 'c') || (c == 'r')) {
            ctrlt_print_line_trace();
        }
        tm_printf((UB *)"[CTRL]   Note where the car ended up (and the nose "
                        "gap for o).\n");
    }
}
#endif /* CTRL_TEST_COURSE */
