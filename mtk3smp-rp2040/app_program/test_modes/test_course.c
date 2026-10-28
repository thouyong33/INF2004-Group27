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
#define CTRLT_SNAP_MS            500
#define CTRLT_SNAPS              24

LOCAL void ctrlt_print_pts(const char *name, const scan_pt_t *p, INT n)
{
    INT i;

    tm_printf((UB *)"[CTRL]   %s (%d readings, cdeg,mm):", name, n);
    for(i = 0; i < n; i++) {
        if((i % 10) == 0) {
            tm_printf((UB *)"\n[CTRL]    ");
        }
        tm_printf((UB *)" %d,%d", p[i].cdeg, p[i].mm);
    }
    tm_printf((UB *)"\n");
}

/* The obstacle scan in detail, as the width test (mode 33) prints it, plus
   the right encoder check across the scan */
LOCAL void ctrlt_print_scan(void)
{
    static scan_pt_t pts[SCAN_LOG_MAX];
    enc_check_t pre, post;
    obstacle_profile_t prof;
    INT n;

    ctrl_get_scan_diag(&pre, &post, &prof);
    tm_printf((UB *)"[CTRL]   right encoder before / after the scan: A raw %d "
                    "/ %d,"
              " B %d / %d, fixed %d / %d (A-B over the scan %d)\n",
              (INT)pre.right_a_raw, (INT)post.right_a_raw, (INT)pre.right_b,
              (INT)post.right_b, (INT)pre.fixed_edges, (INT)post.fixed_edges,
              (INT)((post.right_a_raw - pre.right_a_raw)
                    - (post.right_b - pre.right_b)));
    if(!prof.found) {
        tm_printf((UB *)"[CTRL]   scan: nothing found\n");
        return;
    }
    tm_printf((UB *)"[CTRL]   scan: nearest %d mm at %d cdeg; seen from %d to "
                    "%d cdeg"
              " (span %d), beam %d -> width %d mm (old rule %d mm)\n",
                  prof.nearest_mm,
              prof.nearest_cdeg, prof.right_cdeg, prof.left_cdeg,
              prof.left_cdeg - prof.right_cdeg, prof.beam_cdeg, prof.width_mm,
              prof.width_old_mm);
    n = (INT)scan_get_coarse(pts, SCAN_LOG_MAX);
    ctrlt_print_pts("coarse sweep right to left", pts, n);
    n = (INT)scan_get_fine(0u, pts, SCAN_FINE_MAX);
    ctrlt_print_pts("fine, right edge", pts, n);
    n = (INT)scan_get_fine(1u, pts, SCAN_FINE_MAX);
    ctrlt_print_pts("fine, left edge", pts, n);
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
    static const char *const txt[14] = {
        "-", "FAULT", "OBSTACLE", "LINE_END", "RAMP_ON", "RAMP_OFF", "IMPACT",
        "jolt ignored", "STOP_REQ", "BARCODE", "JUNCTION", "COMMAND",
        "OBST_WIDTH", "OBST_CREEP"
    };
    return (e < 14u) ? txt[e] : "?";
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
                  "the cross"
                  "\n[CTRL]   (any key while running stops it)\n", run);
        console_flush_input();
        do {
            c = console_try_getc();
            tk_dly_tsk(50);
        } while((c != 'l') && (c != 'o') && (c != 'b'));
        opts = (c == 'o') ? CTRL_OPT_OBSTACLE
               : ((c == 'b') ? CTRL_OPT_BARCODE : 0u);
        tk_dly_tsk(500);
        console_flush_input();

        er = ctrl_run(CTRLT_MM_S, CTRLT_MAX_MM, opts);
        if(E_OK != er) {
            tm_printf((UB *)"[CTRL] ctrl_run failed: %d\n", er);
            continue;
        }
        ns = 0;
        while(ctrl_running()) {
            for(i = 0; (i < CTRLT_SNAP_MS / 20) && ctrl_running(); i++) {
                if(console_try_getc() >= 0) {
                    ctrl_stop();
                }
                tk_dly_tsk(20);
            }
            if(ns < CTRLT_SNAPS) {
                ctrl_get_status(&snaps[ns++]);
            }
        }
        ctrl_get_status(&st);

        tm_printf((UB *)"[CTRL] run %d (%s): %s at %d mm, heading %d x0.01 deg,"
                  " faults 0x%x, events dropped %d\n", run,
                  (0u != (opts & CTRL_OPT_OBSTACLE)) ? "line + obstacle"
                  : ((0u != (opts & CTRL_OPT_BARCODE)) ? "line + "
                                                         "barcode" : "line + "
                                                                     "ramp"),
                  ctrlt_state(st.state),
                  st.dist_mm, st.heading_cdeg, (INT)st.faults,
                      (INT)ctrl_evt_dropped());
        tm_printf((UB *)"[CTRL]   REPORT: highest ramp %d mm",
                  (INT)st.max_ramp_mm);
        if(0u != (opts & CTRL_OPT_OBSTACLE)) {
            tm_printf((UB *)", obstacle width %d mm (about +/-4 cm)",
                      st.obstacle_width_mm);
        }
        tm_printf((UB *)"\n");
        n = (INT)ctrl_log_get(lg);
        tm_printf((UB *)"[CTRL]   events (t_ms, d_mm, event, arg, val):\n");
        for(i = 0; i < n; i++) {
            tm_printf((UB *)"[CTRL]     %d, %d, %s, %d, %d", (INT)lg[i].t_ms,
                      lg[i].d_mm,
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
        for(i = 0; i < ns; i++) {
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
            ctrlt_print_scan();
        }
        tm_printf((UB *)"[CTRL]   Note where the car ended up (and the nose "
                        "gap for o).\n");
    }
}
#endif /* CTRL_TEST_COURSE */
