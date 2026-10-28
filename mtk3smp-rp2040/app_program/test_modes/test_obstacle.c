/*
 * test_obstacle.c
 * Buddy 5 test mode: obstacle width calibration.
 * Built only when MOTION_TEST_MODE (test_modes.h) selects one of these modes;
 * moved out of app_main.c unchanged on 28 Oct 2026.
 */

#include "test_common.h"

#define SWEEP_UM_PER_EDGE        328  /* keep in step with UM_PER_EDGE */

#if MOTION_TEST_MODE == OBST_TEST_WIDTH
/* Buddy 5: obstacle width (course: a 10-20 cm block, detect it, measure or
   estimate its width and report it). Calibration runs: the car drives at
   WIDTH_MM_S toward a flat-faced object until the obstacle task's stop
   rule triggers, stops, then scans +/-WIDTH_HALF_DEG in the obstacle task.
   Printed afterwards: the stop, the present width estimate (seen span
   minus the narrow-target beam width) and every coarse and fine reading,
   so the estimator can be fitted to objects of known width. */
#define WIDTH_MM_S               200
#define WIDTH_MAX_MM             1200   /* stop anyway after this far */
#define WIDTH_HALF_DEG           45
#define WIDTH_DETECT_MM          600
#define WIDTH_ROW                8

LOCAL INT width_dist_mm(void)
{
    enc_diag_t l, r;

    (void)motion_get_encoder_diag(&l, &r);
    return (INT)((((l.hw_count + r.hw_count) / 2) * SWEEP_UM_PER_EDGE) / 1000);
}

LOCAL void width_print_pts(const char *name, const scan_pt_t *p, INT n)
{
    INT i;

    tm_printf((UB *)"[WIDTH]   %s (%d readings, cdeg,mm):", name, n);
    for(i = 0; i < n; i++) {
        if((i % WIDTH_ROW) == 0) {
            tm_printf((UB *)"\n[WIDTH]    ");
        }
        tm_printf((UB *)" %d,%d", p[i].cdeg, p[i].mm);
    }
    tm_printf((UB *)"\n");
}

void obst_test_width(void)
{
    static scan_pt_t pts[SCAN_LOG_MAX];
    obstacle_profile_t prof;
    INT run, i, n, d_trig, mm_trig, rest, nr;
    W sum;
    BOOL stopped;
    ER er;

    er = obstacle_task_create();
    if(E_OK != er) {
        tm_printf((UB *)"[WIDTH] obstacle_task_create failed: %d\n", er);
        return;
    }
    tm_printf((UB *)"\n[WIDTH] Obstacle width calibration: drive at %d mm/s, stop at the"
              " object, scan +/-%d deg.\n", WIDTH_MM_S, WIDTH_HALF_DEG);
    tm_printf((UB *)"[WIDTH] Use flat-faced objects ~10, ~15 and ~20 cm wide, face square"
              " to the car, about 50 cm ahead. Keep ~25 cm clear on each side.\n");

    for(run = 1; ; run++) {
        tm_printf((UB *)"\n[WIDTH] Run %d: aim the car at the object (centred, or offset if"
                  " you like), press a key (any key while driving stops it).\n", run);
        console_flush_input();
        while(console_try_getc() < 0) {
            tk_dly_tsk(50);
        }
        tk_dly_tsk(1000);
        console_flush_input();
        fault_clear(FAULT_MOTION_MASK | FAULT_SCAN_TIMEOUT);

        obstacle_watch(TRUE);
        tk_dly_tsk(300);
        motion_reset_odometry();
        motion_set_velocity(WIDTH_MM_S, WIDTH_MM_S);
        stopped = FALSE;
        d_trig = -1;
        mm_trig = -1;
        while(width_dist_mm() < WIDTH_MAX_MM) {
            if(obstacle_blocked()) {
                motion_stop();
                d_trig = width_dist_mm();
                mm_trig = obstacle_last_mm();
                stopped = TRUE;
                break;
            }
            if((console_try_getc() >= 0)
               || (0u != (fault_get() & (FAULT_MOTION_MASK | FAULT_NO_ECHO)))) {
                break;
            }
            tk_dly_tsk(5);
        }
        motion_stop();
        tk_dly_tsk(800);

        /* Range at rest: mean of up to 5 good pings */
        sum = 0;
        nr = 0;
        for(i = 0; (i < 12) && (nr < 5); i++) {
            tk_dly_tsk(70);
            if(obstacle_last_mm() > 0) {
                sum += obstacle_last_mm();
                nr++;
            }
        }
        rest = (nr > 0) ? (INT)(sum / nr) : -1;
        obstacle_watch(FALSE);

        if(!stopped) {
            tm_printf((UB *)"[WIDTH] run %d: no obstacle stop (faults 0x%x), skipped\n",
                      run, (INT)fault_get());
            continue;
        }

        er = obstacle_scan_start(WIDTH_HALF_DEG, WIDTH_DETECT_MM);
        if(E_OK == er) {
            er = obstacle_scan_wait(OBS_SCAN_TIMEOUT_MS, &prof);
        }

        tm_printf((UB *)"[WIDTH] run %d: stopped (triggered at %d mm travelled, sensor %d mm),"
                  " at rest sensor %d mm; scan %s, faults 0x%x\n", run, d_trig, mm_trig, rest,
                  (E_OK == er) ? "ok" : "FAILED", (INT)fault_get());
        if((E_OK == er) && prof.found) {
            tm_printf((UB *)"[WIDTH]   nearest %d mm at %d cdeg; seen from %d to %d cdeg"
                      " (span %d), beam %d -> width estimate %d mm (old rule %d mm),"
                      " sides %d / %d mm (left of centreline +)\n", prof.nearest_mm,
                      prof.nearest_cdeg, prof.right_cdeg, prof.left_cdeg,
                      prof.left_cdeg - prof.right_cdeg, prof.beam_cdeg, prof.width_mm,
                      prof.width_old_mm, prof.left_side_mm, prof.right_side_mm);
        } else if(E_OK == er) {
            tm_printf((UB *)"[WIDTH]   nothing within %d mm\n", WIDTH_DETECT_MM);
        }
        n = (INT)scan_get_coarse(pts, SCAN_LOG_MAX);
        width_print_pts("coarse sweep right to left", pts, n);
        n = (INT)scan_get_fine(0u, pts, SCAN_FINE_MAX);
        width_print_pts("fine, right edge", pts, n);
        n = (INT)scan_get_fine(1u, pts, SCAN_FINE_MAX);
        width_print_pts("fine, left edge", pts, n);
        tm_printf((UB *)"[WIDTH]   Note the object's real width, which face, and the nose"
                  " gap.\n");
    }
}
#endif /* OBST_TEST_WIDTH */
