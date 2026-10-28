/*
 * test_imu.c
 * Buddy 4 test mode: ramp roll log.
 * Built only when MOTION_TEST_MODE (test_modes.h) selects one of these modes;
 * moved out of app_main.c unchanged on 28 Oct 2026.
 */

#include "test_common.h"

/* ruler, 2026-10-07; confirmed by compass spin */
#define HDG_TRACK_MM             114
#define RAMP_SPEED_MM_S          150
#define RAMP_DIST_MM             500
#define RAMP_LOG_LEN             300   /* 20 ms samples: 6 s */
#define SWEEP_UM_PER_EDGE        328  /* keep in step with UM_PER_EDGE */

#if MOTION_TEST_MODE == IMU_TEST_RAMP_ROLL
/* Phase 2 step 1: one-sided hump log. The car drives RAMP_DIST_MM straight
   at RAMP_SPEED_MM_S under the normal PI (no heading hold) with only the
   RIGHT wheel going over the ramp. Every 20 ms it stores roll, pitch,
   heading, both wheels' distance and the horizontal jolt in RAM; the log
   is printed once the car has stopped. The stall/encoder checks stay on
   (no hump mask), to see whether the ramp or the IR bracket trips them. */
typedef struct {
    int16_t d_mm;        /* mean forward distance */
    int16_t l_mm;        /* left wheel distance */
    int16_t r_mm;        /* right wheel distance */
    int16_t roll;        /* x0.01 deg, right side down + */
    int16_t pitch;       /* x0.01 deg, nose up + */
    uint16_t hdg;        /* x0.01 deg clockwise from magnetic north */
    int16_t jolt_mg;     /* larger of |d ax|, |d ay| since last sample */
} ramp_pt_t;

LOCAL INT ramp_wrap_cdeg(INT d)
{
    while(d > 18000) {
        d -= 36000;
    }
    while(d < -18000) {
        d += 36000;
    }
    return d;
}

LOCAL INT ramp_sin_cdeg_x1000(INT cdeg)
{
    /* sin by Taylor to x^7 (no libm), x1000 */
    float x = (float)cdeg * (3.14159265f / 18000.0f);
    float x2 = x * x;

    return (INT)(1000.0f * x * (1.0f - (x2 / 6.0f) * (1.0f - (x2 / 20.0f)
                 * (1.0f - (x2 / 42.0f)))));
}

LOCAL void ramp_pass(const char *name)
{
    static ramp_pt_t log[RAMP_LOG_LEN];
    imu_attitude_t att;
    imu_raw_t s;
    enc_diag_t l, r;
    INT n = 0, i, k, base_n = 0;
    W roll0 = 0, pitch0 = 0, hdg_unwrap = 0;
    INT hdg_first = -1, d, j;
    INT prev_ax = 0, prev_ay = 0;
    BOOL b_have_prev = FALSE, b_stopping = FALSE;
    UW t_stop = 0;
    SYSTIM tim;
    INT roll_min = 0, roll_max = 0, i_min = 0, i_max = 0, jolt_max = 0,
        i_jolt = 0;
    INT h0, h1, enc_yaw;

    /* Rest reference: 0.5 s still */
    for(i = 0; i < 25; i++) {
        if(E_OK == imu_read_attitude(&att, NULL)) {
            roll0 += att.roll_cdeg;
            pitch0 += att.pitch_cdeg;
            if(hdg_first < 0) {
                hdg_first = att.heading_cdeg;
            }
            hdg_unwrap += ramp_wrap_cdeg((INT)att.heading_cdeg - hdg_first);
            base_n++;
        }
        tk_dly_tsk(20);
    }
    if(base_n == 0) {
        tm_printf((UB *)"[RAMP] IMU read failed\n");
        return;
    }
    roll0 /= base_n;
    pitch0 /= base_n;
    h0 = hdg_first + (INT)(hdg_unwrap / base_n);

    fault_clear(FAULT_MOTION_MASK);
    motion_reset_odometry();
    motion_set_velocity(RAMP_SPEED_MM_S, RAMP_SPEED_MM_S);

    while(n < RAMP_LOG_LEN) {
        if(E_OK == imu_read_attitude(&att, &s)) {
            (void)motion_get_encoder_diag(&l, &r);
            log[n].l_mm = (int16_t)((l.hw_count * SWEEP_UM_PER_EDGE) / 1000);
            log[n].r_mm = (int16_t)((r.hw_count * SWEEP_UM_PER_EDGE) / 1000);
            log[n].d_mm = (int16_t)((log[n].l_mm + log[n].r_mm) / 2);
            log[n].roll = (int16_t)(att.roll_cdeg - roll0);
            log[n].pitch = (int16_t)(att.pitch_cdeg - pitch0);
            log[n].hdg = att.heading_cdeg;
            j = 0;
            if(b_have_prev) {
                j = (s.ax > prev_ax) ? (s.ax - prev_ax) : (prev_ax - s.ax);
                k = (s.ay > prev_ay) ? (s.ay - prev_ay) : (prev_ay - s.ay);
                if(k > j) {
                    j = k;
                }
            }
            prev_ax = s.ax;
            prev_ay = s.ay;
            b_have_prev = TRUE;
            log[n].jolt_mg = (int16_t)j;
            n++;

            tk_get_tim(&tim);
            if((!b_stopping) && ((log[n - 1].d_mm >= RAMP_DIST_MM)
                               || (0u != (fault_get() & FAULT_MOTION_MASK)))) {
                motion_set_velocity(0, 0);
                b_stopping = TRUE;
                t_stop = tim.lo;
            }
            if(b_stopping && ((tim.lo - t_stop) > 800u)) {
                break;
            }
        }
        tk_dly_tsk(20);
    }
    motion_set_velocity(0, 0);
    tk_dly_tsk(300);

    /* Heading after, averaged like the rest reference */
    hdg_first = -1;
    hdg_unwrap = 0;
    base_n = 0;
    for(i = 0; i < 25; i++) {
        if(E_OK == imu_read_attitude(&att, NULL)) {
            if(hdg_first < 0) {
                hdg_first = att.heading_cdeg;
            }
            hdg_unwrap += ramp_wrap_cdeg((INT)att.heading_cdeg - hdg_first);
            base_n++;
        }
        tk_dly_tsk(20);
    }
    h1 = (base_n > 0) ? (hdg_first + (INT)(hdg_unwrap / base_n)) : h0;

    tm_printf((UB *)"\n[RAMP] %s: %d samples, rest roll %d pitch %d (x0.01 "
                    "deg), faults 0x%x\n",
              name, n, (INT)roll0, (INT)pitch0, (INT)fault_get());
    tm_printf((UB *)"[RAMP] i,d_mm,l_mm,r_mm,roll,pitch,hdg,jolt_mg  "
                    "(roll/pitch from rest)\n");
    for(i = 0; i < n; i++) {
        tm_printf((UB *)"[RAMP] %d,%d,%d,%d,%d,%d,%d,%d\n", i, log[i].d_mm,
                  log[i].l_mm,
                  log[i].r_mm, log[i].roll, log[i].pitch, (INT)log[i].hdg,
                      log[i].jolt_mg);
        if(log[i].roll < roll_min) {
            roll_min = log[i].roll;
            i_min = i;
        }
        if(log[i].roll > roll_max) {
            roll_max = log[i].roll;
            i_max = i;
        }
        if(log[i].jolt_mg > jolt_max) {
            jolt_max = log[i].jolt_mg;
            i_jolt = i;
        }
    }

    d = ramp_wrap_cdeg(h1 - h0);
    enc_yaw = (n > 0)
        ? (INT)(((log[n - 1].l_mm - log[n - 1].r_mm) * 573L * 100L)
                / (10L * HDG_TRACK_MM)) : 0;
    tm_printf((UB *)"[RAMP] summary %s:\n", name);
    tm_printf((UB *)"[RAMP]   roll range %d .. %d (x0.01 deg) at d=%d / %d "
                    "mm\n",
              roll_min, roll_max, log[i_min].d_mm, log[i_max].d_mm);
    tm_printf((UB *)"[RAMP]   if roll = asin(h / %d mm): right-up h ~ %d mm\n",
              HDG_TRACK_MM,
                  (HDG_TRACK_MM * ramp_sin_cdeg_x1000(-roll_min)) / 1000);
    tm_printf((UB *)"[RAMP]   compass turned %d (x0.01 deg, + = right), "
                    "encoders %d"
              " (L-R over track, + = right)\n", d, enc_yaw);
    tm_printf((UB *)"[RAMP]   wheels L %d R %d mm, largest jolt %d mg at d=%d "
                    "mm\n",
              (n > 0) ? log[n - 1].l_mm : 0, (n > 0) ? log[n - 1].r_mm : 0,
              jolt_max, log[i_jolt].d_mm);
    tm_printf((UB *)"[RAMP]   Now measure how far sideways the car ended up "
                    "from its"
              " start line (mm, left or right).\n");
}

void imu_test_ramp_roll(void)
{
    static const char *const passes[3] = {
        "Pass 1/3: FLAT floor, no ramp",
        "Pass 2/3: SMALL ramp (1.0 cm) under the RIGHT wheel only",
        "Pass 3/3: TALL ramp (2.4 cm) under the RIGHT wheel only",
    };
    INT p;

    i2c1_init();
    if(E_OK != imu_init()) {
        tm_printf((UB *)"[RAMP] imu_init failed, check the IMU\n");
        return;
    }
    tm_printf((UB *)"\n[RAMP] One-sided hump log: %d mm straight at %d mm/s, "
                    "no heading hold.\n",
              RAMP_DIST_MM, RAMP_SPEED_MM_S);
    tm_printf((UB *)"[RAMP] Mark the start line beside the car. Ramp start ~15 "
                    "cm ahead of\n");
    tm_printf((UB *)"[RAMP] the RIGHT wheel, lined up so only that wheel "
                    "climbs it.\n");

    for(p = 0; p < 3; p++) {
        tm_printf((UB *)"\n[RAMP] %s.\n", passes[p]);
        tm_printf((UB *)"[RAMP] Set up, keep the car still, then press a key "
                        "to drive.\n");
        console_flush_input();
        while(console_try_getc() < 0) {
            tk_dly_tsk(50);
        }
        tk_dly_tsk(1000);           /* hands off */
        ramp_pass(passes[p]);
    }
    tm_printf((UB *)"\n=== One-sided hump log complete ===\n");
}
#endif /* IMU_TEST_RAMP_ROLL */
