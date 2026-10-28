/*
 * test_motion.c
 * Buddy 2 test modes: hand turn, PI steps, safety, CPU load.
 * Built only when MOTION_TEST_MODE (test_modes.h) selects one of these modes;
 * moved out of app_main.c unchanged on 28 Oct 2026.
 */

#include "test_common.h"

#define PI_HOLD_MS               1500
#define PI_MEASURE_MS            500
#define PI_STOP_MS               1000
#define PI_MAX_STEPS             4
#define SWEEP_UM_PER_EDGE        328  /* keep in step with UM_PER_EDGE */
#define HAND_PRINT_MS            1000

#if MOTION_TEST_MODE == MOTION_TEST_SAFETY
/* Phase 1 check (Tribunal 1): car LIFTED, wheels free. Key-paced parts:
   A timing and PI error, B STOP with commands queued, C stall (hold both
   wheels), D encoder plausibility (hold one wheel). Nothing is printed
   while the wheels turn. */
LOCAL UW safe_now_ms(void)
{
    SYSTIM tim;

    tk_get_tim(&tim);
    return tim.lo;
}

LOCAL void safe_wait_key(const char *what)
{
    tm_printf((UB *)"[SAFE] %s Press a key.\n", what);
    console_flush_input();
    while(console_try_getc() < 0) {
        tk_dly_tsk(50);
    }
}

/* Wait until neither wheel has moved for 100 ms (or 3 s); returns ms */
LOCAL UW safe_wait_still(UW t0)
{
    enc_diag_t l, r;
    W last = -1, now;
    UW still = 0;

    while((still < 100u) && ((safe_now_ms() - t0) < 3000u)) {
        (void)motion_get_encoder_diag(&l, &r);
        now = (W)(l.hw_edges + r.hw_edges);
        still = (now == last) ? (still + 5u) : 0u;
        last = now;
        tk_dly_tsk(5);
    }
    return safe_now_ms() - t0 - still;
}

/* Drive at v until a motion fault or limit_ms; returns ms to the fault */
LOCAL UW safe_drive_until_fault(INT v, UW limit_ms)
{
    UW t0 = safe_now_ms();

    motion_set_velocity((int16_t)v, (int16_t)v);
    while(((safe_now_ms() - t0) < limit_ms)
          && (0u == (fault_get() & FAULT_MOTION_MASK))) {
        tk_dly_tsk(5);
    }
    return safe_now_ms() - t0;
}

LOCAL void safe_report_fault(UW ms)
{
    UB f = fault_get();

    tm_printf((UB *)"[SAFE]   faults 0x%x after %d ms:%s%s%s%s\n", (INT)f,
              (INT)ms,
              (0u != (f & FAULT_MOTOR_STALL)) ? " MOTOR_STALL" : "",
              (0u != (f & FAULT_ENC_LEFT)) ? " ENC_LEFT" : "",
              (0u != (f & FAULT_ENC_RIGHT)) ? " ENC_RIGHT" : "",
              (0u == (f & FAULT_MOTION_MASK)) ? " none (no trip)" : "");
}

void motion_test_safety(void)
{
    static const INT speeds[3] = { 150, 300, 500 };
    enc_diag_t l0, r0, l1, r1;
    INT i, l_mm_s, r_mm_s;
    UW t0, ms;

    tm_printf((UB *)"\n[SAFE] Motion safety and timing test. Car LIFTED, "
                    "wheels free.\n");

    /* A: loop rate and speed error at three speeds */
    safe_wait_key("Part A: timing and PI error at 150/300/500 mm/s.");
    for(i = 0; i < 3; i++) {
        motion_set_velocity((int16_t)speeds[i], (int16_t)speeds[i]);
        tk_dly_tsk(1500);
        (void)motion_get_encoder_diag(&l0, &r0);
        tk_dly_tsk(1000);
        (void)motion_get_encoder_diag(&l1, &r1);
        motion_set_velocity(0, 0);
        tk_dly_tsk(800);
        l_mm_s = (INT)(((l1.hw_count - l0.hw_count) * SWEEP_UM_PER_EDGE)
                       / 1000);
        r_mm_s = (INT)(((r1.hw_count - r0.hw_count) * SWEEP_UM_PER_EDGE)
                       / 1000);
        tm_printf((UB *)"[SAFE]   %d mm/s: L %d R %d mm/s (error L %d R %d "
                        "%%), loop %d passes/s\n",
                  speeds[i], l_mm_s, r_mm_s,
                  ((l_mm_s - speeds[i]) * 100) / speeds[i],
                  ((r_mm_s - speeds[i]) * 100) / speeds[i],
                  (INT)motion_loop_rate());
    }

    /* B: STOP overtakes queued commands */
    safe_wait_key("Part B: STOP at 300 mm/s with 3 commands queued behind it.");
    motion_set_velocity(300, 300);
    tk_dly_tsk(1500);
    motion_set_velocity(300, 300);
    motion_set_velocity(300, 300);
    motion_set_velocity(300, 300);
    t0 = safe_now_ms();
    motion_stop();
    ms = safe_wait_still(t0);
    tk_dly_tsk(500);
    (void)motion_get_encoder_diag(&l0, &r0);
    tk_dly_tsk(500);
    (void)motion_get_encoder_diag(&l1, &r1);
    tm_printf((UB *)"[SAFE]   wheels still %d ms after STOP; %s\n", (INT)ms,
              ((l1.hw_edges == l0.hw_edges) && (r1.hw_edges == r0.hw_edges))
              ? "stayed stopped (queued commands discarded)" : "MOVED AGAIN");

    /* C: stall, both wheels held */
    safe_wait_key("Part C: wheels spin at 200 mm/s; after ~1 s grip BOTH wheels"
                  " firmly until they stop.");
    ms = safe_drive_until_fault(200, 10000u);
    motion_set_velocity(0, 0);
    safe_report_fault(ms);
    tk_dly_tsk(800);
    fault_clear(FAULT_MOTION_MASK);

    /* D: one wheel held */
    safe_wait_key("Part D: wheels spin at 200 mm/s; after ~1 s grip only the"
                  " LEFT wheel until it stops.");
    ms = safe_drive_until_fault(200, 10000u);
    motion_set_velocity(0, 0);
    safe_report_fault(ms);
    tk_dly_tsk(800);
    fault_clear(FAULT_MOTION_MASK);

    /* E: no false trips while running free */
    safe_wait_key("Part E: 10 s free running at 300 mm/s, hands off (expect no "
                  "trip).");
    ms = safe_drive_until_fault(300, 10000u);
    motion_set_velocity(0, 0);
    safe_report_fault(ms);
    fault_clear(FAULT_MOTION_MASK);

    tm_printf((UB *)"\n=== Motion safety test complete ===\n");
}
#endif /* MOTION_TEST_SAFETY */

#if MOTION_TEST_MODE == MOTION_TEST_CPU
/* CPU and RAM use of the motion task, car LIFTED. Each step holds a state
   for 3 s, then reads the last full second: loop passes, busy time and the
   longest pass, plus the stack high-water mark. Prints only between steps. */
LOCAL void cpu_report(const char *name)
{
    uint32_t busy, max_pass;

    motion_get_cpu(&busy, &max_pass);
    tm_printf((UB *)"[CPU] %s: %d passes/s, busy %d us/s = %d.%d%d %% CPU, "
              "longest pass %d us, %d us per pass on average\n", name,
              (INT)motion_loop_rate(), (INT)busy, (INT)(busy / 10000u),
              (INT)((busy / 1000u) % 10u), (INT)((busy / 100u) % 10u),
                  (INT)max_pass,
              (motion_loop_rate() > 0u) ? (INT)(busy / motion_loop_rate()) : 0);
}

void motion_test_cpu(void)
{
    static const INT speeds[3] = { 150, 300, 500 };
    INT i;

    /* starts the 1 MHz timer the counters use */
    ultrasonic_init();
    tm_printf((UB *)"\n[CPU] Motion task CPU and stack. Car LIFTED. Starting "
                    "in 3 s.\n");
    tk_dly_tsk(3000);

    cpu_report("idle (stopped)");
    for(i = 0; i < 3; i++) {
        motion_set_velocity((int16_t)speeds[i], (int16_t)speeds[i]);
        tk_dly_tsk(3000);
        tm_printf((UB *)"[CPU] ");
        tm_printf((UB *)"%d mm/s:\n", speeds[i]);
        cpu_report("  driving");
    }
    motion_set_velocity(0, 0);
    tk_dly_tsk(1500);
    tm_printf((UB *)"[CPU] stack: %d of %d bytes used (deepest so far)\n",
              (INT)motion_stack_used(), (INT)motion_stack_size());
    tm_printf((UB *)"\n=== CPU test complete ===\n");
}
#endif /* MOTION_TEST_CPU */

#if MOTION_TEST_MODE == MOTION_TEST_PI_STEP
/* Run a list of speed steps under PI control. Nothing is printed while the
   motors run (usermain outranks the motion task, so a blocking print would
   stall the loop); edge counts over the last PI_MEASURE_MS of each hold are
   saved and printed with the trace once the car has stopped. */
LOCAL void motion_pi_steps(const char *name, const int16_t *steps, INT n)
{
    INT i;
    /* static: usermain's stack is only 1 KB */
    static enc_diag_t l0[PI_MAX_STEPS], r0[PI_MAX_STEPS];
    static enc_diag_t l1[PI_MAX_STEPS], r1[PI_MAX_STEPS];
    INT l_mm_s, r_mm_s, diff_pct;

    if(n > PI_MAX_STEPS) {
        n = PI_MAX_STEPS;
    }

    motion_reset_odometry();
    motion_trace_start();

    for(i = 0; i < n; i++) {
        motion_set_velocity(steps[i], steps[i]);
        tk_dly_tsk(PI_HOLD_MS - PI_MEASURE_MS);
        motion_get_encoder_diag(&l0[i], &r0[i]);
        tk_dly_tsk(PI_MEASURE_MS);
        motion_get_encoder_diag(&l1[i], &r1[i]);
    }

    motion_set_velocity(0, 0);
    tk_dly_tsk(PI_STOP_MS);

    tm_printf((UB *)"\n[PI] %s trace:\n", name);
    motion_trace_dump();

    for(i = 0; i < n; i++) {
        /* edges over PI_MEASURE_MS -> mm/s at 322 um/edge */
        l_mm_s = (INT)(((l1[i].hw_count - l0[i].hw_count) * 1000
                        / PI_MEASURE_MS)
                       * SWEEP_UM_PER_EDGE / 1000);
        r_mm_s = (INT)(((r1[i].hw_count - r0[i].hw_count) * 1000
                        / PI_MEASURE_MS)
                       * SWEEP_UM_PER_EDGE / 1000);
        diff_pct = (l_mm_s != 0) ? ((r_mm_s - l_mm_s) * 100 / l_mm_s) : 0;
        tm_printf((UB *)"[PI] %s target=%4d mm/s  L=%4d  R=%4d mm/s  "
                        "R-L=%d%%\n",
                  name, (INT)steps[i], l_mm_s, r_mm_s, diff_pct);
    }
}

void motion_test_pi_step(void)
{
    static const int16_t fwd_steps[] = { 300, 500, 150 };
    static const int16_t rev_steps[] = { -300 };

    tm_printf((UB *)"\n[TEST] PI step response, lifted or on the floor\n");
    tm_printf((UB *)"[TEST] On the floor: ~1.6 m forward then ~0.5 m back\n");
    tm_printf((UB *)"[TEST] Starting in 10 s\n");
    tk_dly_tsk(10000);

    motion_pi_steps("fwd", fwd_steps, 3);
    motion_pi_steps("rev", rev_steps, 1);

    tm_printf((UB *)"\n=== PI step test complete ===\n");
}
#endif /* MOTION_TEST_PI_STEP */

#if MOTION_TEST_MODE == MOTION_TEST_HAND
/* Motors are never commanded, so the only edges are from hand turning. */
void motion_test_hand(void)
{
    enc_check_t chk;

    tm_printf((UB *)"\n[TEST] Hand-turn test: motors OFF\n");
    tm_printf((UB *)"[TEST] Mark each wheel. Turn it 10 full turns FORWARD,"
              " note cnt, then 10 turns BACK. Reset to zero the counters.\n");
    tm_printf((UB *)"[TEST] Right wheel: A = GP15 as counted, B = GP17, fixed "
                    "= taken off"
              " A beyond B + 1 (expected ~6340 for 10 turns).\n");

    motion_reset_odometry();
    while(1) {
        tk_dly_tsk(HAND_PRINT_MS);
        motion_print_encoder_diag("HAND");
        motion_get_enc_check(&chk);
        tm_printf((UB *)"[HAND] right A raw=%d  B=%d  fixed=%d in %d passes\n",
                  (INT)chk.right_a_raw, (INT)chk.right_b, (INT)chk.fixed_edges,
                  (INT)chk.bad_windows);
    }
}
#endif /* MOTION_TEST_HAND */
