/*
 * terrain_lift_test.c
 * Host test of Buddy 4's lift check (terrain_lift_update in
 * app_program/buddy4_imu/terrain.c), which stops the motors when the car
 * is picked up at the end of the demo. Feeds 20 ms periods of attitude,
 * acceleration and IR "no floor" readings. Build and run with
 * tests/host/run_host_tests.sh.
 */

#include <stdio.h>

#include "buddy4_imu/terrain.h"

#define PERIOD_MS    20U

static int g_fails = 0;
static uint32_t g_t = 0U;           /* simulated clock, ms */

static void
check(int b_ok, const char *p_name)
{
    printf("%s %s\n", b_ok ? "ok  " : "FAIL", p_name);
    if (!b_ok)
    {
        g_fails++;
    }
}

/* One stretch of fixed readings, ms long: tilt (pitch, roll, x0.01 deg
 * absolute), acceleration (mg, car frame, Z down), the IRs' "no floor"
 * and the ramp flag. Returns the time the lift fired, or 0. */
static uint32_t
run(uint32_t ms, int pitch, int roll, int ax, int az, int b_no_floor,
    int b_ramp, lift_event_t *p_lift)
{
    imu_attitude_t att = {0};
    imu_raw_t cal = {0};
    uint32_t end = g_t + ms;

    att.pitch_cdeg = (int16_t)pitch;
    att.roll_cdeg = (int16_t)roll;
    cal.ax = (int16_t)ax;
    cal.az = (int16_t)az;
    for (; g_t < end; g_t += PERIOD_MS)
    {
        if (terrain_lift_update(g_t, &att, &cal, (0 != b_no_floor),
                                (0 != b_ramp), p_lift))
        {
            return g_t;
        }
    }
    return 0U;
}

static void
start(int16_t pitch0, int16_t roll0)
{
    terrain_reset(pitch0, roll0);
    g_t = 1000U;
}

int
main(void)
{
    lift_event_t lift = {0};
    int16_t tilt;
    int16_t acc;

    /* Driving: a seam bump, then a cross (all three IRs black 180 ms) */
    start(100, -50);
    (void)run(2000U, 100, -50, 0, -1000, 0, 0, &lift);
    (void)run(40U, 100, -50, 400, -1350, 0, 0, &lift);
    check(0U == run(180U, 100, -50, 0, -1000, 1, 0, &lift),
          "a bump then a cross: no lift");
    check(0U == run(2000U, 100, -50, 0, -1000, 0, 0, &lift),
          "driving on: no lift");
    terrain_lift_peaks(&tilt, &acc);
    printf("     peaks so far: tilt %d x0.01 deg, |a| off 1 g %d mg\n",
           tilt, acc);

    /* Level lift: a pull up, then no floor */
    (void)run(60U, 100, -50, 0, -1450, 0, 0, &lift);
    check((0U != run(1000U, 100, -50, 0, -1000, 1, 0, &lift))
          && (TERRAIN_LIFT_ACCEL == lift.cue),
          "level lift fires on the acceleration cue");
    check(0U == run(500U, 100, -50, 0, -1000, 1, 0, &lift),
          "fires once per run");

    /* Tilted, gentle lift: no acceleration cue */
    start(0, 0);
    (void)run(500U, 0, 0, 0, -1000, 0, 0, &lift);
    check((0U != run(1000U, 3000, 0, 0, -1000, 1, 0, &lift))
          && (TERRAIN_LIFT_TILT == lift.cue),
          "tilted lift fires on the tilt cue");

    /* No floor with no IMU cue (a dark patch) */
    start(0, 0);
    check(0U == run(3000U, 0, 0, 0, -1000, 1, 0, &lift),
          "no floor without an IMU cue: no lift");

    /* The ramp: entry jolt, on it (tilted IR reads black), landing */
    start(0, 0);
    (void)run(40U, 0, 1300, 500, -1500, 1, 0, &lift);
    check(0U == run(800U, 0, 1300, 300, -1400, 1, 1, &lift),
          "on the ramp: no lift");
    check(0U == run(400U, 0, 0, 350, -1400, 1, 0, &lift),
          "landing (acceleration ignored 1 s): no lift");

    /* Lifted with a small tilt: the hump detector holds it "on the ramp" */
    start(0, 0);
    (void)run(40U, 0, 0, 0, -1450, 0, 0, &lift);
    (void)run(200U, 0, 800, 0, -1000, 1, 0, &lift);
    check((0U != run(4000U, 0, 800, 0, -1000, 1, 1, &lift))
          && (TERRAIN_LIFT_RAMP == lift.cue),
          "lifted, held as a ramp: fires after 3 s of no floor");
    start(0, 0);
    check((0U != run(1000U, 3000, 0, 0, -1000, 1, 1, &lift))
          && (TERRAIN_LIFT_TILT == lift.cue),
          "tilted past 25 deg while held as a ramp: fires");

    /* A cue only arms the IR check for 2 s */
    start(0, 0);
    (void)run(40U, 0, 0, 0, -1500, 0, 0, &lift);
    (void)run(2100U, 0, 0, 0, -1000, 0, 0, &lift);
    check(0U == run(1000U, 0, 0, 0, -1000, 1, 0, &lift),
          "a cue older than 2 s does not arm");

    /* Upside down (roll across 180 deg) and a drop (|a| under 1 g) */
    start(0, 0);
    check(0U != run(1000U, 0, -17900, 0, 1000, 1, 0, &lift),
          "upside down fires");
    start(0, 0);
    (void)run(40U, 0, 0, 0, -600, 0, 0, &lift);
    check(0U != run(1000U, 0, 0, 0, -1000, 1, 0, &lift),
          "a drop fires");

    printf("%s\n", (0 == g_fails) ? "ALL PASS" : "FAILED");
    return g_fails;
}
