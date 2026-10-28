/**
 * @file vehicle_ctrl.c
 * @brief Vehicle controller task.
 */

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>

#include "../common/critical.h"
#include "../common/faults.h"
#include "../common/ctrl_events.h"
#include "../buddy2_motion/motion_task.h"
#include "../buddy3_line_barcode/line.h"
#include "../buddy3_line_barcode/barcode_scan.h"
#include "../buddy4_imu/imu.h"
#include "../buddy4_imu/terrain.h"
#include "../buddy5_ultrasonic/obstacle.h"
#include "../buddy5_ultrasonic/ultrasonic.h"
#include "vehicle_ctrl.h"

/* Below motion (5) so the I2C reads never delay the motor loop; same as
 * the obstacle task, above the line task (7) */
#define CTRL_TASK_PRI        6u
#define CTRL_STACK_SIZE      2048u
#define CTRL_FLG_RUN         (1u << 0)

#define CTRL_PERIOD_MS       20u     /* IMU / terrain sampling */
#define CTRL_REST_SAMPLES    25      /* 0.5 s rest attitude before moving */
#define CTRL_WATCH_SETTLE_MS 300u    /* a few pings before driving */
#define CTRL_IMU_FAIL_MAX    10u     /* reads failed in a row = FAULT_IMU */
#define CTRL_END_WAIT_MS     3000u   /* line task to wind down (a search) */
/* Creep in before the scan: every scan has a fixed echo at ~290-315 mm
 * (the floor, the sonar points slightly down). Stopping from 100 mm/s
 * leaves the face at ~290 mm, inside it, and the width gate then took the
 * floor in across +/-32 deg (9 cm face read 151-169 mm, ctrl_course_v13).
 * At 200 mm the face is clear of it, as in the width calibration (stops
 * from 200 mm/s, face ~250 mm). Sonar ~60 mm behind the nose: nose gap
 * ~14 cm; the creep limit keeps it >= 8 cm even if every ping is lost. */
#define CTRL_SCAN_RANGE_MM   200
#define CTRL_CREEP_MM_S      40
#define CTRL_CREEP_MAX_MM    150
#define CTRL_CREEP_PING_MS   65u     /* >= 60 ms between pings */
#define CTRL_CREEP_TIMEOUT_MS 6000u

/* Impact confirmation (as in the roam demo, 21 Oct) */
#define CTRL_SPD_WIN_MS      100u    /* forward speed window */
#define CTRL_SPD_RING        8       /* > CTRL_SPD_WIN_MS / CTRL_PERIOD_MS */
#define CTRL_SPD_SETTLE_MS   400u    /* no speed check while pulling away */
#define CTRL_JOLT_CONFIRM_MS 250u
#define CTRL_JOLT_SLOW_PCT   60

/* Junction turn: the IR spots are ~130 mm ahead of the axle and the arm
 * is first seen at its near edge, 9 mm before its centre line */
#define CTRL_JUNC_AHEAD_MM   139
#define CTRL_JUNC_MM_S       60
#define CTRL_JUNC_BRAKE_MM   5       /* stop command this early at 60 mm/s */
#define CTRL_JUNC_FWD_MS     4000u
#define CTRL_TURN_MM_S       60
#define CTRL_TRIM_MM_S       30
#define CTRL_TRIM_CDEG       60      /* re-turn if a turn misses by more */
/* Both IRs report a cross: the second report within this is the same one */
#define CTRL_JUNC_SAME_MM    60

#define UM_PER_EDGE          328L
#define TRACK_MM             114L

static ID g_ctrl_tsk = 0;
static ID g_ctrl_flg = 0;

static volatile int16_t g_speed = 100;
static volatile int16_t g_max_mm = 450;
static volatile uint8_t g_opts = 0u;
static volatile uint8_t g_running = 0u;
static volatile int16_t g_after_mm = 0;
static bool g_b_imu_ok = false;

static robot_status_t g_status;
static int16_t g_obst_width = 0;
static uint8_t g_max_ramp = 0u;
static crit_lock_t g_status_lock;

static ctrl_log_t g_log[CTRL_LOG_LEN];
static volatile uint32_t g_log_n = 0u;
static uint32_t g_t0 = 0u;

static uint32_t
now_ms(void)
{
    SYSTIM tim;

    tk_get_tim(&tim);
    return (uint32_t)tim.lo;
}

static int32_t
dist_mm(void)
{
    enc_diag_t l, r;

    (void)motion_get_encoder_diag(&l, &r);
    return (((l.hw_count + r.hw_count) / 2) * UM_PER_EDGE) / 1000L;
}

/* Encoder heading since the run started (odometry reset), within +/-180 */
static int32_t
heading_cdeg(void)
{
    enc_diag_t l, r;
    float half_diff;
    int32_t h;

    (void)motion_get_encoder_diag(&l, &r);
    half_diff = ((float)(r.hw_count - l.hw_count)) * 0.5f;
    h = (int32_t)((half_diff * (float)UM_PER_EDGE * 36000.0f)
                  / (3.14159265f * (float)TRACK_MM * 1000.0f));
    h %= 36000;
    if (h > 18000)
    {
        h -= 36000;
    }
    else if (h < -18000)
    {
        h += 36000;
    }
    return h;
}

static void
log_evt(uint8_t type, uint8_t arg, int16_t val, int32_t d)
{
    ctrl_log_t *p_e;

    if (g_log_n >= CTRL_LOG_LEN)
    {
        return;
    }
    p_e = &g_log[g_log_n];
    p_e->t_ms = (uint16_t)(now_ms() - g_t0);
    p_e->d_mm = (int16_t)d;
    p_e->type = type;
    p_e->arg = arg;
    p_e->val = val;
    g_log_n++;
}

static void
status_set(uint8_t state, uint8_t last_evt, uint8_t flags, int32_t d,
           int32_t hdg, int32_t roll)
{
    UINT sts;

    CRIT_ENTER(&g_status_lock, sts);
    g_status.state = state;
    g_status.faults = fault_get();
    if (EVT_NONE != last_evt)
    {
        g_status.last_evt = last_evt;
    }
    g_status.flags = flags;
    g_status.dist_mm = (int16_t)d;
    g_status.heading_cdeg = (int16_t)hdg;
    g_status.roll_cdeg = (int16_t)roll;
    g_status.obstacle_mm = obstacle_last_mm();
    g_status.obstacle_width_mm = g_obst_width;
    g_status.max_ramp_mm = g_max_ramp;
    CRIT_EXIT(&g_status_lock, sts);
}

/* Rest attitude for the terrain detectors; false if the IMU did not answer */
static bool
rest_attitude(int32_t *p_roll0)
{
    imu_attitude_t att;
    int32_t p0 = 0;
    int32_t r0 = 0;
    int32_t n = 0;
    int32_t i;

    for (i = 0; i < CTRL_REST_SAMPLES; i++)
    {
        if (E_OK == imu_read_attitude(&att, NULL))
        {
            p0 += att.pitch_cdeg;
            r0 += att.roll_cdeg;
            n++;
        }
        tk_dly_tsk(CTRL_PERIOD_MS);
    }
    if (0 == n)
    {
        return false;
    }
    *p_roll0 = r0 / n;
    terrain_reset((int16_t)(p0 / n), (int16_t)(r0 / n));
    return true;
}

/* Barcode read -> command (user's mapping; see vehicle_ctrl.h) */
static uint8_t
cmd_for(uint8_t ch, bool b_reversed)
{
    if ('A' == ch)
    {
        return b_reversed ? CTRL_CMD_RIGHT : CTRL_CMD_LEFT;
    }
    if ('Z' == ch)
    {
        return b_reversed ? CTRL_CMD_UTURN : CTRL_CMD_STRAIGHT;
    }
    return CTRL_CMD_NONE;
}

/* Wind the line task down; its LINE_END(STOPPED) is ours, not news */
static void
line_wind_down(void)
{
    uint32_t j;

    line_follow_stop();
    for (j = 0u; (j < CTRL_END_WAIT_MS) && (LINE_END_NONE == line_follow_end());
         j += CTRL_PERIOD_MS)
    {
        tk_dly_tsk(CTRL_PERIOD_MS);
    }
}

/* Re-queue what arrived while the controller was busy, minus line ends
 * and junctions (the other IR's report of the cross just turned at, in
 * the distance before the line task restarted) */
static void
requeue_events(void)
{
    ctrl_evt_t evt[8];
    uint32_t n = 0u;
    uint32_t i;

    while ((n < 8u) && (E_OK == ctrl_evt_wait(&evt[n], TMO_POL)))
    {
        if ((EVT_LINE_END != evt[n].type) && (EVT_JUNCTION != evt[n].type))
        {
            n++;
        }
    }
    for (i = 0u; i < n; i++)
    {
        ctrl_evt_post(evt[i].type, evt[i].arg, evt[i].val);
    }
}

/* Encoder heading since following started, x0.01 deg, not wrapped (the
 * same reference as line_direction_cdeg()) */
static int32_t
heading_raw_cdeg(void)
{
    enc_diag_t l, r;
    float half_diff;

    (void)motion_get_encoder_diag(&l, &r);
    half_diff = ((float)(r.hw_count - l.hw_count)) * 0.5f;
    return (int32_t)((half_diff * (float)UM_PER_EDGE * 36000.0f)
                     / (3.14159265f * (float)TRACK_MM * 1000.0f));
}

/* Junction command: stop, drive on until the axle is over the junction,
 * turn on the spot to 90 deg (180 for a U-turn) from the LINE's direction:
 * at the junction the car is mid-weave, up to ~15 deg off the line, and
 * a plain 90 deg turn kept that error (runs 26 Oct ended 75-96 deg off).
 * Returns the angle turned (x0.01 deg) or a large value if a motion fault
 * stopped it. */
static int32_t
junction_turn(uint8_t cmd, int32_t d_seen)
{
    uint32_t waited = 0u;
    int32_t target;
    int32_t got;
    int32_t cdeg;
    int32_t dir;

    dir = line_direction_cdeg();
    line_wind_down();
    target = d_seen + CTRL_JUNC_AHEAD_MM - CTRL_JUNC_BRAKE_MM;
    if (dist_mm() < target)
    {
        (void)motion_set_velocity(CTRL_JUNC_MM_S, CTRL_JUNC_MM_S);
        while ((dist_mm() < target) && (waited < CTRL_JUNC_FWD_MS)
               && (0u == (fault_get() & FAULT_MOTION_MASK)))
        {
            tk_dly_tsk(CTRL_PERIOD_MS);
            waited += CTRL_PERIOD_MS;
        }
    }
    motion_stop();
    tk_dly_tsk(300u);
    if (0u != (fault_get() & FAULT_MOTION_MASK))
    {
        return 99999;
    }

    cdeg = (CTRL_CMD_LEFT == cmd) ? 9000
           : ((CTRL_CMD_RIGHT == cmd) ? -9000 : 18000);
    cdeg += dir - heading_raw_cdeg();
    got = motion_turn_cdeg(cdeg, CTRL_TURN_MM_S);
    if (((cdeg - got) > CTRL_TRIM_CDEG) || ((got - cdeg) > CTRL_TRIM_CDEG))
    {
        got += motion_turn_cdeg(cdeg - got, CTRL_TRIM_MM_S);
    }
    tk_dly_tsk(200u);
    return got;
}

/* Creep toward the obstacle until the sonar reads CTRL_SCAN_RANGE_MM (see
 * there), then stop; returns the last reading (-1 = none) */
static int32_t
creep_to_scan_range(void)
{
    int32_t d0;
    int32_t mm = -1;
    uint32_t echo_us;
    uint32_t t;

    d0 = dist_mm();
    (void)motion_set_velocity(CTRL_CREEP_MM_S, CTRL_CREEP_MM_S);
    for (t = 0u; t < CTRL_CREEP_TIMEOUT_MS; t += CTRL_CREEP_PING_MS)
    {
        if (E_OK == ultrasonic_read_us(&echo_us))
        {
            mm = ultrasonic_us_to_mm(echo_us);
            if ((mm > 0) && (mm <= CTRL_SCAN_RANGE_MM))
            {
                break;
            }
        }
        if (((dist_mm() - d0) >= CTRL_CREEP_MAX_MM)
            || (0u != (fault_get() & FAULT_MOTION_MASK)))
        {
            break;
        }
        tk_dly_tsk(CTRL_CREEP_PING_MS);
    }
    motion_stop();
    tk_dly_tsk(300u);
    return mm;
}

/*----------------------------------------------------------------------------
 * Obstacle width by side-stepping (28 Oct). The turning scan cannot see
 * width: a flat face echoes only when the sonar looks at it square-on, the
 * sonar is ~70 mm ahead of the axle and barely moves sideways as the car
 * turns, so every face over ~7 cm was seen over the same ~50 deg (9 / 15 /
 * 23 cm faces: 47-52 / 50-56 / 49-57 deg, ctrl_course_v17). Instead the car
 * faces the line's direction and steps sideways (turn 90, drive, turn
 * back, ping) until the face's echo is lost: that is an edge, at the
 * sonar's sideways position. Width = left edge - right edge, measured on
 * the wheels. The range stays ~190 mm, clear of the ~290 mm floor echo.
 *---------------------------------------------------------------------------*/
#define PROBE_STEP_MM        25      /* sideways step: edge to +/-12 mm */
#define PROBE_MAX_MM         250     /* per side: faces up to ~50 cm */
/* A reading is the face if it is no further than this beyond the median
 * of the face readings so far (a nearer one always is): the face's range
 * drifts 25-40 mm along it, and comparing with the first reading ended
 * probes early on one stray reading (ctrl_course_v19 runs 7, 8, 13) */
#define PROBE_BAND_MM        25
#define PROBE_OFF_NEEDED     2       /* off-face readings in a row = edge */
#define PROBE_DRIVE_MM_S     60
#define PROBE_BRAKE_MM       3       /* stop command this early */
#define PROBE_TURN_MM_S      80
#define PROBE_PINGS          3       /* median of */
#define PROBE_PING_MS        65u
#define PROBE_SETTLE_MS      150u
/* The echo runs on past each edge (the beam still clips the face near the
 * corner): 9 runs on 9 / 15 / 23 cm faces read 17 mm wide per edge on
 * average (28 Oct, ctrl_course_v18); with it taken off, 8 of 9 widths
 * were within +/-2 cm, the worst 3.2 cm under */
#define PROBE_EDGE_OVER_MM   17

/* Drive past on the left (the car's right side faces the obstacle) */
#define AVOID_HALF_R_MM      70      /* centre to the right wheel's outside */
#define AVOID_HALF_L_MM      85      /* centre to the left IR bracket */
#define AVOID_SENSOR_AHEAD_MM 70     /* sonar ahead of the axle */
#define AVOID_MARGIN_MM      40      /* beyond the width estimate's error */
/* plan for at least this width (course obstacles are 10-20 cm): a low
 * estimate left too little room (v19, the car brushed the box) */
#define AVOID_MIN_WIDTH_MM   100
/* not measurable from the front: assumed */
#define AVOID_DEPTH_MM       200
#define AVOID_MM_S           100
#define AVOID_AFTER_MM       300     /* follow this far after rejoining */

static ctrl_probe_pt_t g_probe[CTRL_PROBE_MAX];
static volatile uint32_t g_probe_n = 0u;
static int16_t g_face[CTRL_PROBE_MAX];      /* face readings, sorted */
static uint32_t g_face_n = 0u;

/* Keep a face reading (sorted) and return the median of those kept */
static int32_t
face_add(int32_t mm)
{
    uint32_t j;

    if (g_face_n < CTRL_PROBE_MAX)
    {
        for (j = g_face_n; (j > 0u) && (g_face[j - 1u] > mm); j--)
        {
            g_face[j] = g_face[j - 1u];
        }
        g_face[j] = (int16_t)mm;
        g_face_n++;
    }
    return g_face[g_face_n / 2u];
}

/* Turn on the spot to an absolute heading (heading_raw_cdeg reference) */
static void
probe_turn_to(int32_t target_cdeg)
{
    int32_t cdeg;
    int32_t got;

    cdeg = target_cdeg - heading_raw_cdeg();
    got = motion_turn_cdeg(cdeg, PROBE_TURN_MM_S);
    if (((cdeg - got) > CTRL_TRIM_CDEG) || ((got - cdeg) > CTRL_TRIM_CDEG))
    {
        (void)motion_turn_cdeg(cdeg - got, CTRL_TRIM_MM_S);
    }
}

/* Drive straight forward mm at mm_s and stop; returns the distance */
static int32_t
probe_drive(int32_t mm, int16_t mm_s)
{
    int32_t d0;
    uint32_t waited = 0u;
    uint32_t limit_ms;

    limit_ms = (uint32_t)((mm * 1000) / mm_s) + CTRL_JUNC_FWD_MS;
    d0 = dist_mm();
    (void)motion_set_velocity(mm_s, mm_s);
    while (((dist_mm() - d0) < (mm - PROBE_BRAKE_MM))
           && (waited < limit_ms)
           && (0u == (fault_get() & FAULT_MOTION_MASK)))
    {
        tk_dly_tsk(CTRL_PERIOD_MS);
        waited += CTRL_PERIOD_MS;
    }
    motion_stop();
    tk_dly_tsk(PROBE_SETTLE_MS);
    return dist_mm() - d0;
}

/* Median of PROBE_PINGS readings, mm (-1 = no echo) */
static int32_t
probe_ping(void)
{
    int32_t v[PROBE_PINGS];
    int32_t t;
    uint32_t echo_us;
    uint32_t i;
    uint32_t j;

    for (i = 0u; i < PROBE_PINGS; i++)
    {
        v[i] = (E_OK == ultrasonic_read_us(&echo_us))
               ? ultrasonic_us_to_mm(echo_us) : -1;
        tk_dly_tsk(PROBE_PING_MS);
    }
    for (i = 1u; i < PROBE_PINGS; i++)
    {
        t = v[i];
        for (j = i; (j > 0u) && (v[j - 1u] > t); j--)
        {
            v[j] = v[j - 1u];
        }
        v[j] = t;
    }
    return v[PROBE_PINGS / 2u];
}

static void
probe_log(int32_t y, int32_t mm)
{
    if (g_probe_n < CTRL_PROBE_MAX)
    {
        g_probe[g_probe_n].y_mm = (int16_t)y;
        g_probe[g_probe_n].mm = (int16_t)mm;
        g_probe_n++;
    }
}

/* Step sideways from *p_y (left +) by dir (+1 / -1) until the face's echo
 * is lost PROBE_OFF_NEEDED steps in a row; returns the edge (midway
 * between the last step on the face and the first off it), *p_capped if
 * PROBE_MAX_MM went by still on it. *p_ref is the face's median range. */
static int32_t
probe_edge(int32_t dir_cdeg, int32_t side, int32_t *p_ref, int32_t *p_y,
           bool *p_capped)
{
    int32_t mm;
    int32_t moved;
    int32_t y_on = *p_y;
    int32_t y_off = *p_y;
    uint32_t offs = 0u;

    *p_capped = false;
    for (;;)
    {
        if (((side > 0) && (*p_y >= PROBE_MAX_MM))
            || ((side < 0) && (*p_y <= -PROBE_MAX_MM))
            || (0u != (fault_get() & FAULT_MOTION_MASK)))
        {
            *p_capped = true;
            return y_on;
        }
        probe_turn_to(dir_cdeg + (side * 9000));
        moved = probe_drive(PROBE_STEP_MM, PROBE_DRIVE_MM_S);
        *p_y += side * moved;
        probe_turn_to(dir_cdeg);
        tk_dly_tsk(PROBE_SETTLE_MS);
        mm = probe_ping();
        probe_log(*p_y, mm);
        if ((mm > 0) && (mm <= (*p_ref + PROBE_BAND_MM)))
        {
            y_on = *p_y;
            offs = 0u;
            *p_ref = face_add(mm);
        }
        else
        {
            if (0u == offs)
            {
                y_off = *p_y;
            }
            if (++offs >= PROBE_OFF_NEEDED)
            {
                return (y_on + y_off) / 2;
            }
        }
    }
}

/* Find both edges of the obstacle ahead and log them; the car ends just
 * past the left edge (*p_y, mm left of the start), facing the line's
 * direction (*p_dir). *p_left is the left edge, *p_face_mm the range.
 * Returns the width (0 = failed). */
static int32_t
probe_obstacle(int32_t *p_dir, int32_t *p_y, int32_t *p_left,
               int32_t *p_face_mm)
{
    int32_t dir;
    int32_t face_mm;
    int32_t y = 0;
    int32_t right;
    int32_t left;
    int32_t moved;
    int32_t ref;
    bool b_cap_r;
    bool b_cap_l;

    g_probe_n = 0u;
    g_face_n = 0u;
    dir = line_direction_cdeg();
    probe_turn_to(dir);
    tk_dly_tsk(PROBE_SETTLE_MS);
    face_mm = probe_ping();
    probe_log(0, face_mm);
    if (face_mm <= 0)
    {
        log_evt(EVT_OBST_WIDTH, 0u, -1, dist_mm());
        return 0;
    }
    ref = face_add(face_mm);

    right = probe_edge(dir, -1, &ref, &y, &b_cap_r) + PROBE_EDGE_OVER_MM;
    log_evt(EVT_OBST_EDGE, b_cap_r ? 2u : 0u, (int16_t)right, dist_mm());

    /* Back to the start in one move, then on to the left edge */
    if (y < 0)
    {
        probe_turn_to(dir + 9000);
        moved = probe_drive(-y, PROBE_DRIVE_MM_S);
        y += moved;
        probe_turn_to(dir);
    }
    left = probe_edge(dir, +1, &ref, &y, &b_cap_l) - PROBE_EDGE_OVER_MM;
    log_evt(EVT_OBST_EDGE, b_cap_l ? 3u : 1u, (int16_t)left, dist_mm());

    g_obst_width = (int16_t)(left - right);
    log_evt(EVT_OBST_WIDTH, (b_cap_r || b_cap_l) ? 2u : 1u, g_obst_width,
            dist_mm());
    *p_dir = dir;
    *p_y = y;
    *p_left = left;
    *p_face_mm = face_mm;
    return left - right;
}

/* Probe the obstacle, then go round it on the left and stop back on the
 * line's position behind it, facing the line's direction:
 *   out:  sideways to AVOID_HALF_R_MM + AVOID_MARGIN_MM left of its edge
 *   past: forward to the face, its assumed depth, and far enough that the
 *         car's widest side clears its back face after the turn
 *   in:   sideways back to the start's position
 * Returns false if the probe failed or a motion fault stopped it. */
static bool
avoid_obstacle(void)
{
    int32_t dir;
    int32_t y;
    int32_t left;
    int32_t face_mm;
    int32_t out;
    int32_t past;
    int32_t width;

    if (0 >= probe_obstacle(&dir, &y, &left, &face_mm))
    {
        return false;
    }
    width = g_obst_width;
    if (width < AVOID_MIN_WIDTH_MM)
    {
        left += AVOID_MIN_WIDTH_MM - width;  /* all the shortfall this side */
    }
    out = left + AVOID_HALF_R_MM + AVOID_MARGIN_MM;
    if (out > y)
    {
        probe_turn_to(dir + 9000);
        y += probe_drive(out - y, AVOID_MM_S);
    }
    probe_turn_to(dir);
    past = face_mm + AVOID_SENSOR_AHEAD_MM + AVOID_DEPTH_MM
           + AVOID_HALF_L_MM + AVOID_MARGIN_MM;
    (void)probe_drive(past, AVOID_MM_S);
    probe_turn_to(dir - 9000);
    y -= probe_drive(y, AVOID_MM_S);
    probe_turn_to(dir);
    log_evt(EVT_OBST_AVOID, 0u, (int16_t)y, dist_mm());
    return 0u == (fault_get() & FAULT_MOTION_MASK);
}

uint32_t
ctrl_probe_get(ctrl_probe_pt_t *p_out)
{
    uint32_t i;
    uint32_t n = g_probe_n;

    for (i = 0u; i < n; i++)
    {
        p_out[i] = g_probe[i];
    }
    return n;
}

static void
ctrl_task_main(INT stacd, void *exinf)
{
    UINT flgptn;
    ctrl_evt_t evt;
    imu_attitude_t att;
    imu_raw_t raw;
    hump_event_t hump;
    collision_event_t coll;
    side_hump_event_t side;
    uint32_t ring_t[CTRL_SPD_RING];
    int32_t ring_d[CTRL_SPD_RING];
    uint32_t ring_i;
    uint32_t ring_n;
    uint32_t t;
    uint32_t t_sample;
    uint32_t t_jolt = 0u;
    uint32_t imu_fail;
    uint32_t ev;
    uint32_t j;
    uint32_t k;
    int32_t roll0;
    int32_t roll;
    int32_t d;
    int32_t hdg;
    int32_t v_pct;
    int32_t min_pct = 100;
    int32_t ramp_h;
    int16_t jolt_mg = 0;
    uint8_t state;
    uint8_t flags;
    uint8_t opts;
    uint8_t stop_bits;
    uint8_t cmd;
    int32_t got;
    bool b_imu_ok;
    bool b_on_ramp;
    bool b_jolt_pending;
    int32_t junc_last;
    TMO wait;

    (void)stacd;
    (void)exinf;

    while (1)
    {
        (void)tk_wai_flg(g_ctrl_flg, CTRL_FLG_RUN, TWF_ORW | TWF_CLR, &flgptn,
                         TMO_FEVR);

        opts = g_opts;
        g_t0 = now_ms();
        g_log_n = 0u;
        while (E_OK == ctrl_evt_wait(&evt, TMO_POL))
        {
            /* stale events from the last run */
        }

        flags = 0u;
        roll = 0;
        roll0 = 0;
        g_status.last_evt = EVT_NONE;   /* not the previous run's */
        g_obst_width = 0;
        g_max_ramp = 0u;
        status_set(CTRL_ST_START, EVT_NONE, flags, 0, 0, 0);
        b_imu_ok = g_b_imu_ok && rest_attitude(&roll0);
        if (b_imu_ok)
        {
            flags |= CTRL_FLAG_IMU_OK;
        }
        fault_clear(FAULT_MOTION_MASK | FAULT_LINE_LOST | FAULT_SCAN_TIMEOUT);
        if (0u != (opts & CTRL_OPT_OBSTACLE))
        {
            obstacle_watch(true);
            flags |= CTRL_FLAG_WATCHING;
            tk_dly_tsk(CTRL_WATCH_SETTLE_MS);
        }
        while (E_OK == ctrl_evt_wait(&evt, TMO_POL))
        {
            /* events raised while starting up (a fault clears above) */
        }

        state = CTRL_ST_FOLLOW;
        cmd = CTRL_CMD_NONE;
        b_on_ramp = false;
        ramp_h = 0;
        b_jolt_pending = false;
        junc_last = -10000;
        imu_fail = 0u;
        ring_i = 0u;
        ring_n = 0u;
        (void)line_follow_start(g_speed, g_max_mm);
        if (0u != (opts & CTRL_OPT_BARCODE))
        {
            barcode_scan_enable(true);
        }
        g_t0 = now_ms();
        t_sample = g_t0;
        status_set(state, EVT_NONE, flags, 0, 0, 0);

        while ((CTRL_ST_FOLLOW == state) || (CTRL_ST_RAMP == state))
        {
            /* Events: wait until the next sample is due */
            t = now_ms();
            wait = ((t - t_sample) >= CTRL_PERIOD_MS) ? TMO_POL
                 : (TMO)(CTRL_PERIOD_MS - (t - t_sample));
            if (E_OK == ctrl_evt_wait(&evt, wait))
            {
                d = dist_mm();
                switch (evt.type)
                {
                case EVT_OBSTACLE:
                    log_evt(evt.type, evt.arg, evt.val, d);
                    if (0u != (opts & CTRL_OPT_AVOID))
                    {
                        /* Go round it and carry on along the line */
                        status_set(CTRL_ST_OBSTACLE, EVT_OBSTACLE, flags, d,
                                   heading_cdeg(), roll);
                        barcode_scan_enable(false);
                        line_wind_down();
                        obstacle_watch(false);
                        tk_dly_tsk(300u);
                        probe_turn_to(line_direction_cdeg());
                        log_evt(EVT_OBST_CREEP, 0u,
                                (int16_t)creep_to_scan_range(), dist_mm());
                        if (!avoid_obstacle())
                        {
                            state = CTRL_ST_FAULT;
                            break;
                        }
                        obstacle_watch(true);
                        tk_dly_tsk(CTRL_WATCH_SETTLE_MS);
                        requeue_events();
                        (void)line_follow_start(g_speed, AVOID_AFTER_MM);
                        if (0u != (opts & CTRL_OPT_BARCODE))
                        {
                            barcode_scan_enable(true);
                        }
                        junc_last = -10000;
                        ring_n = 0u;
                        t_sample = now_ms();
                        d = dist_mm();
                    }
                    else if (0u != (opts & CTRL_OPT_OBSTACLE))
                    {
                        line_follow_stop();
                        state = CTRL_ST_OBSTACLE;
                    }
                    break;
                case EVT_FAULT:
                    log_evt(evt.type, evt.arg, evt.val, d);
                    stop_bits = (uint8_t)(FAULT_MOTION_MASK
                                          | FAULT_SCAN_TIMEOUT);
                    if (0u != (opts & CTRL_OPT_OBSTACLE))
                    {
                        stop_bits |= FAULT_NO_ECHO;
                    }
                    if (0u != (evt.arg & stop_bits))
                    {
                        line_follow_stop();
                        state = CTRL_ST_FAULT;
                    }
                    break;
                case EVT_LINE_END:
                    log_evt(evt.type, evt.arg, evt.val, d);
                    state = (LINE_END_DISTANCE == evt.arg) ? CTRL_ST_DONE
                                                           : CTRL_ST_FAULT;
                    break;
                case EVT_STOP_REQ:
                    log_evt(evt.type, evt.arg, evt.val, d);
                    line_follow_stop();
                    state = CTRL_ST_DONE;
                    break;
                case EVT_BARCODE:
                    log_evt(evt.type, evt.arg, evt.val, d);
                    if ((0u != evt.arg)
                        && (CTRL_CMD_NONE != cmd_for(evt.arg, 0 != evt.val)))
                    {
                        cmd = cmd_for(evt.arg, 0 != evt.val);
                        flags = (uint8_t)((flags & 0x0Fu)
                                          | (cmd << CTRL_FLAG_CMD_SHIFT));
                    }
                    break;
                case EVT_JUNCTION:
                    log_evt(evt.type, evt.arg, evt.val, d);
                    if (((evt.val > junc_last) ? (evt.val - junc_last)
                                               : (junc_last - evt.val))
                        < CTRL_JUNC_SAME_MM)
                    {
                        break;      /* the other IR saw the same cross */
                    }
                    junc_last = evt.val;
                    if (CTRL_CMD_STRAIGHT == cmd)
                    {
                        /* The line task already drives straight over it */
                        log_evt(EVT_COMMAND, cmd, 0, d);
                        cmd = CTRL_CMD_NONE;
                        flags &= 0x0Fu;
                    }
                    else if (CTRL_CMD_NONE != cmd)
                    {
                        status_set(CTRL_ST_TURN, EVT_JUNCTION, flags, d,
                                   heading_cdeg(), roll);
                        barcode_scan_enable(false);
                        got = junction_turn(cmd, evt.val);
                        log_evt(EVT_COMMAND, cmd,
                                (int16_t)((got > 30000) ? 30000 : got),
                                    dist_mm());
                        cmd = CTRL_CMD_NONE;
                        flags &= 0x0Fu;
                        requeue_events();
                        if (0u != (fault_get() & FAULT_MOTION_MASK))
                        {
                            state = CTRL_ST_FAULT;
                        }
                        else if (g_after_mm <= 0)
                        {
                            state = CTRL_ST_DONE;
                        }
                        else
                        {
                            (void)line_follow_start(g_speed, g_after_mm);
                            junc_last = -10000;
                            if (0u != (opts & CTRL_OPT_BARCODE))
                            {
                                barcode_scan_enable(true);
                            }
                            ring_n = 0u;
                            t_sample = now_ms();
                        }
                        d = dist_mm();
                    }
                    break;
                default:
                    break;
                }
                status_set(state, evt.type, flags, d, heading_cdeg(), roll);
                continue;
            }

            /* Sample */
            t = now_ms();
            if ((t - t_sample) < CTRL_PERIOD_MS)
            {
                continue;
            }
            t_sample = t;
            d = dist_mm();
            hdg = heading_cdeg();

            /* Forward speed over the last CTRL_SPD_WIN_MS, % of the run
               speed */
            ring_t[ring_i] = t;
            ring_d[ring_i] = d;
            ring_i = (ring_i + 1u) % CTRL_SPD_RING;
            if (ring_n < CTRL_SPD_RING)
            {
                ring_n++;
            }
            v_pct = 100;
            if ((t - g_t0) >= CTRL_SPD_SETTLE_MS)
            {
                for (j = 1u; j < ring_n; j++)
                {
                    k = (ring_i + CTRL_SPD_RING - 1u - j) % CTRL_SPD_RING;
                    if ((t - ring_t[k]) >= CTRL_SPD_WIN_MS)
                    {
                        v_pct = ((d - ring_d[k]) * 100000L)
                                / ((int32_t)(t - ring_t[k]) * g_speed);
                        break;
                    }
                }
            }

            if (b_imu_ok)
            {
                if (E_OK == imu_read_attitude(&att, &raw))
                {
                    imu_fail = 0u;
                    roll = att.roll_cdeg - roll0;
                    ev = terrain_update(d, &att, &raw, motion_is_steady(),
                                        &hump, &coll);

                    if (0u != (ev & TERRAIN_EVT_SIDE_HUMP))
                    {
                        terrain_last_side_hump(&side);
                        ramp_h = side.height_mm;
                    }
                    if (terrain_on_hump() != b_on_ramp)
                    {
                        b_on_ramp = !b_on_ramp;
                        motion_set_hump_mask(b_on_ramp);
                        line_set_hold(b_on_ramp);
                        if (b_on_ramp)
                        {
                            flags |= CTRL_FLAG_ON_RAMP;
                            state = CTRL_ST_RAMP;
                            b_jolt_pending = false;
                            ramp_h = 0;
                            log_evt(EVT_RAMP_ON, 0u, 0, d);
                            status_set(state, EVT_RAMP_ON, flags, d, hdg, roll);
                        }
                        else
                        {
                            flags &= (uint8_t)~CTRL_FLAG_ON_RAMP;
                            state = CTRL_ST_FOLLOW;
                            log_evt(EVT_RAMP_OFF, 0u, (int16_t)ramp_h, d);
                            if ((ramp_h > (int32_t)g_max_ramp)
                                && (ramp_h < 256))
                            {
                                g_max_ramp = (uint8_t)ramp_h;
                            }
                            status_set(state, EVT_RAMP_OFF, flags, d, hdg,
                                       roll);
                        }
                    }

                    /* A jolt is an impact only at steady speed, off the
                     * ramp, and only if the wheels then slow right down */
                    if ((0u != (ev & TERRAIN_EVT_COLLISION))
                        && (!b_jolt_pending)
                        && (!b_on_ramp) && motion_is_steady())
                    {
                        b_jolt_pending = true;
                        t_jolt = t;
                        jolt_mg = coll.jolt_mg;
                        min_pct = v_pct;
                    }
                }
                else if (++imu_fail >= CTRL_IMU_FAIL_MAX)
                {
                    fault_set(FAULT_IMU);
                    b_imu_ok = false;
                    flags &= (uint8_t)~CTRL_FLAG_IMU_OK;
                    if (b_on_ramp)
                    {
                        b_on_ramp = false;
                        motion_set_hump_mask(false);
                        line_set_hold(false);
                    }
                }
            }

            if (b_jolt_pending)
            {
                if (v_pct < min_pct)
                {
                    min_pct = v_pct;
                }
                if (min_pct < CTRL_JOLT_SLOW_PCT)
                {
                    b_jolt_pending = false;
                    log_evt(EVT_IMPACT, (uint8_t)((min_pct < 0) ? 0 : min_pct),
                            jolt_mg, d);
                    if (0u != (opts & CTRL_OPT_IMPACT_STOP))
                    {
                        line_follow_stop();
                        state = CTRL_ST_IMPACT;
                    }
                    status_set(state, EVT_IMPACT, flags, d, hdg, roll);
                }
                else if ((t - t_jolt) > CTRL_JOLT_CONFIRM_MS)
                {
                    b_jolt_pending = false;
                    log_evt(EVT_JOLT_IGNORED, (uint8_t)min_pct, jolt_mg, d);
                }
            }

            status_set(state, EVT_NONE, flags, d, hdg, roll);
        }

        /* Wind down: the line task stops the wheels and ends its loop */
        barcode_scan_enable(false);
        motion_stop();
        for (j = 0u; (j < CTRL_END_WAIT_MS)
             && (LINE_END_NONE == line_follow_end());
             j += CTRL_PERIOD_MS)
        {
            tk_dly_tsk(CTRL_PERIOD_MS);
        }
        motion_set_hump_mask(false);
        line_set_hold(false);
        if (0u != (opts & CTRL_OPT_OBSTACLE))
        {
            obstacle_watch(false);
        }

        /* Stopped at an obstacle: creep in, then find both edges by
         * side-stepping (see probe_obstacle) and report its width */
        if (CTRL_ST_OBSTACLE == state)
        {
            tk_dly_tsk(300u);
            probe_turn_to(line_direction_cdeg());
            log_evt(EVT_OBST_CREEP, 0u, (int16_t)creep_to_scan_range(),
                    dist_mm());
            {
                int32_t dir;
                int32_t y;
                int32_t left;
                int32_t face_mm;

                (void)probe_obstacle(&dir, &y, &left, &face_mm);
            }
        }
        flags &= (uint8_t)~(CTRL_FLAG_ON_RAMP | CTRL_FLAG_WATCHING);
        status_set(state, EVT_NONE, flags, dist_mm(), heading_cdeg(), roll);
        g_running = 0u;
    }
}

ER
ctrl_task_create(void)
{
    T_CFLG cflg;
    T_CTSK ctsk;
    ER er;

    if (g_ctrl_tsk > 0)
    {
        return E_OBJ;
    }
    er = ctrl_evt_init();
    if (E_OK != er)
    {
        return er;
    }
    er = line_task_create();
    if ((E_OK != er) && (E_OBJ != er))
    {
        return er;
    }
    er = obstacle_task_create();
    if ((E_OK != er) && (E_OBJ != er))
    {
        return er;
    }
    er = barcode_scan_create();
    if ((E_OK != er) && (E_OBJ != er))
    {
        return er;
    }
    i2c1_init();
    g_b_imu_ok = (E_OK == imu_init());
    if (!g_b_imu_ok)
    {
        fault_set(FAULT_IMU);
    }

    cflg.flgatr = TA_TFIFO | TA_WSGL;
    cflg.iflgptn = 0u;
    g_ctrl_flg = tk_cre_flg(&cflg);
    if (g_ctrl_flg <= 0)
    {
        er = g_ctrl_flg;
        g_ctrl_flg = 0;
        return er;
    }

    ctsk.tskatr = TA_HLNG | TA_RNG3;
    ctsk.task = ctrl_task_main;
    ctsk.itskpri = CTRL_TASK_PRI;
    ctsk.stksz = CTRL_STACK_SIZE;
    g_ctrl_tsk = tk_cre_tsk(&ctsk);
    if (g_ctrl_tsk <= 0)
    {
        er = g_ctrl_tsk;
        g_ctrl_tsk = 0;
        (void)tk_del_flg(g_ctrl_flg);
        g_ctrl_flg = 0;
        return er;
    }
    er = tk_sta_tsk(g_ctrl_tsk, 0);
    if (E_OK != er)
    {
        (void)tk_del_tsk(g_ctrl_tsk);
        g_ctrl_tsk = 0;
        (void)tk_del_flg(g_ctrl_flg);
        g_ctrl_flg = 0;
    }
    return er;
}

ER
ctrl_run(int16_t speed_mm_s, int16_t max_mm, uint8_t opts)
{
    if (g_ctrl_flg <= 0)
    {
        return E_NOEXS;
    }
    if (0u != g_running)
    {
        return E_OBJ;
    }
    /* Watching for obstacles caps the speed (two pings + braking) */
    if ((0u != (opts & CTRL_OPT_OBSTACLE)) && (speed_mm_s > OBS_WATCH_MAX_MM_S))
    {
        speed_mm_s = OBS_WATCH_MAX_MM_S;
    }
    g_speed = (speed_mm_s > 0) ? speed_mm_s : 1;
    g_max_mm = max_mm;
    g_opts = opts;
    g_running = 1u;
    return tk_set_flg(g_ctrl_flg, CTRL_FLG_RUN);
}

void
ctrl_set_after_turn_mm(int16_t after_mm)
{
    g_after_mm = after_mm;
}

void
ctrl_stop(void)
{
    ctrl_evt_post(EVT_STOP_REQ, 0u, 0);
}

bool
ctrl_running(void)
{
    return 0u != g_running;
}

void
ctrl_get_status(robot_status_t *p_st)
{
    UINT sts;

    if (NULL == p_st)
    {
        return;
    }
    CRIT_ENTER(&g_status_lock, sts);
    *p_st = g_status;
    CRIT_EXIT(&g_status_lock, sts);
}

uint32_t
ctrl_log_get(ctrl_log_t *p_out)
{
    uint32_t i;
    uint32_t n = g_log_n;

    if (NULL == p_out)
    {
        return 0u;
    }
    for (i = 0u; i < n; i++)
    {
        p_out[i] = g_log[i];
    }
    return n;
}
