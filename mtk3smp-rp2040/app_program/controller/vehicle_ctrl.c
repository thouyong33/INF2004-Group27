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
#include "../common/cpu_meter.h"
#include "../buddy2_motion/motion_task.h"
#include "../buddy3_line_barcode/ir_sensor.h"
#include "../buddy3_line_barcode/line.h"
#include "../buddy3_line_barcode/barcode_scan.h"
#include "../buddy4_imu/imu.h"
#include "../buddy4_imu/terrain.h"
#include "../buddy5_ultrasonic/obstacle.h"
#include "../buddy5_ultrasonic/ultrasonic.h"
#include "../buddy1_telemetry/cmd_map.h"
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
/* Creep in before measuring: every reading has a fixed echo at ~290-315 mm
 * (the floor, the sonar points slightly down). Stopping from 100 mm/s
 * leaves the face at ~290 mm, inside it, so the face and the floor mix
 * in the readings (a 9 cm face read 151-169 mm wide). At 200 mm the face
 * is clear of it. The sonar is ~60 mm behind the nose: nose gap
 * ~14 cm; the creep limit keeps it >= 8 cm even if every ping is lost. */
#define CTRL_SCAN_RANGE_MM   200
#define CTRL_CREEP_MM_S      40
#define CTRL_CREEP_MAX_MM    150
#define CTRL_CREEP_PING_MS   65u     /* >= 60 ms between pings */
#define CTRL_CREEP_TIMEOUT_MS 6000u

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
/* A left-IR junction (EVT_JUNCTION arg 1) within this distance after the
 * line follower's curve turn, or the ramp, is ignored: on a left curve the
 * line passes under the left IR at an angle and reads as a 10-35 mm arm
 * (ghost junctions, 21 Nov: one caused a wrong right turn), and the ramp's
 * edge did the same. Crosses are on straights; the C-and-R junction (arg 0)
 * is unchanged. */
#define CTRL_JUNC_CURVE_MM   100

#define UM_PER_EDGE          328L
#define TRACK_MM             114L

static ID g_ctrl_tsk = 0;
static ID g_ctrl_flg = 0;

static volatile int16_t g_speed = 100;
static volatile int16_t g_max_mm = 450;
static volatile uint8_t g_opts = 0u;
static volatile uint8_t g_running = 0u;
/* STOP asked (ctrl_stop): seen by the sequences too, and not lost in
 * the start-up event drain */
static volatile uint8_t g_stop_req = 0u;
static volatile int16_t g_after_mm = 0;
static bool g_b_imu_ok = false;
static volatile bool g_b_run_imu = false;   /* IMU usable this run */

static robot_status_t g_status = { .obstacle_mm = -1 };  /* no ping */
static int16_t g_obst_width = 0;
static uint8_t g_max_ramp = 0u;
static crit_lock_t g_status_lock;

static ctrl_log_t g_log[CTRL_LOG_LEN];
static volatile uint32_t g_log_n = 0u;      /* events logged this run */
static int32_t g_dist_base = 0;             /* run distance before this
                                               following segment, mm */
static uint32_t g_t0 = 0u;
static volatile uint32_t g_samples = 0u;   /* sensor samples, ever */
static volatile uint32_t g_gap_max_ms = 0u;   /* longest, this run */

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

/* Encoder heading since following (re)started (odometry reset), +/-180 */
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

    p_e = &g_log[g_log_n % CTRL_LOG_LEN];   /* rolling: oldest overwritten */
    p_e->t_ms = now_ms() - g_t0;
    p_e->d_mm = g_dist_base + d;
    p_e->type = type;
    p_e->arg = arg;
    p_e->val = val;
    __asm__ volatile ("" ::: "memory");   /* entry before the count */
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
    /* a few mm below zero just after a turn read as 655 m */
    g_status.total_cm = ((g_dist_base + d) < 0) ? 0u
                        : (uint16_t)((g_dist_base + d) / 10);
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
        cpu_dly(CTRL_PERIOD_MS);
    }
    if (0 == n)
    {
        return false;
    }
    *p_roll0 = r0 / n;
    terrain_reset((int16_t)(p0 / n), (int16_t)(r0 / n));
    return true;
}

/* A sequence (junction turn, obstacle probe and avoid) ends early on a
 * motion fault (a lift included) or a STOP */
static bool
seq_abort(void)
{
    return (0u != g_stop_req)
           || (0u != (fault_get() & FAULT_MOTION_MASK));
}

/* Lift check (Buddy 4, terrain_lift_update): p_att / p_raw are this
 * period's IMU sample or NULL. On a lift: log it, raise FAULT_LIFTED (in
 * the motion mask, so the motion task, the line task and every sequence
 * below stop) and stop the wheels. True once lifted. */
static bool
lift_check(const imu_attitude_t *p_att, const imu_raw_t *p_raw,
           bool b_on_ramp)
{
    lift_event_t lift;
    uint32_t all = IR_BIT_LEFT | IR_BIT_CENTRE | IR_BIT_RIGHT;

    if (terrain_lift_update(now_ms(), p_att, p_raw,
                            (all == (ir_read() & all)), b_on_ramp, &lift))
    {
        motion_stop();
        log_evt(EVT_LIFT, lift.cue, lift.cue_val, dist_mm());
        fault_set(FAULT_LIFTED);
        return true;
    }
    return false;
}

/* Wait in the obstacle and junction sequences, which leave the main loop
 * for up to a minute: a fresh IMU sample and a lift check, then ms. A
 * lift shows there as FAULT_LIFTED, which ends the sequence. A turn on
 * the spot (motion_turn_cdeg, up to a few s) is not watched: the wait
 * after it catches the lift. */
static void
seq_wait(uint32_t ms)
{
    imu_attitude_t att;
    imu_raw_t raw;
    bool b_got;

    b_got = g_b_run_imu && (E_OK == imu_read_attitude(&att, &raw));
    (void)lift_check(b_got ? &att : NULL, b_got ? &raw : NULL, false);
    cpu_dly(ms);
}

/* The check a turn on the spot runs every 4 ms (motion_set_turn_abort),
 * in this task: the lift check every 20 ms, as seq_wait does between the
 * steps, then the sequence abort. Without it a lift during a turn was
 * only seen once the turn ended: 2.3 s instead of 1 s (19 Nov). */
static uint32_t g_turn_chk_ms = 0u;

static bool
turn_abort(void)
{
    imu_attitude_t att;
    imu_raw_t raw;
    uint32_t t = now_ms();
    bool b_got;

    if ((t - g_turn_chk_ms) >= CTRL_PERIOD_MS)
    {
        g_turn_chk_ms = t;
        b_got = g_b_run_imu && (E_OK == imu_read_attitude(&att, &raw));
        (void)lift_check(b_got ? &att : NULL, b_got ? &raw : NULL, false);
    }
    return seq_abort();
}

/* Barcode read -> command: the mapping the PC sent over MQTT
 * (buddy1_telemetry/cmd_map.h), the same whichever way the code was read;
 * no command for a character outside it, or before any mapping */
static uint8_t
cmd_for(uint8_t ch)
{
    return cmd_map_lookup(ch);
}

/* What the car is doing now (CTRL_MOTION_*, see vehicle_ctrl.h) */
#define MOTION_STILL_CM_S    1.0f    /* both wheels below: stationary */
#define MOTION_TURN_CM_S     2.0f    /* wheel difference above: turning */
static uint8_t
motion_now(uint8_t state)
{
    odometry_t odo = {0};
    float diff;
    int8_t turn;

    if (0u == g_running)
    {
        return CTRL_MOTION_STOP;
    }
    (void)motion_get_odometry(&odo);
    if ((odo.left_speed_cm_s < MOTION_STILL_CM_S)
        && (odo.left_speed_cm_s > -MOTION_STILL_CM_S)
        && (odo.right_speed_cm_s < MOTION_STILL_CM_S)
        && (odo.right_speed_cm_s > -MOTION_STILL_CM_S))
    {
        return CTRL_MOTION_STATIONARY;
    }
    turn = line_turn();
    if ((CTRL_ST_FOLLOW == state) && (LINE_TURN_OTHER != turn))
    {
        /* the curve turn, not the weave's small corrections */
        return (LINE_TURN_LEFT == turn) ? CTRL_MOTION_LEFT
             : ((LINE_TURN_RIGHT == turn) ? CTRL_MOTION_RIGHT
                                          : CTRL_MOTION_STRAIGHT);
    }
    if (CTRL_ST_RAMP == state)
    {
        return CTRL_MOTION_STRAIGHT;            /* held straight */
    }
    diff = odo.right_speed_cm_s - odo.left_speed_cm_s;
    return (diff > MOTION_TURN_CM_S) ? CTRL_MOTION_LEFT
         : ((diff < -MOTION_TURN_CM_S) ? CTRL_MOTION_RIGHT
                                       : CTRL_MOTION_STRAIGHT);
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
        seq_wait(CTRL_PERIOD_MS);
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

/* How far target_mm is ahead of the car, mm. The junction's distance
 * comes in a 16-bit event, which wraps after 32.7 m of following without
 * a restart: the difference is taken in 16 bits so it still holds. */
static int32_t
ahead16_mm(int32_t target_mm)
{
    return (int32_t)(int16_t)(uint16_t)((uint32_t)target_mm
                                        - (uint32_t)dist_mm());
}

/* Junction command: stop, drive on until the axle is over the junction,
 * turn on the spot to 90 deg (180 for a U-turn) from the LINE's direction:
 * at the junction the car is mid-weave, up to ~15 deg off the line, and
 * a plain 90 deg turn kept that error (turns ended 75-96 deg off).
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
    if (ahead16_mm(target) > 0)
    {
        (void)motion_set_velocity(CTRL_JUNC_MM_S, CTRL_JUNC_MM_S);
        while ((ahead16_mm(target) > 0) && (waited < CTRL_JUNC_FWD_MS)
               && (!seq_abort()))
        {
            seq_wait(CTRL_PERIOD_MS);
            waited += CTRL_PERIOD_MS;
        }
    }
    motion_stop();
    seq_wait(300u);
    if (seq_abort())
    {
        return 99999;
    }

    cdeg = (CTRL_CMD_LEFT == cmd) ? 9000
           : ((CTRL_CMD_RIGHT == cmd) ? -9000 : 18000);
    cdeg += dir - heading_raw_cdeg();
    got = motion_turn_cdeg(cdeg, CTRL_TURN_MM_S);
    if ((!seq_abort())
        && (((cdeg - got) > CTRL_TRIM_CDEG)
            || ((got - cdeg) > CTRL_TRIM_CDEG)))
    {
        got += motion_turn_cdeg(cdeg - got, CTRL_TRIM_MM_S);
    }
    seq_wait(200u);
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
            || seq_abort())
        {
            break;
        }
        seq_wait(CTRL_CREEP_PING_MS);
    }
    motion_stop();
    seq_wait(300u);
    return mm;
}

/*----------------------------------------------------------------------------
 * Obstacle width by side-stepping. Turning on the spot cannot measure
 * width: a flat face echoes only when the sonar looks at it square-on, the
 * sonar is ~70 mm ahead of the axle and barely moves sideways as the car
 * turns, so every face over ~7 cm was seen over the same ~50 deg (9 / 15 /
 * 23 cm faces: 47-52 / 50-56 / 49-57 deg). Instead the car
 * faces the line's direction and steps sideways (turn 90, drive, turn
 * back, ping) until the face's echo is lost: that is an edge, at the
 * sonar's sideways position. Width = left edge - right edge, measured on
 * the wheels. The range stays ~190 mm, clear of the ~290 mm floor echo.
 *---------------------------------------------------------------------------*/
#define PROBE_STEP_MM        25      /* sideways step: edge to +/-12 mm */
#define PROBE_STEPS_MAX      24u     /* per side: 2 x PROBE_MAX_MM / step */
#define PROBE_MAX_MM         250     /* per side: faces up to ~50 cm */
/* A reading is the face if it is no further than this beyond the median
 * of the face readings so far (a nearer one always is): the face's range
 * drifts 25-40 mm along it, and comparing with the first reading ended
 * probes early on one stray reading */
#define PROBE_BAND_MM        25
/* ...and a reading up to this beyond it is still the obstacle, a rounded
 * corner curving away (9.5 cm box: 188-194 mm face, 220 mm at the corner,
 * was taken as the edge and the width read 44 mm); it holds the
 * edge further out but is not a face reading */
#define PROBE_SHOULDER_MM    60
#define PROBE_OFF_NEEDED     2       /* off-face readings in a row = edge */
#define PROBE_DRIVE_MM_S     60
#define PROBE_BRAKE_MM       3       /* stop command this early */
/* 120 (was 80, 24 Nov): each side step is two 90 deg turns, ~1.1 s each
 * at 80; the turn is trimmed to CTRL_TRIM_CDEG and logged (OBST_HDG) */
#define PROBE_TURN_MM_S      120
#define PROBE_PINGS          3       /* median of */
#define PROBE_PING_MS        65u
#define PROBE_SETTLE_MS      150u
/* The echo runs on past each edge (the beam still clips the face near the
 * corner): 9 runs on 9 / 15 / 23 cm faces read 17 mm wide per edge on
 * average; with it taken off, 8 of 9 widths
 * were within +/-2 cm, the worst 3.2 cm under */
#define PROBE_EDGE_OVER_MM   17

/* Drive past on the left (the car's right side faces the obstacle) */
#define AVOID_HALF_R_MM      70      /* centre to the right wheel's outside */
/* centre to the left IR bracket: wheel outside (70) + 20 mm */
#define AVOID_HALF_L_MM      90
/* Turning on the spot, the left IR tip (140 mm ahead of the axle, 65 mm
 * left) sweeps a circle of this radius: the car must be this far from the
 * box's side, plus AVOID_SIDE_MARGIN_MM, to turn beside it (the depth
 * check) */
#define AVOID_SWING_MM       155
/* ...and this beyond it (80, was 40, 24 Nov): a width read a few cm
 * narrow put the car close enough that the turn toward the line touched
 * the box */
#define AVOID_SIDE_MARGIN_MM 80
#define AVOID_SENSOR_AHEAD_MM 70     /* sonar ahead of the axle */
#define AVOID_MARGIN_MM      40      /* beyond the width estimate's error */
/* plan for at least this width (course obstacles are 10-20 cm): a low
 * estimate left too little room and the car brushed the box) */
#define AVOID_MIN_WIDTH_MM   100
/* Depth is not measurable from the front: taken as the planned width (the
 * obstacle square), at most this. 200 mm for every obstacle drove ~100 mm
 * further past a 10 cm box than needed and could rejoin on a curve; a
 * long narrow obstacle is the risk this cap keeps in check */
#define AVOID_DEPTH_MAX_MM   200
/* Back to the line: stop over it when C or R sees it, not at the planned
 * sideways distance, so a line that curved away is met where it is. C
 * meets the line's far edge AVOID_IR_AHEAD_MM + AVOID_LINE_W_MM before
 * the axle is back at the start's position (C on the right edge) */
#define AVOID_IR_AHEAD_MM    139     /* = CTRL_JUNC_AHEAD_MM */
#define AVOID_LINE_W_MM      18
/* ...and this much further than planned once the line has been seen
 * (80, 24 Nov: the turns' heading drift past a long box put the line 57 mm
 * further than planned; 120 let a car squared up crooked after a curve
 * reach the next piece of track) */
#define AVOID_SEEK_EXTRA_MM  80
#define AVOID_MM_S           100
/* Depth check (24 Nov). The demo box may be much deeper than its face is
 * wide, so the depth taken as the width is only a first guess. Facing the
 * line before the way back, the sonar looks along the car's path in: an
 * echo nearer than the line (at most AVOID_DEPTH_NEAR_MAX_MM: the floor
 * echoes at ~290 mm) is the box, so the car goes AVOID_DEPTH_STEP_MM
 * further and looks again. Once clear, AVOID_DEPTH_CLEAR_MM more (see
 * there). Past
 * AVOID_DEPTH_MAX_EXTRA_MM still blocked: stop (fault), not guess. */
#define AVOID_DEPTH_NEAR_MAX_MM 260
#define AVOID_DEPTH_STEP_MM  60
/* 70 (24 Nov), after every depth check: with AVOID_DEPTH_OFF_NEEDED clear
 * readings the car is already past the back face; 110 and 180 after a single clear reading
 * left the rear clipping the box (that reading came a step early), 180
 * after two left ~1.5 car lengths spare, and 50 cleared it but close */
#define AVOID_DEPTH_CLEAR_MM 70
#define AVOID_DEPTH_MAX_EXTRA_MM 600
#define AVOID_DEPTH_OFF_NEEDED 2u      /* clear readings in a row, once seen */
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

    if (seq_abort())
    {
        return;
    }
    cdeg = target_cdeg - heading_raw_cdeg();
    got = motion_turn_cdeg(cdeg, PROBE_TURN_MM_S);
    if (((cdeg - got) > CTRL_TRIM_CDEG) || ((got - cdeg) > CTRL_TRIM_CDEG))
    {
        (void)motion_turn_cdeg(cdeg - got, CTRL_TRIM_MM_S);
    }
}

/* probe_turn_to, then log how far the encoder heading ended from the
 * target (EVT_OBST_HDG, arg = OBST_HDG_TURN + k) */
static void
avoid_turn_to(int32_t target_cdeg, uint8_t k)
{
    probe_turn_to(target_cdeg);
    log_evt(EVT_OBST_HDG, (uint8_t)(OBST_HDG_TURN + k),
            (int16_t)(heading_raw_cdeg() - target_cdeg), dist_mm());
}

/* Drive straight forward mm at mm_s and stop; returns the distance */
static int32_t
probe_drive(int32_t mm, int16_t mm_s)
{
    int32_t d0;
    uint32_t waited = 0u;
    uint32_t limit_ms;

    if (seq_abort())
    {
        return 0;
    }
    limit_ms = (uint32_t)((mm * 1000) / mm_s) + CTRL_JUNC_FWD_MS;
    d0 = dist_mm();
    (void)motion_set_velocity(mm_s, mm_s);
    while (((dist_mm() - d0) < (mm - PROBE_BRAKE_MM))
           && (waited < limit_ms)
           && (!seq_abort()))
    {
        seq_wait(CTRL_PERIOD_MS);
        waited += CTRL_PERIOD_MS;
    }
    motion_stop();
    seq_wait(PROBE_SETTLE_MS);
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
        seq_wait(PROBE_PING_MS);
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

/*----------------------------------------------------------------------------
 * The avoid squares up to the line's direction and measures every move
 * from it. Just after a curve or the ramp there is no plain straight to
 * take it from, and the fallback (the curve's running average) was 13 deg
 * short of where the line went: the car crept in at an angle, slid past
 * the face and found nothing to measure (21 Nov). So then the car finds
 * the face itself: it turns FACE_STEP_CDEG at a time each way from where it
 * stopped until the echo is lost, and faces the middle of that span. A flat
 * face echoes only near square-on, so the middle is square to the face.
 *---------------------------------------------------------------------------*/
#define FACE_STEP_CDEG       500
#define FACE_MAX_CDEG        4000    /* each way */
/* still the face if no further than this beyond the first reading (the
 * range along the beam grows as the car turns off square) */
#define FACE_BAND_MM         60

static int32_t g_obst_dir = 0;       /* the avoid's reference, see above */
static bool g_obst_face = false;     /* g_obst_dir came from the face */

/* Turn from h0 toward side (+1 left, -1 right) a step at a time. If h0
 * already sees the face (b_seen) it stops at the first step off it;
 * otherwise it carries on until it finds the face, then stops at the first
 * step off it. *p_first / *p_last: the first and last headings that saw it.
 * Returns false if it never did. */
static bool
face_edge(int32_t h0, int32_t side, int32_t r_max, bool b_seen,
          int32_t *p_first, int32_t *p_last)
{
    int32_t k;
    int32_t mm;
    bool b_found = false;

    *p_first = h0;
    *p_last = h0;
    for (k = FACE_STEP_CDEG; k <= FACE_MAX_CDEG; k += FACE_STEP_CDEG)
    {
        if (seq_abort())
        {
            break;
        }
        probe_turn_to(h0 + (side * k));
        seq_wait(PROBE_SETTLE_MS);
        mm = probe_ping();
        if ((mm > 0) && (mm <= r_max))
        {
            if (!b_found)
            {
                *p_first = h0 + (side * k);
            }
            b_found = true;
            *p_last = h0 + (side * k);
        }
        else if (b_seen || b_found)
        {
            break;                      /* off the face again */
        }
        else
        {
            /* not found yet: keep looking */
        }
    }
    return b_found;
}

/* Find the face's echo span by turning on the spot and set g_obst_dir to
 * its middle. r_hint: the range last read (used when the heading the car
 * is on does not echo now). Returns false if no face was found. A failed
 * first ping after the creep (median of 3 at 197 mm, the face 7 deg off
 * square) ended the avoid before (21 Nov), so the edges are searched
 * outward from a heading that does not see the face as well. */
static bool
face_sweep(int32_t r_hint)
{
    int32_t h0;
    int32_t r0;
    int32_t r_max;
    int32_t lf;
    int32_t ll;
    int32_t rf;
    int32_t rl;
    bool b0;
    bool bl;
    bool br;
    int32_t left;
    int32_t right;

    if (seq_abort())
    {
        return false;
    }
    h0 = heading_raw_cdeg();
    r0 = probe_ping();
    b0 = (r0 > 0);
    r_max = (b0 ? r0 : r_hint) + FACE_BAND_MM;
    if ((!b0) && (r_hint <= 0))
    {
        return false;
    }
    bl = face_edge(h0, 1, r_max, b0, &lf, &ll);
    br = face_edge(h0, -1, r_max, b0, &rf, &rl);
    if (seq_abort())
    {
        return false;
    }
    if (b0)
    {
        left = ll;                      /* h0 if the left step missed */
        right = rl;
    }
    else if (bl && br)
    {
        left = ll;
        right = rl;
    }
    else if (bl)
    {
        left = ll;
        right = lf;
    }
    else if (br)
    {
        left = rf;
        right = rl;
    }
    else
    {
        probe_turn_to(h0);
        return false;
    }
    g_obst_dir = (left + right) / 2;
    return true;
}

/* The avoid's reference direction (sets g_obst_dir, g_obst_face): plain
 * following if there was enough of it, else the face found by turning,
 * else the fallback */
static int32_t
obstacle_dir(void)
{
    g_obst_face = false;
    g_obst_dir = line_clean_direction_cdeg();
    if (line_clean_ok())
    {
        return g_obst_dir;
    }
    g_obst_face = face_sweep(-1);
    return g_obst_dir;
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

/* Step sideways from *p_y (left +) by side (+1 / -1), facing dir_cdeg,
 * until the face's echo
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
    uint32_t steps = 0u;

    *p_capped = false;
    for (;;)
    {
        /* PROBE_STEPS_MAX: a car that cannot move (and does not fault)
         * never reaches PROBE_MAX_MM */
        if (((side > 0) && (*p_y >= PROBE_MAX_MM))
            || ((side < 0) && (*p_y <= -PROBE_MAX_MM))
            || (++steps > PROBE_STEPS_MAX)
            || seq_abort())
        {
            *p_capped = true;
            return y_on;
        }
        probe_turn_to(dir_cdeg + (side * 9000));
        moved = probe_drive(PROBE_STEP_MM, PROBE_DRIVE_MM_S);
        *p_y += side * moved;
        probe_turn_to(dir_cdeg);
        seq_wait(PROBE_SETTLE_MS);
        mm = probe_ping();
        probe_log(*p_y, mm);
        if ((mm > 0) && (mm <= (*p_ref + PROBE_BAND_MM)))
        {
            y_on = *p_y;
            offs = 0u;
            *p_ref = face_add(mm);
        }
        else if ((mm > 0) && (mm <= (*p_ref + PROBE_SHOULDER_MM)))
        {
            y_on = *p_y;               /* a rounded corner: still on it */
            offs = 0u;
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
    if (g_obst_face)
    {
        /* again from the scan range (the creep's reading): the first
           sweep was at ~300 mm, where the span is less sure */
        (void)face_sweep(CTRL_SCAN_RANGE_MM);
    }
    dir = g_obst_dir;                   /* set by obstacle_dir() */
    /* the reference: plain following, the face or the fallback, and how
       far it is from the plain 100 mm average */
    log_evt(EVT_OBST_HDG,
            g_obst_face ? OBST_HDG_FACE
                        : (line_clean_ok() ? OBST_HDG_CLEAN : OBST_HDG_PLAIN),
            (int16_t)(dir - line_direction_cdeg()), dist_mm());
    avoid_turn_to(dir, 0u);
    seq_wait(PROBE_SETTLE_MS);
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

    if (seq_abort())
    {
        /* stopped or lifted part-way: no width (an aborted probe read
         * -55 mm and published it, 19 Nov) */
        log_evt(EVT_OBST_WIDTH, 0u, -1, dist_mm());
        return 0;
    }
    /* An object narrower than a step loses the echo at once on both
     * sides, and the edge correction makes left - right negative (-34
     * mm): report 0 and plan the avoid for AVOID_MIN_WIDTH_MM */
    g_obst_width = (int16_t)(((left - right) > 0) ? (left - right) : 0);
    log_evt(EVT_OBST_WIDTH, (b_cap_r || b_cap_l) ? 2u : 1u, g_obst_width,
            dist_mm());
    *p_dir = dir;
    *p_y = y;
    *p_left = left;
    *p_face_mm = face_mm;
    return ((left - right) > 0) ? (left - right) : 1;
}

/* The sideways leg back to the line (facing it), planned as y mm: stops
 * over the line if C or R sees it (see AVOID_IR_AHEAD_MM), else after y.
 * Logs EVT_OBST_AVOID arg 1, val = where the line was against the plan
 * (mm, + = further), or arg 2 if it was not seen. Returns the distance. */
static int32_t
avoid_return(int32_t y)
{
    int32_t d0;
    int32_t moved = 0;
    int32_t target = y;
    int32_t seen = -1;
    uint32_t waited = 0u;
    uint32_t limit_ms;

    limit_ms = (uint32_t)(((y + AVOID_SEEK_EXTRA_MM) * 1000) / AVOID_MM_S)
               + CTRL_JUNC_FWD_MS;
    d0 = dist_mm();
    (void)motion_set_velocity(AVOID_MM_S, AVOID_MM_S);
    while ((moved < (target - PROBE_BRAKE_MM)) && (waited < limit_ms)
           && (!seq_abort()))
    {
        if ((seen < 0)
            && (0u != (ir_read() & (IR_BIT_CENTRE | IR_BIT_RIGHT))))
        {
            seen = moved;
            target = seen + AVOID_IR_AHEAD_MM + AVOID_LINE_W_MM;
            target = (target > (y + AVOID_SEEK_EXTRA_MM))
                     ? (y + AVOID_SEEK_EXTRA_MM) : target;
        }
        seq_wait(CTRL_PERIOD_MS);
        waited += CTRL_PERIOD_MS;
        moved = dist_mm() - d0;
    }
    motion_stop();
    seq_wait(PROBE_SETTLE_MS);
    moved = dist_mm() - d0;
    log_evt(EVT_OBST_AVOID, (seen >= 0) ? 1u : 2u,
            (int16_t)((seen >= 0)
                      ? ((seen + AVOID_IR_AHEAD_MM + AVOID_LINE_W_MM) - y)
                      : 0), dist_mm());
    return moved;
}

/* Facing the line beside or past the box (see AVOID_DEPTH_STEP_MM): step
 * along the line's direction dir until the sonar no longer sees the box
 * between the car and the line (y mm away), then clear it by
 * AVOID_DEPTH_CLEAR_MM. Ends facing the line. Logs EVT_OBST_AVOID arg 3,
 * val = mm driven further than planned (only if the box was longer), or
 * arg 4 if it was still there after AVOID_DEPTH_MAX_EXTRA_MM. Returns
 * false then, or on STOP or a fault. */
static bool
avoid_depth_check(int32_t dir, int32_t y)
{
    int32_t mm;
    int32_t extra = 0;
    int32_t near_mm;
    uint32_t clear = 0u;
    bool b_seen = false;

    /* anything nearer than the line is the box (not worked out from the box's
     * left edge: a width over-read by 50 mm put the box just past that
     * limit and the car turned in, 24 Nov); the floor echo is ~290 mm */
    near_mm = y - AVOID_SENSOR_AHEAD_MM;
    near_mm = (near_mm > AVOID_DEPTH_NEAR_MAX_MM) ? AVOID_DEPTH_NEAR_MAX_MM
                                                  : near_mm;
    for (;;)
    {
        if (seq_abort())
        {
            return false;
        }
        mm = probe_ping();
        if ((mm <= 0) || (mm > near_mm))
        {
            /* the box is not beside the car: believe that only on
             * AVOID_DEPTH_OFF_NEEDED readings in a row, even the first time
             * (the past leg ends at the box's assumed back face, where the
             * echo can drop out a step before the corner: the rear clipped
             * the box, 24 Nov) */
            clear++;
            if (clear >= AVOID_DEPTH_OFF_NEEDED)
            {
                break;
            }
        }
        else
        {
            clear = 0u;
            b_seen = true;
        }
        if (extra >= AVOID_DEPTH_MAX_EXTRA_MM)
        {
            log_evt(EVT_OBST_AVOID, 4u, (int16_t)extra, dist_mm());
            return false;
        }
        probe_turn_to(dir);
        extra += probe_drive(AVOID_DEPTH_STEP_MM, AVOID_MM_S);
        probe_turn_to(dir - 9000);
    }
    /* every time: the past leg leaves no margin of its own */
    probe_turn_to(dir);
    extra += probe_drive(AVOID_DEPTH_CLEAR_MM, AVOID_MM_S);
    probe_turn_to(dir - 9000);
    if (b_seen)
    {
        log_evt(EVT_OBST_AVOID, 3u, (int16_t)extra, dist_mm());
    }
    return !seq_abort();
}

/* Probe the obstacle, then go round it on the left and stop back on the
 * line behind it, facing the line's direction:
 *   out:  sideways to AVOID_SWING_MM + AVOID_SIDE_MARGIN_MM left of its edge
 *   past: forward to the face and a depth taken as its width (100-200 mm)
 *   check: facing the line, further while the box is still seen
 *         (avoid_depth_check)
 *   in:   sideways back until the line is seen (avoid_return)
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

    if ((0 >= probe_obstacle(&dir, &y, &left, &face_mm)) || seq_abort())
    {
        return false;
    }
    width = g_obst_width;
    if (width < AVOID_MIN_WIDTH_MM)
    {
        left += AVOID_MIN_WIDTH_MM - width;  /* all the shortfall this side */
        width = AVOID_MIN_WIDTH_MM;
    }
    /* room to turn on the spot beside the box (the depth check) */
    out = left + AVOID_SWING_MM + AVOID_SIDE_MARGIN_MM;
    if (out > y)
    {
        avoid_turn_to(dir + 9000, 1u);
        y += probe_drive(out - y, AVOID_MM_S);
    }
    avoid_turn_to(dir, 2u);
    /* to the box's assumed back face only (24 Nov): with a margin added,
     * a box up to ~130 mm deeper than its width was never seen by the
     * depth check, which now adds the clearance itself */
    past = face_mm + AVOID_SENSOR_AHEAD_MM
           + ((width < AVOID_DEPTH_MAX_MM) ? width : AVOID_DEPTH_MAX_MM);
    (void)probe_drive(past, AVOID_MM_S);
    avoid_turn_to(dir - 9000, 3u);
    if (!avoid_depth_check(dir, y))
    {
        return false;
    }
    y -= avoid_return(y);
    avoid_turn_to(dir, 4u);
    log_evt(EVT_OBST_AVOID, 0u, (int16_t)y, dist_mm());
    return !seq_abort();
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
    ER er;
    imu_attitude_t att;
    imu_raw_t raw;
    hump_event_t hump;
    side_hump_event_t side;
    uint32_t t;
    uint32_t t_sample;
    uint32_t imu_fail;
    uint32_t ev;
    uint32_t j;
    int32_t roll0;
    int32_t roll;
    int32_t d;
    int32_t hdg;
    int32_t ramp_h;
    uint8_t state;
    uint8_t flags;
    uint8_t opts;
    uint8_t stop_bits;
    uint8_t cmd;
    int32_t got;
    bool b_imu_ok;
    bool b_on_ramp;
    bool b_got;
    int32_t junc_last;
    int32_t curve_d;          /* last distance in a curve turn */
    uint32_t t_obst;
    TMO wait;

    (void)stacd;
    (void)exinf;
    cpu_meter_register(CPU_CTRL);

    while (1)
    {
        cpu_idle();
        (void)tk_wai_flg(g_ctrl_flg, CTRL_FLG_RUN, TWF_ORW | TWF_CLR, &flgptn,
                         TMO_FEVR);
        cpu_busy();

        opts = g_opts;
        g_t0 = now_ms();
        g_log_n = 0u;
        g_gap_max_ms = 0u;
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
        g_probe_n = 0u;            /* no previous run's probe in this log */
        /* a run that stops before it moves must not report the last
         * run's distance (line_follow_start resets it too) */
        motion_reset_odometry();
        line_trace_clear();
        status_set(CTRL_ST_START, EVT_NONE, flags, 0, 0, 0);
        if (!g_b_imu_ok)
        {
            /* one glitch at boot must not leave the ramp height and the
             * lift stop off until a power cycle: try again each run */
            i2c1_init();
            g_b_imu_ok = (E_OK == imu_init());
        }
        b_imu_ok = g_b_imu_ok && rest_attitude(&roll0);
        if (b_imu_ok)
        {
            flags |= CTRL_FLAG_IMU_OK;
            fault_clear(FAULT_IMU);
        }
        g_b_run_imu = b_imu_ok;
        fault_clear(FAULT_MOTION_MASK | FAULT_LINE_LOST);
        if (0u != (opts & CTRL_OPT_OBSTACLE))
        {
            obstacle_watch(true);
            flags |= CTRL_FLAG_WATCHING;
            cpu_dly(CTRL_WATCH_SETTLE_MS);
        }
        while (E_OK == ctrl_evt_wait(&evt, TMO_POL))
        {
            /* events raised while starting up (a fault clears above) */
        }

        state = CTRL_ST_FOLLOW;
        cmd = CTRL_CMD_NONE;
        b_on_ramp = false;
        ramp_h = 0;
        junc_last = -10000;
        curve_d = -10000;
        t_obst = 0u;
        g_dist_base = 0;
        imu_fail = 0u;
        if (0u != g_stop_req)
        {
            log_evt(EVT_STOP_REQ, 0u, 0, 0);
            state = CTRL_ST_DONE;       /* stopped before it moved */
        }
        else if ((0u != (opts & CTRL_OPT_OBSTACLE))
                 && (0u != (fault_get() & FAULT_NO_ECHO)))
        {
            /* A dead sonar is found within the watch settle (5 pings), so
             * its EVT_FAULT went in the drain above and was lost: the
             * car drove unprotected (19 Nov). Do not start. */
            log_evt(EVT_FAULT, FAULT_NO_ECHO, 0, 0);
            state = CTRL_ST_FAULT;
        }
        else
        {
            (void)line_follow_start(g_speed, g_max_mm);
            if (0u != (opts & CTRL_OPT_BARCODE))
            {
                barcode_scan_enable(true);
            }
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
            cpu_idle();
            er = ctrl_evt_wait(&evt, wait);
            cpu_busy();
            if (E_OK == er)
            {
                d = dist_mm();
                switch (evt.type)
                {
                case EVT_OBSTACLE:
                    log_evt(evt.type, evt.arg, evt.val, d);
                    if (0u != (opts & CTRL_OPT_AVOID))
                    {
                        /* Go round it and carry on along the line */
                        t_obst = now_ms();
                        if (b_on_ramp)
                        {
                            /* not across a sequence: the hump mask would
                             * end it with a stall fault after 4 s */
                            b_on_ramp = false;
                            motion_set_hump_mask(false);
                            line_set_hold(false);
                            flags &= (uint8_t)~CTRL_FLAG_ON_RAMP;
                        }
                        status_set(CTRL_ST_OBSTACLE, EVT_OBSTACLE, flags, d,
                                   heading_cdeg(), roll);
                        barcode_scan_enable(false);
                        line_wind_down();
                        obstacle_watch(false);
                        seq_wait(300u);
                        probe_turn_to(obstacle_dir());
                        log_evt(EVT_OBST_CREEP, 0u,
                                (int16_t)creep_to_scan_range(), dist_mm());
                        if (!avoid_obstacle())
                        {
                            log_evt(EVT_OBST_DONE, 0u,
                                    (int16_t)((now_ms() - t_obst) / 100u),
                                    dist_mm());
                            if (0u != g_stop_req)
                            {
                                log_evt(EVT_STOP_REQ, 0u, 0, dist_mm());
                                state = CTRL_ST_DONE;
                            }
                            else
                            {
                                state = CTRL_ST_FAULT;
                            }
                            break;
                        }
                        log_evt(EVT_OBST_DONE, 1u,
                                (int16_t)((now_ms() - t_obst) / 100u),
                                dist_mm());
                        obstacle_watch(true);
                        seq_wait(CTRL_WATCH_SETTLE_MS);
                        requeue_events();
                        g_dist_base += dist_mm();
                        (void)line_follow_start(g_speed,
                                                (g_max_mm > 0)
                                                ? AVOID_AFTER_MM : 0);
                        if (0u != (opts & CTRL_OPT_BARCODE))
                        {
                            barcode_scan_enable(true);
                        }
                        junc_last = -10000;
                        curve_d = -10000;
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
                    stop_bits = (uint8_t)FAULT_MOTION_MASK;
                    if (0u != (opts & CTRL_OPT_OBSTACLE))
                    {
                        stop_bits |= FAULT_NO_ECHO;
                    }
                    if (0u != (evt.arg & stop_bits))
                    {
                        line_follow_stop();
                        state = (0u != (evt.arg & FAULT_LIFTED))
                                ? CTRL_ST_LIFTED : CTRL_ST_FAULT;
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
                    if ((0u != evt.arg) && (0u == (opts & CTRL_OPT_NO_CMD))
                        && (CTRL_CMD_NONE
                            != cmd_for(evt.arg)))
                    {
                        cmd = cmd_for(evt.arg);
                        flags = (uint8_t)((flags & 0x0Fu)
                                          | (cmd << CTRL_FLAG_CMD_SHIFT));
                    }
                    break;
                case EVT_JUNCTION:
                    if ((1u == evt.arg) && ((d - curve_d) < CTRL_JUNC_CURVE_MM))
                    {
                        /* logged with arg 3: a left-IR "arm" on a curve */
                        log_evt(evt.type, 3u, evt.val, d);
                        break;
                    }
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
                        if (b_on_ramp)
                        {
                            /* not across a sequence: the hump mask would
                             * end it with a stall fault after 4 s */
                            b_on_ramp = false;
                            motion_set_hump_mask(false);
                            line_set_hold(false);
                            flags &= (uint8_t)~CTRL_FLAG_ON_RAMP;
                        }
                        status_set(CTRL_ST_TURN, EVT_JUNCTION, flags, d,
                                   heading_cdeg(), roll);
                        barcode_scan_enable(false);
                        if (0u != (opts & CTRL_OPT_OBSTACLE))
                        {
                            /* the sonar sweeps the side as the car turns:
                             * no obstacle stop from that */
                            obstacle_watch(false);
                        }
                        got = junction_turn(cmd, evt.val);
                        log_evt(EVT_COMMAND, cmd,
                                (int16_t)((got > 30000) ? 30000 : got),
                                    dist_mm());
                        cmd = CTRL_CMD_NONE;
                        flags &= 0x0Fu;
                        requeue_events();
                        if (0u != g_stop_req)
                        {
                            log_evt(EVT_STOP_REQ, 0u, 0, dist_mm());
                            state = CTRL_ST_DONE;
                        }
                        else if (0u != (fault_get() & FAULT_MOTION_MASK))
                        {
                            state = CTRL_ST_FAULT;
                        }
                        else if ((g_max_mm > 0) && (g_after_mm <= 0))
                        {
                            state = CTRL_ST_DONE;
                        }
                        else
                        {
                            if (0u != (opts & CTRL_OPT_OBSTACLE))
                            {
                                obstacle_watch(true);
                                seq_wait(CTRL_WATCH_SETTLE_MS);
                            }
                            g_dist_base += dist_mm();
                            /* the turn leaves the line under R: acquire
                             * it before any curve turn (line.h) */
                            line_acquire_next();
                            (void)line_follow_start(g_speed,
                                                    (g_max_mm > 0) ? g_after_mm
                                                                   : 0);
                            junc_last = -10000;
                            curve_d = -10000;
                            if (0u != (opts & CTRL_OPT_BARCODE))
                            {
                                barcode_scan_enable(true);
                            }
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
            if ((t - t_sample) > g_gap_max_ms)
            {
                g_gap_max_ms = t - t_sample;
            }
            g_samples++;
            t_sample = t;
            d = dist_mm();
            hdg = heading_cdeg();
            if ((LINE_TURN_LEFT == line_turn())
                || (LINE_TURN_RIGHT == line_turn()) || b_on_ramp)
            {
                curve_d = d;        /* see CTRL_JUNC_CURVE_MM */
            }

            if (b_imu_ok)
            {
                if (E_OK == imu_read_attitude(&att, &raw))
                {
                    imu_fail = 0u;
                    roll = att.roll_cdeg - roll0;
                    ev = terrain_update(g_dist_base + d, &att,
                                        motion_is_steady(), &hump);

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
                }
                else if (++imu_fail >= CTRL_IMU_FAIL_MAX)
                {
                    fault_set(FAULT_IMU);
                    b_imu_ok = false;
                    g_b_run_imu = false;
                    flags &= (uint8_t)~CTRL_FLAG_IMU_OK;
                    if (b_on_ramp)
                    {
                        b_on_ramp = false;
                        motion_set_hump_mask(false);
                        line_set_hold(false);
                    }
                }
            }

            if ((0u != g_stop_req) && ((CTRL_ST_FOLLOW == state)
                                       || (CTRL_ST_RAMP == state)))
            {
                log_evt(EVT_STOP_REQ, 0u, 0, d);
                line_follow_stop();
                state = CTRL_ST_DONE;
            }

            /* Picked up (the end of the demo): stop now */
            b_got = b_imu_ok && (0u == imu_fail);
            if (lift_check(b_got ? &att : NULL, b_got ? &raw : NULL,
                           b_on_ramp))
            {
                line_follow_stop();
                state = CTRL_ST_LIFTED;
            }

            status_set(state, EVT_NONE, flags, d, hdg, roll);
        }

        if (0u != (fault_get() & FAULT_LIFTED))
        {
            state = CTRL_ST_LIFTED;     /* also when a sequence saw it */
        }

        /* Wind down: the line task stops the wheels and ends its loop */
        barcode_scan_enable(false);
        motion_stop();
        for (j = 0u; (j < CTRL_END_WAIT_MS)
             && (LINE_END_NONE == line_follow_end());
             j += CTRL_PERIOD_MS)
        {
            cpu_dly(CTRL_PERIOD_MS);
        }
        motion_set_hump_mask(false);
        line_set_hold(false);
        if (0u != (opts & CTRL_OPT_OBSTACLE))
        {
            obstacle_watch(false);
        }

        /* Stopped at an obstacle: creep in, then find both edges by
         * side-stepping (see probe_obstacle) and report its width */
        if ((CTRL_ST_OBSTACLE == state) && (0u == g_stop_req))
        {
            seq_wait(300u);
            probe_turn_to(obstacle_dir());
            log_evt(EVT_OBST_CREEP, 0u, (int16_t)creep_to_scan_range(),
                    dist_mm());
            {
                int32_t dir;
                int32_t y;
                int32_t left;
                int32_t face_mm;

                (void)probe_obstacle(&dir, &y, &left, &face_mm);
            }
            if (0u != (fault_get() & FAULT_LIFTED))
            {
                state = CTRL_ST_LIFTED;     /* picked up while probing */
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
    /* only the controller turns on the spot in a course run */
    motion_set_turn_abort(turn_abort);

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
    g_stop_req = 0u;
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
    if (0u != g_running)
    {
        g_stop_req = 1u;
        motion_stop();          /* also ends a turn on the spot */
    }
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
    p_st->motion = motion_now(p_st->state);
    /* live, not the copy: the copy is only refreshed in the follow loop
     * (a Wi-Fi-loss fault stayed after the link was back) */
    p_st->faults = fault_get();
    p_st->obstacle_mm = obstacle_last_mm();
}

uint32_t
ctrl_log_get(ctrl_log_t *p_out)
{
    uint32_t i;
    uint32_t n = g_log_n;
    uint32_t first;

    if (NULL == p_out)
    {
        return 0u;
    }
    first = (n > CTRL_LOG_LEN) ? (n - CTRL_LOG_LEN) : 0u;
    for (i = first; i < n; i++)
    {
        p_out[i - first] = g_log[i % CTRL_LOG_LEN];
    }
    return n - first;
}

uint32_t
ctrl_sample_count(void)
{
    return g_samples;
}

uint32_t
ctrl_sample_gap_max_ms(void)
{
    return g_gap_max_ms;
}

uint32_t
ctrl_log_count(void)
{
    return g_log_n;
}

bool
ctrl_log_at(uint32_t idx, ctrl_log_t *p_out)
{
    bool b_ok = false;

    if ((NULL != p_out) && (idx < g_log_n))
    {
        *p_out = g_log[idx % CTRL_LOG_LEN];
        __asm__ volatile ("" ::: "memory");
        /* still this entry if the writer has not wrapped onto it, and is
         * not writing it now (n - idx == CTRL_LOG_LEN is the slot the next
         * event overwrites) */
        b_ok = ((g_log_n - idx) < CTRL_LOG_LEN);
    }
    return b_ok;
}
