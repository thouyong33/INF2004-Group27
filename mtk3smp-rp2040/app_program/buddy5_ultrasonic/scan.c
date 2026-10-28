/**
 * @file scan.c
 * @brief Buddy 5: obstacle scan by turning the car, coarse then fine.
 */

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "../buddy2_motion/motion_task.h"
#include "ultrasonic.h"
#include "servo.h"
#include "scan.h"

/* wheel track, confirmed by the compass */
#define TRACK_MM                 114
#define UM_PER_EDGE              328     /* odometry calibration (Buddy 2) */
#define PING_GAP_MS              70      /* datasheet: >= 60 ms between pings */

#define TURN_MM_S                60      /* repositioning turns (~60 deg/s) */
#define TURN_BRAKE_CDEG          150
#define TURN_SETTLE_MS           200
#define COARSE_MM_S              25      /* ~25 deg/s: a ping every ~2 deg */
#define FINE_STEP_MM_S           40
#define FINE_STEP_CDEG           200     /* stop and look every ~2 deg */
/* Fine steps: the car coasts ~1 deg after the stop command at 40 mm/s
 * (scan timing log 2026-10-22: 2 deg steps braked 1.5 deg early moved
 * 1.45 deg), so brake 1 deg early. Braking from 40 mm/s takes ~60 ms, so
 * 150 ms is enough to be still before the pings. That log's full scan
 * took 29.2 s, 22 s of it in 22 fine steps of ~1 s. */
#define FINE_BRAKE_CDEG          100
#define FINE_SETTLE_MS           150
#define FINE_HALF_CDEG           800     /* fine window: edge +/-8 deg */
#define LOG_MAX                  ((int32_t)SCAN_LOG_MAX)
/* missed pings tolerated inside the obstacle */
#define GAP_ALLOW                2
/* Edges by range gate (27 Oct): a 9 cm toolbox read 33 cm wide because
 * every echo within detect_mm counted, including things 5-20 cm behind and
 * beside it. Now only readings within SCAN_GATE_MM of the nearest one are
 * the obstacle, and an edge is where they stop (one stray reading
 * tolerated). Replaying those two scans: ~6-8 cm against 9 cm. */
#define SCAN_GATE_MM             45
#define GAP_ALLOW_GATE           1
/* Reported width, from the coarse sweep only: the face is the readings
 * within WIDTH_GATE_MM of the face range (median of the nearest third of
 * the readings, so one short echo cannot set it), up to WIDTH_GAP_ALLOW
 * readings off it tolerated. A flat face is a mirror to the sonar, so the
 * span it is seen over is mostly beam and grows only ~0.7 deg per cm of
 * width at the ~190 mm the controller scans from (geometry alone: ~2.7).
 * The span maps to width on a line through the two faces measured
 * (28 Oct, ctrl_course_v16, 7 scans: 9 cm face seen over 47-51 deg,
 * 15 cm over 52-57 deg; the line gives 79-114 and 128-171 mm, within
 * +/-25 mm). Valid for scans from ~190 mm and faces 9-15 cm until a wider
 * face is measured. Earlier rules (beam-subtracted geometry x1.2, gates
 * 30/40 with 3 strays) read the 15 cm face 2-4 cm low. */
#define WIDTH_GATE_MM            30
#define WIDTH_GAP_ALLOW          1
#define WIDTH_REF_SPAN_CDEG      4798    /* 9 cm face, mean seen span */
#define WIDTH_REF_MM             90
#define WIDTH_MM_PER_DEG_X100    910     /* (150 - 90) mm / 6.59 deg */

static scan_pt_t g_log[SCAN_LOG_MAX];
static volatile uint32_t g_log_n = 0u;
static scan_pt_t g_fine[2][SCAN_FINE_MAX];  /* kept for width calibration */
static volatile uint32_t g_fine_n[2] = {0u, 0u};
static const volatile uint8_t *g_p_abort = NULL;
static scan_timing_t g_tm;

static uint32_t
now_ms(void)
{
    SYSTIM tim;

    tk_get_tim(&tim);
    return tim.lo;
}
static int32_t g_l0;
static int32_t g_r0;

/*----------------------------------------------------------------------------
 * Helpers
 *---------------------------------------------------------------------------*/
/* True once the owner has asked the scan to stop (Tribunal 1, R2) */
static bool
aborted(void)
{
    return (NULL != g_p_abort) && (0u != *g_p_abort);
}
/* Car angle since the scan started, left positive, x0.01 deg */
static int32_t
angle_cdeg(void)
{
    enc_diag_t l, r;
    float half_diff;

    (void)motion_get_encoder_diag(&l, &r);
    half_diff = ((float)((r.hw_count - g_r0) - (l.hw_count - g_l0))) * 0.5f;
    return (int32_t)((half_diff * (float)UM_PER_EDGE * 36000.0f)
                     / (3.14159265f * (float)TRACK_MM * 1000.0f));
}

static int16_t
ping_mm(void)
{
    uint32_t e;

    if (E_OK == ultrasonic_read_us(&e))
    {
        return (int16_t)ultrasonic_us_to_mm(e);
    }
    return -1;
}

static int16_t
ping_median3_mm(void)
{
    int16_t v[3];
    int16_t t;
    uint32_t i;

    for (i = 0u; i < 3u; i++)
    {
        v[i] = ping_mm();
        tk_dly_tsk(PING_GAP_MS);
    }
    if (v[0] > v[1]) { t = v[0]; v[0] = v[1]; v[1] = t; }
    if (v[1] > v[2]) { t = v[1]; v[1] = v[2]; v[2] = t; }
    if (v[0] > v[1]) { t = v[0]; v[0] = v[1]; v[1] = t; }
    return v[1];
}

/* Turn on the spot to an absolute scan angle and stop */
static void
turn_to_ex(int32_t target_cdeg, int16_t speed, int32_t brake_cdeg,
           uint32_t settle_ms)
{
    int32_t a;
    uint32_t i;
    bool b_left;

    a = angle_cdeg();
    if ((target_cdeg - a) > -50 && (target_cdeg - a) < 50)
    {
        return;
    }
    b_left = (target_cdeg > a);

    if (b_left)
    {
        (void)motion_set_velocity((int16_t)-speed, speed);
    }
    else
    {
        (void)motion_set_velocity(speed, (int16_t)-speed);
    }
    for (i = 0u; (i < 3000u) && (!aborted()); i++)
    {
        a = angle_cdeg();
        if ((b_left && (a >= target_cdeg - brake_cdeg))
            || ((!b_left) && (a <= target_cdeg + brake_cdeg)))
        {
            break;
        }
        tk_dly_tsk(5);
    }
    (void)motion_set_velocity(0, 0);
    if (!aborted())
    {
        tk_dly_tsk(settle_ms);
    }
}

/* Repositioning turn */
static void
turn_to(int32_t target_cdeg, int16_t speed)
{
    turn_to_ex(target_cdeg, speed, TURN_BRAKE_CDEG, TURN_SETTLE_MS);
}

static bool
is_obstacle(int16_t mm, int16_t detect_mm)
{
    return (mm > 0) && (mm < detect_mm);
}

/* Beam width at a distance, linear between the measured points */
static int32_t
beam_cdeg_at(int32_t mm)
{
    if (mm <= 200)
    {
        return 2800;
    }
    if (mm <= 350)
    {
        return 2800 + ((mm - 200) * (3300 - 2800)) / 150;
    }
    if (mm <= 500)
    {
        return 3300 + ((mm - 350) * (5400 - 3300)) / 150;
    }
    return 5400;
}

/* tan of hundredths of a degree via sin/cos series (fine below ~60 deg) */
static float
tan_cdeg(int32_t cdeg)
{
    float x = (float)cdeg * (3.14159265f / 18000.0f);
    float x2 = x * x;
    float s = x * (1.0f - (x2 / 6.0f)
                          * (1.0f - (x2 / 20.0f) * (1.0f - (x2 / 42.0f))));
    float c = 1.0f - (x2 / 2.0f)
                     * (1.0f - (x2 / 12.0f) * (1.0f - (x2 / 30.0f)));

    return s / c;
}

static float
sin_cdeg(int32_t cdeg)
{
    float x = (float)cdeg * (3.14159265f / 18000.0f);
    float x2 = x * x;

    return x * (1.0f - (x2 / 6.0f)
                       * (1.0f - (x2 / 20.0f) * (1.0f - (x2 / 42.0f))));
}

/* Fine look around one edge: stop every ~2 deg across edge +/-8 deg and
 * return the outermost angle (in the outward direction) where the
 * obstacle is still seen. outward = +1 for the left edge, -1 for the right. */
static int32_t
fine_edge(int32_t edge_cdeg, int32_t outward, int16_t detect_mm, bool b_print)
{
    int32_t a;
    int32_t from;
    int32_t to;
    int32_t best = edge_cdeg;
    int16_t mm;
    uint32_t guard;
    uint32_t misses = 0u;
    bool b_past = false;

    uint32_t t0 = now_ms();
    uint32_t idx = (outward < 0) ? 0u : 1u;
    int32_t a_first = 0;

    from = edge_cdeg - (outward * FINE_HALF_CDEG);
    to = edge_cdeg + (outward * FINE_HALF_CDEG);
    turn_to(from, TURN_MM_S);
    g_tm.fine_steps[idx] = 0u;
    g_fine_n[idx] = 0u;

    for (guard = 0u; (guard < 30u) && (!aborted()); guard++)
    {
        a = angle_cdeg();
        if (0u == guard)
        {
            a_first = a;
        }
        g_tm.fine_steps[idx] = (uint16_t)(guard + 1u);
        g_tm.fine_span_cdeg[idx] = (int16_t)(a - a_first);
        mm = ping_median3_mm();
        if (g_fine_n[idx] < SCAN_FINE_MAX)
        {
            g_fine[idx][g_fine_n[idx]].cdeg = (int16_t)a;
            g_fine[idx][g_fine_n[idx]].mm = mm;
            g_fine_n[idx]++;
        }
        if (b_print)
        {
            tm_printf((UB *)"[SCAN] fine %s %d,%d\n", (outward > 0) ? "L" : "R",
                      (INT)a, (INT)mm);
        }
        /* The edge is the outermost reading still in the gate before the
         * readings leave it for good (more than GAP_ALLOW_GATE in a row) */
        if (is_obstacle(mm, detect_mm))
        {
            misses = 0u;
            if ((!b_past)
                && (((outward > 0) && (a > best)) || ((outward < 0)
                                                      && (a < best))))
            {
                best = a;
            }
        }
        else if (((outward > 0) && (a > best)) || ((outward < 0) && (a < best)))
        {
            if (++misses > GAP_ALLOW_GATE)
            {
                b_past = true;
            }
        }
        if (((outward > 0) && (a >= to)) || ((outward < 0) && (a <= to)))
        {
            break;
        }
        turn_to_ex(a + (outward * FINE_STEP_CDEG), FINE_STEP_MM_S,
                   FINE_BRAKE_CDEG, FINE_SETTLE_MS);
    }
    g_tm.fine_ms[idx] = now_ms() - t0;
    return best;
}

/* Width from the coarse sweep (see WIDTH_*); 0 if below resolution.
 * *p_capped is set if the face still ran on at either end of the sweep:
 * the width is then only a lower bound. */
static int16_t
coarse_width_mm(int32_t n, int32_t near_i, bool *p_capped)
{
    int16_t v[LOG_MAX];
    int16_t t;
    int32_t m = 0;
    int32_t i;
    int32_t j;
    int32_t face;
    int32_t gate;
    int32_t lo;
    int32_t hi;
    int32_t gap;
    int32_t w;

    *p_capped = false;
    for (i = 0; i < n; i++)
    {
        if (g_log[i].mm > 0)
        {
            v[m++] = g_log[i].mm;
        }
    }
    if (0 == m)
    {
        return 0;
    }
    for (i = 1; i < m; i++)               /* insertion sort, <= 128 values */
    {
        t = v[i];
        for (j = i - 1; (j >= 0) && (v[j] > t); j--)
        {
            v[j + 1] = v[j];
        }
        v[j + 1] = t;
    }
    j = (m / 3 > 3) ? (m / 3) : ((m < 3) ? m : 3);
    face = v[j / 2];
    gate = face + WIDTH_GATE_MM;

    lo = near_i;
    gap = 0;
    for (j = near_i - 1; j >= 0; j--)
    {
        if (is_obstacle(g_log[j].mm, (int16_t)gate))
        {
            lo = j;
            gap = 0;
        }
        else if (++gap > WIDTH_GAP_ALLOW)
        {
            break;
        }
    }
    hi = near_i;
    gap = 0;
    for (j = near_i + 1; j < n; j++)
    {
        if (is_obstacle(g_log[j].mm, (int16_t)gate))
        {
            hi = j;
            gap = 0;
        }
        else if (++gap > WIDTH_GAP_ALLOW)
        {
            break;
        }
    }
    *p_capped = (0 == lo) || ((n - 1) == hi);
    w = WIDTH_REF_MM
        + (((((int32_t)g_log[hi].cdeg - (int32_t)g_log[lo].cdeg)
             - WIDTH_REF_SPAN_CDEG) * WIDTH_MM_PER_DEG_X100) / 10000);
    return (int16_t)((w > 0) ? w : 0);
}

/*----------------------------------------------------------------------------
 * Scan
 *---------------------------------------------------------------------------*/
ER
scan_obstacle(int16_t half_deg, int16_t detect_mm, bool b_print,
              obstacle_profile_t *p_prof)
{
    enc_diag_t l, r;
    int32_t n = 0;
    int32_t i, j;
    int32_t near_i = -1;
    int32_t lo, hi, gap;
    int32_t old_lo, old_hi;
    int32_t gate;
    int32_t a;
    int32_t span, beam, excess, centre;
    float d;
    float half_w;
    float centre_off;

    if (NULL == p_prof)
    {
        return E_PAR;
    }
    p_prof->found = false;
    g_tm.start_ms = 0u;
    g_tm.coarse_ms = 0u;
    g_tm.coarse_samples = 0u;
    g_tm.fine_ms[0] = 0u;
    g_tm.fine_ms[1] = 0u;
    g_tm.fine_steps[0] = 0u;
    g_tm.fine_steps[1] = 0u;
    g_tm.fine_span_cdeg[0] = 0;
    g_tm.fine_span_cdeg[1] = 0;
    g_tm.return_ms = 0u;
    g_tm.t0_ms = now_ms();
    g_log_n = 0u;
    g_fine_n[0] = 0u;
    g_fine_n[1] = 0u;

    (void)motion_get_encoder_diag(&l, &r);
    g_l0 = l.hw_count;
    g_r0 = r.hw_count;
    (void)servo_set_us(SERVO_CENTRE_US);

    /* ---- Coarse: continuous sweep right to left ---- */
    turn_to(-(int32_t)half_deg * 100, TURN_MM_S);
    g_tm.start_ms = now_ms() - g_tm.t0_ms;
    (void)motion_set_velocity((int16_t)-COARSE_MM_S, (int16_t)COARSE_MM_S);
    while ((n < LOG_MAX) && (!aborted()))
    {
        a = angle_cdeg();
        g_log[n].cdeg = (int16_t)a;
        g_log[n].mm = ping_mm();
        n++;
        if (a >= (int32_t)half_deg * 100)
        {
            break;
        }
        tk_dly_tsk(PING_GAP_MS);
    }
    (void)motion_set_velocity(0, 0);
    g_tm.coarse_ms = now_ms() - g_tm.t0_ms - g_tm.start_ms;
    g_tm.coarse_samples = (uint16_t)n;
    g_log_n = (uint32_t)n;
    if (aborted())
    {
        return E_RLWAI;
    }
    tk_dly_tsk(TURN_SETTLE_MS);

    if (b_print)
    {
        tm_printf((UB *)"[SCAN] coarse: angle_cdeg,range_mm\n");
        for (i = 0; i < n; i++)
        {
            tm_printf((UB *)"[SCAN] %d,%d\n", (INT)g_log[i].cdeg,
                      (INT)g_log[i].mm);
        }
    }

    /* Nearest obstacle reading, then grow its stretch both ways, allowing
     * a couple of missed pings (narrow targets drop echoes) */
    for (i = 0; i < n; i++)
    {
        if (is_obstacle(g_log[i].mm, detect_mm)
            && ((near_i < 0) || (g_log[i].mm < g_log[near_i].mm)))
        {
            near_i = i;
        }
    }
    if (near_i < 0)
    {
        turn_to(0, TURN_MM_S);
        return E_OK;                         /* nothing in range */
    }

    /* Old rule (anything within detect_mm), kept for comparison only */
    old_lo = near_i;
    gap = 0;
    for (j = near_i - 1; j >= 0; j--)
    {
        if (is_obstacle(g_log[j].mm, detect_mm))
        {
            old_lo = j;
            gap = 0;
        }
        else if (++gap > GAP_ALLOW)
        {
            break;
        }
    }
    old_hi = near_i;
    gap = 0;
    for (j = near_i + 1; j < n; j++)
    {
        if (is_obstacle(g_log[j].mm, detect_mm))
        {
            old_hi = j;
            gap = 0;
        }
        else if (++gap > GAP_ALLOW)
        {
            break;
        }
    }

    /* Range gate: the obstacle is what lies within SCAN_GATE_MM of the
     * nearest reading */
    gate = (int32_t)g_log[near_i].mm + SCAN_GATE_MM;
    lo = near_i;
    gap = 0;
    for (j = near_i - 1; j >= 0; j--)
    {
        if (is_obstacle(g_log[j].mm, (int16_t)gate))
        {
            lo = j;
            gap = 0;
        }
        else if (++gap > GAP_ALLOW_GATE)
        {
            break;
        }
    }
    hi = near_i;
    gap = 0;
    for (j = near_i + 1; j < n; j++)
    {
        if (is_obstacle(g_log[j].mm, (int16_t)gate))
        {
            hi = j;
            gap = 0;
        }
        else if (++gap > GAP_ALLOW_GATE)
        {
            break;
        }
    }

    p_prof->found = true;
    p_prof->nearest_mm = g_log[near_i].mm;
    p_prof->nearest_cdeg = g_log[near_i].cdeg;

    /* ---- Fine: stop-and-look around each coarse edge ---- */
    p_prof->right_cdeg = (int16_t)fine_edge(g_log[lo].cdeg, -1, (int16_t)gate,
                                            b_print);
    p_prof->left_cdeg = (int16_t)fine_edge(g_log[hi].cdeg, +1, (int16_t)gate,
                                           b_print);
    if (aborted())
    {
        p_prof->found = false;
        return E_RLWAI;
    }

    {
        uint32_t t_ret = now_ms();

        turn_to(0, TURN_MM_S);
        g_tm.return_ms = now_ms() - t_ret;
    }

    /* ---- Profile ---- */
    span = (int32_t)p_prof->left_cdeg - (int32_t)p_prof->right_cdeg;
    beam = beam_cdeg_at(p_prof->nearest_mm);
    excess = span - beam;
    d = (float)p_prof->nearest_mm;
    p_prof->beam_cdeg = (int16_t)beam;

    half_w = (excess > 0) ? (d * tan_cdeg(excess / 2)) : 0.0f;
    p_prof->width_mm = coarse_width_mm(n, near_i, &p_prof->width_capped);

    /* Old rule's estimate from the coarse sweep alone (comparison) */
    excess = (int32_t)g_log[old_hi].cdeg - (int32_t)g_log[old_lo].cdeg - beam;
    p_prof->width_old_mm = (int16_t)((excess > 0)
                                     ? (2.0f * d * tan_cdeg(excess / 2))
                                     : 0.0f);

    /* Centre bearing, then the sides relative to the car's centreline */
    centre = ((int32_t)p_prof->left_cdeg + (int32_t)p_prof->right_cdeg) / 2;
    centre_off = d * sin_cdeg(centre);
    p_prof->left_side_mm = (int16_t)(centre_off + half_w);
    p_prof->right_side_mm = (int16_t)(centre_off - half_w);

    return E_OK;
}

void
scan_set_abort_flag(const volatile uint8_t *p_flag)
{
    g_p_abort = p_flag;
}

uint32_t
scan_get_coarse(scan_pt_t *p_out, uint32_t max)
{
    uint32_t i;
    uint32_t n = g_log_n;

    if (NULL == p_out)
    {
        return 0u;
    }
    n = (n < max) ? n : max;
    for (i = 0u; i < n; i++)
    {
        p_out[i] = g_log[i];
    }
    return n;
}

uint32_t
scan_get_fine(uint32_t edge, scan_pt_t *p_out, uint32_t max)
{
    uint32_t i;
    uint32_t n;

    if ((NULL == p_out) || (edge > 1u))
    {
        return 0u;
    }
    n = g_fine_n[edge];
    n = (n < max) ? n : max;
    for (i = 0u; i < n; i++)
    {
        p_out[i] = g_fine[edge][i];
    }
    return n;
}

void
scan_get_timing(scan_timing_t *p_t)
{
    if (NULL != p_t)
    {
        *p_t = g_tm;
    }
}
