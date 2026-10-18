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

#define TRACK_MM                 114     /* wheel track, confirmed by the compass */
#define UM_PER_EDGE              328     /* odometry calibration (Buddy 2) */
#define PING_GAP_MS              70      /* datasheet: >= 60 ms between pings */

#define TURN_MM_S                60      /* repositioning turns (~60 deg/s) */
#define TURN_BRAKE_CDEG          150
#define COARSE_MM_S              25      /* ~25 deg/s: a ping every ~2 deg */
#define FINE_STEP_MM_S           40
#define FINE_STEP_CDEG           200     /* stop and look every ~2 deg */
#define FINE_HALF_CDEG           800     /* fine window: edge +/-8 deg */
#define LOG_MAX                  128
#define GAP_ALLOW                2       /* missed pings tolerated inside the obstacle */

typedef struct {
    int16_t cdeg;
    int16_t mm;                  /* -1 = no echo */
} scan_pt_t;

static scan_pt_t g_log[LOG_MAX];
static int32_t g_l0;
static int32_t g_r0;

/*----------------------------------------------------------------------------
 * Helpers
 *---------------------------------------------------------------------------*/
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
turn_to(int32_t target_cdeg, int16_t speed)
{
    int32_t a;
    uint32_t i;
    bool left;

    a = angle_cdeg();
    if ((target_cdeg - a) > -50 && (target_cdeg - a) < 50)
    {
        return;
    }
    left = (target_cdeg > a);

    if (left)
    {
        (void)motion_set_velocity((int16_t)-speed, speed);
    }
    else
    {
        (void)motion_set_velocity(speed, (int16_t)-speed);
    }
    for (i = 0u; i < 3000u; i++)
    {
        a = angle_cdeg();
        if ((left && (a >= target_cdeg - TURN_BRAKE_CDEG))
            || ((!left) && (a <= target_cdeg + TURN_BRAKE_CDEG)))
        {
            break;
        }
        tk_dly_tsk(5);
    }
    (void)motion_set_velocity(0, 0);
    tk_dly_tsk(500);
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
    float s = x * (1.0f - (x2 / 6.0f) * (1.0f - (x2 / 20.0f) * (1.0f - (x2 / 42.0f))));
    float c = 1.0f - (x2 / 2.0f) * (1.0f - (x2 / 12.0f) * (1.0f - (x2 / 30.0f)));

    return s / c;
}

static float
sin_cdeg(int32_t cdeg)
{
    float x = (float)cdeg * (3.14159265f / 18000.0f);
    float x2 = x * x;

    return x * (1.0f - (x2 / 6.0f) * (1.0f - (x2 / 20.0f) * (1.0f - (x2 / 42.0f))));
}

/* Fine look around one edge: stop every ~2 deg across edge +/-8 deg and
 * return the outermost angle (in the outward direction) where the
 * obstacle is still seen. outward = +1 for the left edge, -1 for the right. */
static int32_t
fine_edge(int32_t edge_cdeg, int32_t outward, int16_t detect_mm, bool print)
{
    int32_t a;
    int32_t from;
    int32_t to;
    int32_t best = edge_cdeg;
    int16_t mm;
    uint32_t guard;

    from = edge_cdeg - (outward * FINE_HALF_CDEG);
    to = edge_cdeg + (outward * FINE_HALF_CDEG);
    turn_to(from, TURN_MM_S);

    for (guard = 0u; guard < 30u; guard++)
    {
        a = angle_cdeg();
        mm = ping_median3_mm();
        if (print)
        {
            tm_printf((UB *)"[SCAN] fine %s %d,%d\n", (outward > 0) ? "L" : "R", (INT)a, (INT)mm);
        }
        if (is_obstacle(mm, detect_mm)
            && (((outward > 0) && (a > best)) || ((outward < 0) && (a < best))))
        {
            best = a;
        }
        if (((outward > 0) && (a >= to)) || ((outward < 0) && (a <= to)))
        {
            break;
        }
        turn_to(a + (outward * FINE_STEP_CDEG), FINE_STEP_MM_S);
    }
    return best;
}

/*----------------------------------------------------------------------------
 * Scan
 *---------------------------------------------------------------------------*/
ER
scan_obstacle(int16_t half_deg, int16_t detect_mm, bool print,
              obstacle_profile_t *p_prof)
{
    enc_diag_t l, r;
    int32_t n = 0;
    int32_t i, j;
    int32_t near_i = -1;
    int32_t lo, hi, gap;
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

    (void)motion_get_encoder_diag(&l, &r);
    g_l0 = l.hw_count;
    g_r0 = r.hw_count;
    (void)servo_set_us(SERVO_CENTRE_US);

    /* ---- Coarse: continuous sweep right to left ---- */
    turn_to(-(int32_t)half_deg * 100, TURN_MM_S);
    (void)motion_set_velocity((int16_t)-COARSE_MM_S, (int16_t)COARSE_MM_S);
    while (n < LOG_MAX)
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
    tk_dly_tsk(500);

    if (print)
    {
        tm_printf((UB *)"[SCAN] coarse: angle_cdeg,range_mm\n");
        for (i = 0; i < n; i++)
        {
            tm_printf((UB *)"[SCAN] %d,%d\n", (INT)g_log[i].cdeg, (INT)g_log[i].mm);
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

    lo = near_i;
    gap = 0;
    for (j = near_i - 1; j >= 0; j--)
    {
        if (is_obstacle(g_log[j].mm, detect_mm))
        {
            lo = j;
            gap = 0;
        }
        else if (++gap > GAP_ALLOW)
        {
            break;
        }
    }
    hi = near_i;
    gap = 0;
    for (j = near_i + 1; j < n; j++)
    {
        if (is_obstacle(g_log[j].mm, detect_mm))
        {
            hi = j;
            gap = 0;
        }
        else if (++gap > GAP_ALLOW)
        {
            break;
        }
    }

    p_prof->found = true;
    p_prof->nearest_mm = g_log[near_i].mm;
    p_prof->nearest_cdeg = g_log[near_i].cdeg;

    /* ---- Fine: stop-and-look around each coarse edge ---- */
    p_prof->right_cdeg = (int16_t)fine_edge(g_log[lo].cdeg, -1, detect_mm, print);
    p_prof->left_cdeg = (int16_t)fine_edge(g_log[hi].cdeg, +1, detect_mm, print);

    turn_to(0, TURN_MM_S);

    /* ---- Profile ---- */
    span = (int32_t)p_prof->left_cdeg - (int32_t)p_prof->right_cdeg;
    beam = beam_cdeg_at(p_prof->nearest_mm);
    excess = span - beam;
    d = (float)p_prof->nearest_mm;
    p_prof->beam_cdeg = (int16_t)beam;

    half_w = (excess > 0) ? (d * tan_cdeg(excess / 2)) : 0.0f;
    p_prof->width_mm = (int16_t)(2.0f * half_w);

    /* Centre bearing, then the sides relative to the car's centreline */
    centre = ((int32_t)p_prof->left_cdeg + (int32_t)p_prof->right_cdeg) / 2;
    centre_off = d * sin_cdeg(centre);
    p_prof->left_side_mm = (int16_t)(centre_off + half_w);
    p_prof->right_side_mm = (int16_t)(centre_off - half_w);

    return E_OK;
}
