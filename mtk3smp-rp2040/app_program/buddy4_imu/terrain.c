/**
 * @file terrain.c
 * @brief Buddy 4: real-time hump and lift detection.
 *
 * Thresholds come from the ramp logs of 2026-10-09 (200 mm/s):
 *   flat floor, filtered pitch noise RMS 0.7 deg
 *   1.0 cm ramps, most nose-down 30 mm mean -8.6 deg
 *   2.4 cm ramps, most nose-down 30 mm mean -17.9 deg
 * A hump starts when filtered pitch stays more than 4 deg nose-down for
 * 15 mm, and ends when it stays within 2 deg of level for 15 mm.
 *
 * One-sided hump (the course test: a ramp under one wheel only). The car
 * rolls instead of pitching: with one wheel raised h above the other,
 * roll = asin(h / track). One-sided ramp log of 2026-10-21 (150 mm/s,
 * 114 mm track): raw roll noise +/-3-6 deg while driving; mean roll on
 * the 1.0 cm ramp about -4.4 deg (h ~ 9 mm), on the 2.4 cm ramp about
 * -13 deg (h ~ 26 mm). A side hump starts when filtered roll stays past
 * 2.5 deg for 20 mm and ends when it stays within 1.5 deg for 15 mm; the
 * height comes from the most-tilted 30 mm mean. Roll is not gated on
 * steady speed: braking and speeding up tilt the car in pitch, not roll.
 */

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>

#include "terrain.h"

#define FILT_N                   7       /* causal moving average */
#define WIN_MAX                  32      /* samples held for the 30 mm window */
#define PLAT_MM                  30      /* plateau window length */
#define PLAT_MIN_SPAN_MM         25      /* window must span this much */
#define HUMP_ENTER_CDEG          (-400)  /* nose-down past -4 deg ... */
#define HUMP_EXIT_CDEG           (-200)  /* ... back within -2 deg ... */
#define HUMP_HOLD_MM             15      /* ... for this much travel */
#define WHEELBASE_MM             80.0f   /* drive axle to castor */
#define SIDE_ENTER_CDEG          250     /* |roll| past 2.5 deg ... */
#define SIDE_ENTER_MM            20      /* ... for this much travel */
#define SIDE_EXIT_CDEG           150     /* back within 1.5 deg ... */
#define SIDE_EXIT_MM             15      /* ... for this much travel */
#define TRACK_MM                 114.0f  /* wheel track: lever for roll */

/* Lift (the demo's end: picked up, the motors stop). A cross blackens all
 * three IRs for ~18 mm (0.18 s at 100 mm/s). On the car (v57, 18 Nov) the
 * IRs saw every lift, while the IMU missed a slow one (17 deg, 251 mg) and
 * normal driving reached 27.7 deg in turns; a light ramp (13 mm, under a
 * wheel) never blackened all three. So no floor for 1 s is a lift on its
 * own, and an IMU cue only makes it quicker (0.5 s). */
#define LIFT_TILT_CDEG           2500    /* tilted past 25 deg from rest */
#define LIFT_TILT_MS             100u    /* ... for this long */
#define LIFT_ACCEL_MG            300     /* |a| this far from 1 g */
#define LIFT_CUE_KEEP_MS         2000u   /* a cue arms the IR check this long */
/* ...after a cue, all three IRs black this long: normal driving reached
 * 32 deg and 328 mg (both cues) and a cross is ~0.18 s, so 0.3 s left too
 * little margin (19 Nov) */
#define LIFT_NO_FLOOR_MS         500u
#define LIFT_RAMP_QUIET_MS       1000u   /* accel cue ignored after a ramp */
/* all three IRs black this long is a lift with or without an IMU cue */
#define LIFT_NO_FLOOR_ALONE_MS   1000u
/* ...or this long while terrain_lift_alone(false): turning on the spot in
 * the obstacle sequence, all three IRs sat on a cross arm for 1 s and it
 * read as a lift (24 Nov). The arm fits under the IR row within about +/-12
 * deg, at most ~5 of the face search's 5 deg steps (~2.3 s); switched fully
 * off instead, a gentle lift the IMU missed was not stopped for seconds. */
#define LIFT_NO_FLOOR_SEQ_MS     3000u
static bool g_b_alone = true;
#define ONE_G_MG                 1000

typedef enum {
    STATE_LEVEL = 0,
    STATE_IN_HUMP
} terrain_state_t;

static int16_t g_rest_pitch = 0;
static int16_t g_pitch_ring[FILT_N];
static uint32_t g_pitch_n = 0u;

/* Distance window of filtered tilt samples, for the 30 mm plateau mean */
typedef struct {
    int32_t  d[WIN_MAX];
    int16_t  v[WIN_MAX];
    uint32_t head;                       /* index of the oldest entry */
    uint32_t count;
} tilt_win_t;

static tilt_win_t g_pwin;                /* pitch */
static tilt_win_t g_rwin;                /* roll */

/* One-sided hump (roll) */
static int16_t g_rest_roll = 0;
static int16_t g_roll_ring[FILT_N];
static uint32_t g_roll_n = 0u;
static terrain_state_t g_side_state = STATE_LEVEL;
static int32_t g_side_in_since = -1;
static int32_t g_side_out_since = -1;
static side_hump_event_t g_side;
static int16_t g_max_roll = 0;           /* largest |filtered roll| seen */
/* largest |filtered roll| in a pitch hump */
static int16_t g_hump_roll_max = 0;

static terrain_state_t g_state = STATE_LEVEL;
static int32_t g_below_since = -1;
static int32_t g_above_since = -1;
static hump_event_t g_hump;

static int16_t g_min_pitch = 0;

/* Lift check */
static bool g_b_lift_armed = false;      /* an IMU cue in the last 2 s */
static uint32_t g_lift_cue_t = 0u;
static lift_event_t g_lift_cue;
static bool g_b_tilted = false;
static uint32_t g_tilt_since = 0u;
static bool g_b_no_floor = false;
static uint32_t g_no_floor_since = 0u;
static bool g_b_ramp_seen = false;
static uint32_t g_ramp_t = 0u;           /* last period on the ramp */
static bool g_b_lifted = false;          /* reported once per run */
static int16_t g_peak_tilt = 0;
static int16_t g_peak_accel = 0;

/* sin of hundredths of a degree, Taylor to x^7 (no libm) */
static float
sin_cdeg(int32_t cdeg)
{
    float x = (float)cdeg * (3.14159265f / 18000.0f);
    float x2 = x * x;

    return x * (1.0f - (x2 / 6.0f)
                       * (1.0f - (x2 / 20.0f) * (1.0f - (x2 / 42.0f))));
}

static int16_t
abs16(int16_t v)
{
    return (v < 0) ? (int16_t)(-v) : v;
}

/* Push one (distance, tilt) pair and drop entries older than 30 mm.
 * Returns true and the window mean once the window spans enough travel. */
static bool
window_push(tilt_win_t *p_w, int32_t dist_mm, int16_t v, int16_t *p_mean,
            int32_t *p_from_mm)
{
    uint32_t i;
    uint32_t idx;
    int32_t sum = 0;

    if (WIN_MAX == p_w->count)
    {
        p_w->head = (p_w->head + 1u) % WIN_MAX;
        p_w->count--;
    }
    idx = (p_w->head + p_w->count) % WIN_MAX;
    p_w->d[idx] = dist_mm;
    p_w->v[idx] = v;
    p_w->count++;

    while ((p_w->count > 1u) && ((dist_mm - p_w->d[p_w->head]) > PLAT_MM))
    {
        p_w->head = (p_w->head + 1u) % WIN_MAX;
        p_w->count--;
    }

    if ((dist_mm - p_w->d[p_w->head]) < PLAT_MIN_SPAN_MM)
    {
        return false;
    }

    for (i = 0u; i < p_w->count; i++)
    {
        sum += p_w->v[(p_w->head + i) % WIN_MAX];
    }
    *p_mean = (int16_t)(sum / (int32_t)p_w->count);
    *p_from_mm = p_w->d[p_w->head];
    return true;
}

/* Causal moving average over the last FILT_N samples */
static int16_t
filt_push(int16_t *p_ring, uint32_t *p_n, int16_t v)
{
    uint32_t i;
    uint32_t n;
    int32_t sum = 0;

    p_ring[*p_n % FILT_N] = v;
    (*p_n)++;
    n = (*p_n < FILT_N) ? *p_n : FILT_N;
    for (i = 0u; i < n; i++)
    {
        sum += p_ring[i];
    }
    return (int16_t)(sum / (int32_t)n);
}

/* One-sided hump: roll state machine, one call per sample */
static uint32_t
side_update(int32_t dist_mm, int16_t fr)
{
    int16_t mean;
    int16_t afr = abs16(fr);
    int32_t from_mm;
    uint32_t events = 0u;

    if (afr > g_max_roll)
    {
        g_max_roll = afr;
    }

    if (window_push(&g_rwin, dist_mm, fr, &mean, &from_mm)
        && (STATE_IN_HUMP == g_side_state)
        && (abs16(mean) > abs16(g_side.plateau_cdeg)))
    {
        g_side.plateau_cdeg = mean;
        g_side.plateau_mm = from_mm;
    }

    if (STATE_LEVEL == g_side_state)
    {
        if (afr > SIDE_ENTER_CDEG)
        {
            if (g_side_in_since < 0)
            {
                g_side_in_since = dist_mm;
            }
            if ((dist_mm - g_side_in_since) >= SIDE_ENTER_MM)
            {
                g_side_state = STATE_IN_HUMP;
                g_side.start_mm = g_side_in_since;
                g_side.plateau_cdeg = fr;
                g_side.plateau_mm = dist_mm;
                g_side_out_since = -1;
            }
        }
        else
        {
            g_side_in_since = -1;
        }
    }
    else if (afr < SIDE_EXIT_CDEG)
    {
        if (g_side_out_since < 0)
        {
            g_side_out_since = dist_mm;
        }
        if ((dist_mm - g_side_out_since) >= SIDE_EXIT_MM)
        {
            g_side.end_mm = g_side_out_since;
            g_side.height_mm = (int16_t)(TRACK_MM
                * sin_cdeg((int32_t)abs16(g_side.plateau_cdeg)) + 0.5f);
            /* Roll is right-side-down positive: raised right wheel =
               negative */
            g_side.right_side = (g_side.plateau_cdeg < 0) ? 1u : 0u;
            g_side_state = STATE_LEVEL;
            g_side_in_since = -1;
            g_side_out_since = -1;
            events |= TERRAIN_EVT_SIDE_HUMP;
        }
    }
    else
    {
        g_side_out_since = -1;
    }

    return events;
}

/* Close a pitch hump. Returns false (no event) if the car also rolled
 * past the side-hump threshold during it: then it was a one-sided ramp,
 * which the roll detector reports, and counting it here too would report
 * the same ramp twice (one-sided ramp test, 2026-10-21). */
static bool
hump_finish(int32_t dist_mm, hump_event_t *p_hump)
{
    bool b_report = (g_hump_roll_max < SIDE_ENTER_CDEG)
        && (STATE_LEVEL == g_side_state);

    g_hump.end_mm = dist_mm;
    g_hump.height_mm = (int16_t)(WHEELBASE_MM
                                 * sin_cdeg(-(int32_t)g_hump.plateau_cdeg)
                                 + 0.5f);
    if (b_report)
    {
        *p_hump = g_hump;
    }
    g_state = STATE_LEVEL;
    g_below_since = -1;
    g_above_since = -1;
    return b_report;
}

void
terrain_reset(int16_t rest_pitch_cdeg, int16_t rest_roll_cdeg)
{
    g_b_alone = true;
    g_rest_pitch = rest_pitch_cdeg;
    g_pitch_n = 0u;
    g_pwin.head = 0u;
    g_pwin.count = 0u;
    g_rest_roll = rest_roll_cdeg;
    g_roll_n = 0u;
    g_rwin.head = 0u;
    g_rwin.count = 0u;
    g_side_state = STATE_LEVEL;
    g_side_in_since = -1;
    g_side_out_since = -1;
    g_max_roll = 0;
    g_hump_roll_max = 0;
    g_state = STATE_LEVEL;
    g_below_since = -1;
    g_above_since = -1;
    g_min_pitch = 0;
    g_b_lift_armed = false;
    g_b_tilted = false;
    g_b_no_floor = false;
    g_b_ramp_seen = false;
    g_b_lifted = false;
    g_peak_tilt = 0;
    g_peak_accel = 0;
}

uint32_t
terrain_update(int32_t dist_mm, const imu_attitude_t *p_att, bool b_steady,
               hump_event_t *p_hump)
{
    uint32_t events = 0u;
    int16_t fp;
    int16_t fr;
    int16_t mean;
    int32_t from_mm;

    if ((NULL == p_att) || (NULL == p_hump))
    {
        return 0u;
    }

    /* Causal moving averages of pitch and roll, relative to rest */
    fp = (int16_t)(filt_push(g_pitch_ring, &g_pitch_n,
                             p_att->pitch_cdeg) - g_rest_pitch);
    fr = (int16_t)(filt_push(g_roll_ring, &g_roll_n,
                             p_att->roll_cdeg) - g_rest_roll);

    /* ---- One-sided hump (roll) ---- */
    events |= side_update(dist_mm, fr);

    if (b_steady && (fp < g_min_pitch))
    {
        g_min_pitch = fp;
    }

    /* ---- Hump ---- */
    if (window_push(&g_pwin, dist_mm, fp, &mean, &from_mm)
        && (STATE_IN_HUMP == g_state)
        && (mean < g_hump.plateau_cdeg))
    {
        g_hump.plateau_cdeg = mean;
        g_hump.plateau_mm = from_mm;
    }

    if (STATE_LEVEL == g_state)
    {
        if (b_steady && (fp < HUMP_ENTER_CDEG))
        {
            if (g_below_since < 0)
            {
                g_below_since = dist_mm;
            }
            if ((dist_mm - g_below_since) >= HUMP_HOLD_MM)
            {
                g_state = STATE_IN_HUMP;
                g_hump.start_mm = g_below_since;
                g_hump.plateau_cdeg = fp;
                g_hump.plateau_mm = dist_mm;
                g_hump_roll_max = 0;
                g_above_since = -1;
            }
        }
        else
        {
            g_below_since = -1;
        }
    }
    else
    {
        if (abs16(fr) > g_hump_roll_max)
        {
            g_hump_roll_max = abs16(fr);
        }

        if (!b_steady)
        {
            /* Braking or a speed change: tilt is no longer trustworthy,
               so close the hump with what has been seen so far. */
            if (hump_finish(dist_mm, p_hump))
            {
                events |= TERRAIN_EVT_HUMP;
            }
            /* else: dropped as one-sided */
        }
        else if (fp > HUMP_EXIT_CDEG)
        {
            if (g_above_since < 0)
            {
                g_above_since = dist_mm;
            }
            if ((dist_mm - g_above_since) >= HUMP_HOLD_MM)
            {
                if (hump_finish(g_above_since, p_hump))
                {
                    events |= TERRAIN_EVT_HUMP;
                }
                /* else: dropped as one-sided */
            }
        }
        else
        {
            g_above_since = -1;
        }
    }

    return events;
}

/* Tilt from rest, x0.01 deg: the larger of |pitch| and |roll| */
static int16_t
tilt_from_rest(const imu_attitude_t *p_att)
{
    int32_t p = (int32_t)p_att->pitch_cdeg - g_rest_pitch;
    int32_t r = (int32_t)p_att->roll_cdeg - g_rest_roll;

    p = (p < 0) ? -p : p;
    r = (r < 0) ? -r : r;
    p = (p > 18000) ? (36000 - p) : p;     /* across +/-180 deg */
    r = (r > 18000) ? (36000 - r) : r;
    return (int16_t)((p > r) ? p : r);
}

/* |a| - 1 g, mg (a = the calibrated accelerometer, mg) */
static int16_t
accel_off_1g(const imu_raw_t *p_cal)
{
    int32_t sq = ((int32_t)p_cal->ax * p_cal->ax)
                 + ((int32_t)p_cal->ay * p_cal->ay)
                 + ((int32_t)p_cal->az * p_cal->az);
    int32_t m = ONE_G_MG;
    int32_t off;

    /* integer square root by Newton steps from 1 g (|a| < 8 g here) */
    if (sq > 0)
    {
        m = (m + (sq / m)) / 2;
        m = (m + (sq / m)) / 2;
        m = (m + (sq / m)) / 2;
        m = (m + (sq / m)) / 2;
        m = (m + (sq / m)) / 2;
    }
    else
    {
        m = 0;
    }
    off = m - ONE_G_MG;
    return (int16_t)((off < 0) ? -off : off);
}

static void
lift_cue(uint32_t t_ms, uint8_t cue, int16_t val)
{
    g_b_lift_armed = true;
    g_lift_cue_t = t_ms;
    g_lift_cue.cue = cue;
    g_lift_cue.cue_val = val;
}

bool
terrain_lift_update(uint32_t t_ms, const imu_attitude_t *p_att,
                    const imu_raw_t *p_cal, bool b_no_floor, bool b_on_ramp,
                    lift_event_t *p_lift)
{
    int16_t tilt;
    int16_t acc;
    bool b_ramp_quiet;

    if (g_b_lifted)
    {
        return false;
    }
    /* On the ramp only the acceleration cue is off (its bumps are normal
     * there). The tilt cue and the no-floor timer keep running: a ramp
     * never tilts the car 25 deg, and a lifted car's free wheels and the
     * hand's tilt soon make the hump detector report a ramp, which must
     * not switch the lift check off. */
    if (b_on_ramp)
    {
        g_b_ramp_seen = true;
        g_ramp_t = t_ms;
        if (TERRAIN_LIFT_ACCEL == g_lift_cue.cue)
        {
            g_b_lift_armed = false;      /* that was the ramp's jolt */
        }
    }
    b_ramp_quiet = b_on_ramp
                   || (g_b_ramp_seen
                       && ((t_ms - g_ramp_t) < LIFT_RAMP_QUIET_MS));

    if ((NULL != p_att) && (NULL != p_cal))
    {
        tilt = tilt_from_rest(p_att);
        acc = accel_off_1g(p_cal);
        if (tilt > g_peak_tilt)
        {
            g_peak_tilt = tilt;
        }
        if ((!b_ramp_quiet) && (acc > g_peak_accel))
        {
            g_peak_accel = acc;
        }

        if (tilt > LIFT_TILT_CDEG)
        {
            if (!g_b_tilted)
            {
                g_b_tilted = true;
                g_tilt_since = t_ms;
            }
            if ((t_ms - g_tilt_since) >= LIFT_TILT_MS)
            {
                lift_cue(t_ms, TERRAIN_LIFT_TILT, tilt);
            }
        }
        else
        {
            g_b_tilted = false;
        }
        if ((!b_ramp_quiet) && (acc > LIFT_ACCEL_MG))
        {
            lift_cue(t_ms, TERRAIN_LIFT_ACCEL, acc);
        }
    }
    if (g_b_lift_armed && ((t_ms - g_lift_cue_t) > LIFT_CUE_KEEP_MS))
    {
        g_b_lift_armed = false;
    }

    if (!b_no_floor)
    {
        g_b_no_floor = false;
        return false;
    }
    if (!g_b_no_floor)
    {
        g_b_no_floor = true;
        g_no_floor_since = t_ms;
    }
    /* No floor for 1 s is a lift by itself (in or out of a "ramp": a
     * lifted car's free wheels and a small tilt make the hump detector
     * hold it as one). An IMU cue makes it 0.5 s; on the ramp only a
     * tilt cue counts, an acceleration cue being the ramp's own jolt. */
    if ((g_b_lift_armed
         && ((!b_on_ramp) || (TERRAIN_LIFT_TILT == g_lift_cue.cue))
         && ((t_ms - g_no_floor_since) >= LIFT_NO_FLOOR_MS))
        || ((t_ms - g_no_floor_since)
            >= (g_b_alone ? LIFT_NO_FLOOR_ALONE_MS : LIFT_NO_FLOOR_SEQ_MS)))
    {
        if ((!g_b_lift_armed)
            || (b_on_ramp && (TERRAIN_LIFT_TILT != g_lift_cue.cue)))
        {
            g_lift_cue.cue = TERRAIN_LIFT_NO_FLOOR;
            g_lift_cue.cue_val = (int16_t)(t_ms - g_no_floor_since);
        }
        g_b_lifted = true;
        if (NULL != p_lift)
        {
            *p_lift = g_lift_cue;
        }
        return true;
    }
    return false;
}

void
terrain_lift_alone(bool b_on)
{
    g_b_alone = b_on;
}

void
terrain_lift_peaks(int16_t *p_tilt_cdeg, int16_t *p_accel_mg)
{
    if (NULL != p_tilt_cdeg)
    {
        *p_tilt_cdeg = g_peak_tilt;
    }
    if (NULL != p_accel_mg)
    {
        *p_accel_mg = g_peak_accel;
    }
}

bool
terrain_on_hump(void)
{
    return (STATE_IN_HUMP == g_state) || (STATE_IN_HUMP == g_side_state);
}

void
terrain_last_side_hump(side_hump_event_t *p_ev)
{
    if (NULL != p_ev)
    {
        *p_ev = g_side;
    }
}
