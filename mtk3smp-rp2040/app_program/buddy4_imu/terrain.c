/**
 * @file terrain.c
 * @brief Buddy 4: real-time hump and collision detection.
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
#define COLL_JOLT_MG             500     /* first guess; see terrain_max_jolt_mg */
#define SIDE_ENTER_CDEG          250     /* |roll| past 2.5 deg ... */
#define SIDE_ENTER_MM            20      /* ... for this much travel */
#define SIDE_EXIT_CDEG           150     /* back within 1.5 deg ... */
#define SIDE_EXIT_MM             15      /* ... for this much travel */
#define TRACK_MM                 114.0f  /* wheel track: lever for roll */
#define COLL_HOLDOFF_MM          100     /* one collision per 10 cm */

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
static int16_t g_hump_roll_max = 0;      /* largest |filtered roll| in a pitch hump */
static uint32_t g_pitch_dropped = 0u;    /* pitch humps dropped as one-sided */

static terrain_state_t g_state = STATE_LEVEL;
static int32_t g_below_since = -1;
static int32_t g_above_since = -1;
static hump_event_t g_hump;

static bool g_have_prev = false;
static int16_t g_prev_ax = 0;
static int16_t g_prev_ay = 0;
static int32_t g_last_coll_mm = -1000000;
static int16_t g_max_jolt = 0;
static int16_t g_min_pitch = 0;

/* sin of hundredths of a degree, Taylor to x^7 (no libm) */
static float
sin_cdeg(int32_t cdeg)
{
    float x = (float)cdeg * (3.14159265f / 18000.0f);
    float x2 = x * x;

    return x * (1.0f - (x2 / 6.0f) * (1.0f - (x2 / 20.0f) * (1.0f - (x2 / 42.0f))));
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
            /* Roll is right-side-down positive: raised right wheel = negative */
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
    bool report = (g_hump_roll_max < SIDE_ENTER_CDEG) && (STATE_LEVEL == g_side_state);

    g_hump.end_mm = dist_mm;
    g_hump.height_mm = (int16_t)(WHEELBASE_MM * sin_cdeg(-(int32_t)g_hump.plateau_cdeg) + 0.5f);
    if (report)
    {
        *p_hump = g_hump;
    }
    g_state = STATE_LEVEL;
    g_below_since = -1;
    g_above_since = -1;
    return report;
}

void
terrain_reset(int16_t rest_pitch_cdeg, int16_t rest_roll_cdeg)
{
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
    g_pitch_dropped = 0u;
    g_state = STATE_LEVEL;
    g_below_since = -1;
    g_above_since = -1;
    g_have_prev = false;
    g_last_coll_mm = -1000000;
    g_max_jolt = 0;
    g_min_pitch = 0;
}

uint32_t
terrain_update(int32_t dist_mm, const imu_attitude_t *p_att,
               const imu_raw_t *p_cal, bool steady,
               hump_event_t *p_hump, collision_event_t *p_coll)
{
    uint32_t events = 0u;
    int16_t fp;
    int16_t fr;
    int16_t mean;
    int32_t from_mm;
    int16_t jolt;

    if ((NULL == p_att) || (NULL == p_cal) || (NULL == p_hump) || (NULL == p_coll))
    {
        return 0u;
    }

    /* Causal moving averages of pitch and roll, relative to rest */
    fp = (int16_t)(filt_push(g_pitch_ring, &g_pitch_n, p_att->pitch_cdeg) - g_rest_pitch);
    fr = (int16_t)(filt_push(g_roll_ring, &g_roll_n, p_att->roll_cdeg) - g_rest_roll);

    /* ---- One-sided hump (roll) ---- */
    events |= side_update(dist_mm, fr);

    if (steady && (fp < g_min_pitch))
    {
        g_min_pitch = fp;
    }

    /* ---- Hump ---- */
    if (window_push(&g_pwin, dist_mm, fp, &mean, &from_mm) && (STATE_IN_HUMP == g_state)
        && (mean < g_hump.plateau_cdeg))
    {
        g_hump.plateau_cdeg = mean;
        g_hump.plateau_mm = from_mm;
    }

    if (STATE_LEVEL == g_state)
    {
        if (steady && (fp < HUMP_ENTER_CDEG))
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

        if (!steady)
        {
            /* Braking or a speed change: tilt is no longer trustworthy,
               so close the hump with what has been seen so far. */
            if (hump_finish(dist_mm, p_hump))
            {
                events |= TERRAIN_EVT_HUMP;
            }
            else
            {
                g_pitch_dropped++;
            }
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
                else
                {
                    g_pitch_dropped++;
                }
            }
        }
        else
        {
            g_above_since = -1;
        }
    }

    /* ---- Collision: a sharp jump in horizontal acceleration ---- */
    if (g_have_prev)
    {
        jolt = abs16((int16_t)(p_cal->ax - g_prev_ax));
        if (abs16((int16_t)(p_cal->ay - g_prev_ay)) > jolt)
        {
            jolt = abs16((int16_t)(p_cal->ay - g_prev_ay));
        }
        if (jolt > g_max_jolt)
        {
            g_max_jolt = jolt;
        }
        if ((jolt >= COLL_JOLT_MG) && ((dist_mm - g_last_coll_mm) >= COLL_HOLDOFF_MM))
        {
            p_coll->dist_mm = dist_mm;
            p_coll->jolt_mg = jolt;
            g_last_coll_mm = dist_mm;
            events |= TERRAIN_EVT_COLLISION;
        }
    }
    g_prev_ax = p_cal->ax;
    g_prev_ay = p_cal->ay;
    g_have_prev = true;

    return events;
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

int16_t
terrain_max_roll_cdeg(void)
{
    return g_max_roll;
}

uint32_t
terrain_pitch_humps_dropped(void)
{
    return g_pitch_dropped;
}

int16_t
terrain_max_jolt_mg(void)
{
    return g_max_jolt;
}

int16_t
terrain_min_pitch_cdeg(void)
{
    return g_min_pitch;
}
