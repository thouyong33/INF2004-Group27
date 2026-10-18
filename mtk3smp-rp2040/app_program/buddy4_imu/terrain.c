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
#define COLL_HOLDOFF_MM          100     /* one collision per 10 cm */

typedef enum {
    STATE_LEVEL = 0,
    STATE_IN_HUMP
} terrain_state_t;

static int16_t g_rest_pitch = 0;
static int16_t g_pitch_ring[FILT_N];
static uint32_t g_pitch_n = 0u;

static int32_t g_win_d[WIN_MAX];
static int16_t g_win_p[WIN_MAX];
static uint32_t g_win_head = 0u;         /* index of the oldest entry */
static uint32_t g_win_count = 0u;

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

/* Push one (distance, pitch) pair and drop entries older than 30 mm.
 * Returns true and the window mean once the window spans enough travel. */
static bool
window_push(int32_t dist_mm, int16_t fp, int16_t *p_mean, int32_t *p_from_mm)
{
    uint32_t i;
    uint32_t idx;
    int32_t sum = 0;

    if (WIN_MAX == g_win_count)
    {
        g_win_head = (g_win_head + 1u) % WIN_MAX;
        g_win_count--;
    }
    idx = (g_win_head + g_win_count) % WIN_MAX;
    g_win_d[idx] = dist_mm;
    g_win_p[idx] = fp;
    g_win_count++;

    while ((g_win_count > 1u) && ((dist_mm - g_win_d[g_win_head]) > PLAT_MM))
    {
        g_win_head = (g_win_head + 1u) % WIN_MAX;
        g_win_count--;
    }

    if ((dist_mm - g_win_d[g_win_head]) < PLAT_MIN_SPAN_MM)
    {
        return false;
    }

    for (i = 0u; i < g_win_count; i++)
    {
        sum += g_win_p[(g_win_head + i) % WIN_MAX];
    }
    *p_mean = (int16_t)(sum / (int32_t)g_win_count);
    *p_from_mm = g_win_d[g_win_head];
    return true;
}

static void
hump_finish(int32_t dist_mm, hump_event_t *p_hump)
{
    g_hump.end_mm = dist_mm;
    g_hump.height_mm = (int16_t)(WHEELBASE_MM * sin_cdeg(-(int32_t)g_hump.plateau_cdeg) + 0.5f);
    *p_hump = g_hump;
    g_state = STATE_LEVEL;
    g_below_since = -1;
    g_above_since = -1;
}

void
terrain_reset(int16_t rest_pitch_cdeg)
{
    g_rest_pitch = rest_pitch_cdeg;
    g_pitch_n = 0u;
    g_win_head = 0u;
    g_win_count = 0u;
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
    uint32_t i;
    uint32_t n;
    int32_t sum = 0;
    int16_t fp;
    int16_t mean;
    int32_t from_mm;
    int16_t jolt;

    if ((NULL == p_att) || (NULL == p_cal) || (NULL == p_hump) || (NULL == p_coll))
    {
        return 0u;
    }

    /* Causal moving average of pitch, relative to the rest reading */
    g_pitch_ring[g_pitch_n % FILT_N] = p_att->pitch_cdeg;
    g_pitch_n++;
    n = (g_pitch_n < FILT_N) ? g_pitch_n : FILT_N;
    for (i = 0u; i < n; i++)
    {
        sum += g_pitch_ring[i];
    }
    fp = (int16_t)((sum / (int32_t)n) - g_rest_pitch);

    if (steady && (fp < g_min_pitch))
    {
        g_min_pitch = fp;
    }

    /* ---- Hump ---- */
    if (window_push(dist_mm, fp, &mean, &from_mm) && (STATE_IN_HUMP == g_state)
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
        if (!steady)
        {
            /* Braking or a speed change: tilt is no longer trustworthy,
               so close the hump with what has been seen so far. */
            hump_finish(dist_mm, p_hump);
            events |= TERRAIN_EVT_HUMP;
        }
        else if (fp > HUMP_EXIT_CDEG)
        {
            if (g_above_since < 0)
            {
                g_above_since = dist_mm;
            }
            if ((dist_mm - g_above_since) >= HUMP_HOLD_MM)
            {
                hump_finish(g_above_since, p_hump);
                events |= TERRAIN_EVT_HUMP;
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
