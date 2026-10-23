/**
 * @file line.c
 * @brief Buddy 3: line following task.
 */

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>

#include "../common/faults.h"
#include "../buddy2_motion/motion_task.h"
#include "ir_sensor.h"
#include "line.h"

#define LINE_TASK_PRI        7u      /* below motion (5) and obstacle (6) */
#define LINE_STACK_SIZE      1024u
#define LINE_FLG_RUN         (1u << 0)

#define LINE_PERIOD_MS       20u     /* = the PI period */
/* Gains from the first mat runs (2026-10-23): 40 + 6/period with a 150
 * cap wound the trim up to -150, so the car turned ~70 deg/s, swept past
 * the line in one period and kept turning away from it. Now gentler, with
 * the trim cleared whenever the line changes side and the steering slewed. */
#define LINE_KP_MM_S         25      /* kick while off to one side (~25 deg/s) */
#define LINE_KI_MM_S         1       /* trim build-up per period off-side */
/* A 15 cm radius arc (course curve mats, assumed) at 100 mm/s needs the
 * wheels ~75 mm/s apart over the 114 mm track */
#define LINE_TRIM_MAX_MM_S   80
#define LINE_CORR_MAX_MM_S   120
#define LINE_SLEW_MM_S       15      /* max steering change per period */

#define LINE_LOST_MM         30      /* centre and right both white this far: lost */
#define SEARCH_MM_S          30      /* on-spot search turn speed */
#define SEARCH_FIRST_CDEG    6000    /* toward the last side, then ... */
#define SEARCH_SECOND_CDEG   12000   /* ... back past centre to the other */
#define SEARCH_TIMEOUT_MS    8000u

#define UM_PER_EDGE          328L
#define TRACK_MM             114L

static ID g_line_tsk = 0;
static ID g_line_flg = 0;

static volatile int16_t g_base = 150;
static volatile int16_t g_max_mm = 450;
static volatile uint8_t g_end = LINE_END_NONE;
static volatile uint8_t g_stop_req = 0u;
static volatile uint32_t g_recoveries = 0u;

static line_trace_t g_trace[LINE_TRACE_LEN];
static volatile uint32_t g_trace_n = 0u;

static int32_t
dist_mm(void)
{
    enc_diag_t l, r;

    (void)motion_get_encoder_diag(&l, &r);
    return (((l.hw_count + r.hw_count) / 2) * UM_PER_EDGE) / 1000L;
}

/* Turn angle since (l0, r0), x0.01 deg, left positive */
static int32_t
turned_cdeg(int32_t l0, int32_t r0)
{
    enc_diag_t l, r;
    float half_diff;

    (void)motion_get_encoder_diag(&l, &r);
    half_diff = ((float)((r.hw_count - r0) - (l.hw_count - l0))) * 0.5f;
    return (int32_t)((half_diff * (float)UM_PER_EDGE * 36000.0f)
                     / (3.14159265f * (float)TRACK_MM * 1000.0f));
}

static void
trace(int32_t d, int32_t corr, int32_t trim, uint32_t bits, bool searching)
{
    line_trace_t *p_t;

    if (g_trace_n >= LINE_TRACE_LEN)
    {
        return;
    }
    p_t = &g_trace[g_trace_n];
    p_t->d_mm = (int16_t)d;
    p_t->corr = (int16_t)corr;
    p_t->trim = (int16_t)trim;
    p_t->bits = (uint8_t)bits;
    p_t->searching = searching ? 1u : 0u;
    g_trace_n++;
}

static bool
on_line(uint32_t bits)
{
    return 0u != (bits & (IR_BIT_CENTRE | IR_BIT_RIGHT));
}

/* Turn on the spot by up to |cdeg| (sign = direction) until either centre
 * or right sees the line. Returns true if found (car stopped). */
static bool
search_turn(int32_t cdeg)
{
    enc_diag_t l, r;
    int32_t l0;
    int32_t r0;
    uint32_t waited = 0u;
    uint32_t bits;
    bool found = false;

    (void)motion_get_encoder_diag(&l, &r);
    l0 = l.hw_count;
    r0 = r.hw_count;

    if (cdeg > 0)
    {
        (void)motion_set_velocity(-SEARCH_MM_S, SEARCH_MM_S);
    }
    else
    {
        (void)motion_set_velocity(SEARCH_MM_S, -SEARCH_MM_S);
    }

    while ((waited < SEARCH_TIMEOUT_MS) && (0u == g_stop_req)
           && (0u == (fault_get() & FAULT_MOTION_MASK)))
    {
        bits = ir_read();
        if (on_line(bits))
        {
            found = true;
            break;
        }
        if (((cdeg > 0) && (turned_cdeg(l0, r0) >= cdeg))
            || ((cdeg < 0) && (turned_cdeg(l0, r0) <= cdeg)))
        {
            break;
        }
        tk_dly_tsk(4u);
        waited += 4u;
        if (0u == (waited % LINE_PERIOD_MS))
        {
            trace(dist_mm(), 0, 0, bits, true);
        }
    }
    motion_stop();
    tk_dly_tsk(150u);
    return found;
}

static void
line_task_main(INT stacd, void *exinf)
{
    UINT flgptn;
    uint32_t bits;
    int32_t trim;
    int32_t corr;
    int32_t off;
    int32_t last_side;
    int32_t d;
    int32_t base;
    int32_t none_since;
    int32_t target;
    bool c_on;
    bool r_on;

    (void)stacd;
    (void)exinf;

    while (1)
    {
        (void)tk_wai_flg(g_line_flg, LINE_FLG_RUN, TWF_ORW | TWF_CLR, &flgptn, TMO_FEVR);

        trim = 0;
        corr = 0;
        last_side = 1;
        none_since = -1;
        base = g_base;
        g_end = LINE_END_NONE;

        while (LINE_END_NONE == g_end)
        {
            if ((0u != g_stop_req) || (0u != (fault_get() & FAULT_MOTION_MASK)))
            {
                g_end = LINE_END_STOPPED;
                break;
            }
            d = dist_mm();
            if (d >= g_max_mm)
            {
                g_end = LINE_END_DISTANCE;
                break;
            }

            bits = ir_read();
            c_on = (0u != (bits & IR_BIT_CENTRE));
            r_on = (0u != (bits & IR_BIT_RIGHT));

            /* Centre follows the line, right guards it (the two spots are
             * too far apart to both sit on the 18 mm line):
             *   C R
             *   1 0  on track
             *   1 1  line moving right (car drifting left)  -> steer right
             *   0 1  line well to the right                 -> steer harder
             *   0 0  line to the left of centre (drifting right), or lost:
             *        steer left for up to LINE_LOST_MM, then treat as lost */
            if (c_on && (!r_on))
            {
                off = 0;
                none_since = -1;
            }
            else if (c_on && r_on)
            {
                off = -1;
                last_side = -1;
                none_since = -1;
            }
            else if (r_on)
            {
                off = -2;
                last_side = -1;
                none_since = -1;
            }
            else
            {
                if (none_since < 0)
                {
                    none_since = d;
                    /* Came off the left edge of the line unless the right
                     * guard saw it last */
                    if (last_side >= 0)
                    {
                        last_side = 1;
                    }
                }
                off = (last_side > 0) ? 1 : -2;
                if ((d - none_since) > LINE_LOST_MM)
                {
                    /* Lost: stop, then look on the spot only (raised mats) */
                    motion_stop();
                    tk_dly_tsk(200u);
                    trim = 0;
                    trace(d, 0, trim, bits, true);
                    if (search_turn(last_side * SEARCH_FIRST_CDEG)
                        || search_turn(-last_side * SEARCH_SECOND_CDEG))
                    {
                        g_recoveries++;
                        trim = 0;
                        corr = 0;
                        none_since = -1;
                        continue;
                    }
                    if (0u != g_stop_req)
                    {
                        g_end = LINE_END_STOPPED;
                    }
                    else
                    {
                        fault_set(FAULT_LINE_LOST);
                        g_end = LINE_END_LOST;
                    }
                    break;
                }
            }
            if ((0 == off) && (last_side < 0))
            {
                last_side = 1;           /* back on track: reset to default */
            }

            /* Anti-windup: the line changed side, so the held trim now
             * points the wrong way */
            if (((off > 0) && (trim < 0)) || ((off < 0) && (trim > 0)))
            {
                trim = 0;
            }
            trim += off * LINE_KI_MM_S;
            trim = (trim > LINE_TRIM_MAX_MM_S) ? LINE_TRIM_MAX_MM_S
                 : ((trim < -LINE_TRIM_MAX_MM_S) ? -LINE_TRIM_MAX_MM_S : trim);
            target = trim + (off * LINE_KP_MM_S);
            target = (target > LINE_CORR_MAX_MM_S) ? LINE_CORR_MAX_MM_S
                   : ((target < -LINE_CORR_MAX_MM_S) ? -LINE_CORR_MAX_MM_S : target);
            /* Slew-limit the steering so it never jerks */
            corr += (target > corr + LINE_SLEW_MM_S) ? LINE_SLEW_MM_S
                  : ((target < corr - LINE_SLEW_MM_S) ? -LINE_SLEW_MM_S : (target - corr));

            (void)motion_set_velocity((int16_t)(base - corr), (int16_t)(base + corr));
            trace(d, corr, trim, bits, false);
            tk_dly_tsk(LINE_PERIOD_MS);
        }

        motion_stop();
        g_stop_req = 0u;
    }
}

ER
line_task_create(void)
{
    T_CFLG cflg;
    T_CTSK ctsk;
    ER er;

    if (g_line_tsk > 0)
    {
        return E_OBJ;
    }
    ir_init();

    cflg.flgatr = TA_TFIFO | TA_WSGL;
    cflg.iflgptn = 0u;
    g_line_flg = tk_cre_flg(&cflg);
    if (g_line_flg <= 0)
    {
        return g_line_flg;
    }

    ctsk.tskatr = TA_HLNG | TA_RNG3;
    ctsk.task = line_task_main;
    ctsk.itskpri = LINE_TASK_PRI;
    ctsk.stksz = LINE_STACK_SIZE;
    g_line_tsk = tk_cre_tsk(&ctsk);
    if (g_line_tsk <= 0)
    {
        er = g_line_tsk;
        g_line_tsk = 0;
        (void)tk_del_flg(g_line_flg);
        g_line_flg = 0;
        return er;
    }
    er = tk_sta_tsk(g_line_tsk, 0);
    if (E_OK != er)
    {
        (void)tk_del_tsk(g_line_tsk);
        g_line_tsk = 0;
        (void)tk_del_flg(g_line_flg);
        g_line_flg = 0;
    }
    return er;
}

ER
line_follow_start(int16_t base_mm_s, int16_t max_mm)
{
    if (g_line_flg <= 0)
    {
        return E_NOEXS;
    }
    g_base = base_mm_s;
    g_max_mm = max_mm;
    g_stop_req = 0u;
    g_trace_n = 0u;
    g_end = LINE_END_NONE;
    fault_clear(FAULT_LINE_LOST);
    motion_reset_odometry();
    return tk_set_flg(g_line_flg, LINE_FLG_RUN);
}

void
line_follow_stop(void)
{
    g_stop_req = 1u;
    motion_stop();
}

uint8_t
line_follow_end(void)
{
    return g_end;
}

uint32_t
line_recoveries(void)
{
    return g_recoveries;
}

uint32_t
line_trace_get(line_trace_t *p_out)
{
    uint32_t i;
    uint32_t n = g_trace_n;

    if (NULL == p_out)
    {
        return 0u;
    }
    for (i = 0u; i < n; i++)
    {
        p_out[i] = g_trace[i];
    }
    return n;
}
