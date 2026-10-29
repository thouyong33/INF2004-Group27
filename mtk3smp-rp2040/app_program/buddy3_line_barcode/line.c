/**
 * @file line.c
 * @brief Buddy 3: line following task.
 */

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>

#include "../common/faults.h"
#include "../common/ctrl_events.h"
#include "../buddy2_motion/motion_task.h"
#include "ir_sensor.h"
#include "line.h"

#define LINE_TASK_PRI        7u      /* below motion (5) and obstacle (6) */
#define LINE_STACK_SIZE      1024u
#define LINE_FLG_RUN         (1u << 0)

#define LINE_PERIOD_MS       20u     /* = the PI period */
/* Right-edge following gains. The first runs (40 + 6/period trim, then
 * 25 + 1/period) swung or weaved; now the steering starts at LINE_KP and
 * grows by LINE_KI per period while it stays on one side (curves), capped
 * at LINE_SIDE_MAX, and restarts whenever the side changes. A 15 cm radius
 * curve at 100 mm/s needs ~75 mm/s between the wheels, i.e. ~38 each way. */
#define LINE_KP_MM_S         20
#define LINE_KI_MM_S         2
#define LINE_TRIM_MAX_MM_S   60      /* one side's steering cap */
#define LINE_CORR_MAX_MM_S   120     /* the hard-right case (line under R) */
#define LINE_SLEW_MM_S       15      /* max steering change per period */
#define LINE_JUNCTION_MM     30      /* drive straight this far over an arm */
/* C and R both black this far before it counts as a junction: a cross arm
 * keeps both black ~18 mm; a diagonal glance across the line's edge only a
 * few mm (false junctions on the plain straight, 26 Oct) */
#define LINE_JUNC_CONFIRM_MM 6
/* ...and the right IR's black stretch must end within this: an 18 mm arm
 * keeps it black for ~6-35 mm, a black ramp under the right IR (the small
 * test ramp, 27 Oct) for 100 mm or more. The junction is reported when the
 * black ends, from where it began. */
#define LINE_JUNC_MAX_MM     35
/* Line under the right IR alone (centre white): the car overshot the line
 * to the left, a hard right brings it back. This is the weave's normal
 * recovery (without it, ctrl_course_v14 lost the line and searched on
 * nearly every run). A black ramp beside the line reads the same, so the
 * hard right only lasts while R's black stretch is arm-sized and stops
 * once the heading is LINE_R_DEV_MAX_CDEG right of the line's direction
 * (v13: it turned 13 deg right onto the ramp). Past that it counts as
 * "centre on the white" or straight. */
#define LINE_R_ONLY_MAX_MM   35
#define LINE_R_DEV_MAX_CDEG  500
/* centre white this far (steering left): lost */
#define LINE_LOST_MM         40
#define SEARCH_MM_S          30      /* on-spot search turn speed */
/* Searches sweep a window around the line's direction (below), first
 * toward the side the line was last seen, so a perpendicular junction arm
 * is never taken (ramp runs 2026-10-23/24 picked up a cross arm) */
#define SEARCH_HALF_CDEG     6000    /* line lost while following */
#define SEARCH_RAMP_CDEG     4500    /* line not under C/R right after a ramp */
#define SEARCH_TIMEOUT_MS    8000u
/* trace a search this often (keeps the trace short) */
#define SEARCH_TRACE_MS      200u
/* Ramp hold (line_set_hold): tilted on the one-sided ramp the IR reads
 * false black (ramp runs 2026-10-24), so the car drives straight on the
 * encoders and only looks at the line again this far past the ramp */
#define HOLD_AFTER_MM        50
/* steering mm/s = heading x0.01 deg / 20 */
#define HOLD_GAIN_DIV        20
#define HOLD_CORR_MAX_MM_S   30
/* Line direction: the encoder heading averaged over ~LINE_DIR_MM of
 * following, so the weave and a knock just before the ramp (the IR
 * bracket hits it first) do not set the direction the ramp is held on */
#define LINE_DIR_MM          100

#define UM_PER_EDGE          328L
#define TRACK_MM             114L

static ID g_line_tsk = 0;
static ID g_line_flg = 0;

static volatile int16_t g_base = 150;
static volatile int16_t g_max_mm = 450;
static volatile uint8_t g_end = LINE_END_NONE;
static volatile uint8_t g_stop_req = 0u;
static volatile uint32_t g_recoveries = 0u;
static volatile uint8_t g_hold = 0u;
static volatile int32_t g_line_dir = 0;     /* published for the controller */

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
trace(int32_t d, int32_t corr, int32_t trim, uint32_t bits, uint8_t mode,
      int32_t off)
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
    p_t->searching = mode;
    p_t->heading_cdeg = (int16_t)turned_cdeg(0, 0);
    p_t->off = (int8_t)off;
    g_trace_n++;
}

/* Found the line: the centre IR only (the right IR also sees a black ramp
 * beside the line, see the steering notes below) */
static bool
on_line(uint32_t bits)
{
    return 0u != (bits & IR_BIT_CENTRE);
}

/* Heading since following started (odometry reset), x0.01 deg, left + */
static int32_t
heading_cdeg(void)
{
    return turned_cdeg(0, 0);
}

/* Turn on the spot until either centre or right sees the line, or the
 * heading reaches to_cdeg (absolute, since the start of following).
 * Returns true if found (car stopped). */
static bool
search_turn(int32_t to_cdeg)
{
    enc_diag_t l, r;
    int32_t l0;
    int32_t r0;
    int32_t cdeg;
    uint32_t waited = 0u;
    uint32_t bits;
    bool b_found = false;

    (void)motion_get_encoder_diag(&l, &r);
    l0 = l.hw_count;
    r0 = r.hw_count;
    cdeg = to_cdeg - heading_cdeg();
    if (0 == cdeg)
    {
        return on_line(ir_read());
    }

    if (cdeg > 0)
    {
        (void)motion_set_velocity(-SEARCH_MM_S, SEARCH_MM_S);
    }
    else
    {
        (void)motion_set_velocity(SEARCH_MM_S, -SEARCH_MM_S);
    }

    /* ...and stop as soon as the ramp hold starts: a search that began
     * at the ramp's foot kept turning with a wheel on it until the hump
     * mask limit raised a stall (ctrl_course_v15 run 2) */
    while ((waited < SEARCH_TIMEOUT_MS) && (0u == g_stop_req)
           && (0u == g_hold)
           && (0u == (fault_get() & FAULT_MOTION_MASK)))
    {
        bits = ir_read();
        if (on_line(bits))
        {
            b_found = true;
            break;
        }
        if (((cdeg > 0) && (turned_cdeg(l0, r0) >= cdeg))
            || ((cdeg < 0) && (turned_cdeg(l0, r0) <= cdeg)))
        {
            break;
        }
        tk_dly_tsk(4u);
        waited += 4u;
        if (0u == (waited % SEARCH_TRACE_MS))
        {
            trace(dist_mm(), 0, 0, bits, LINE_TR_SEARCH,
                  LINE_TR_NO_OFF);
        }
    }
    motion_stop();
    tk_dly_tsk(150u);
    return b_found;
}

/* Sweep dir +/- half on the spot, first toward side (+1 left, -1 right) */
static bool
search_window(int32_t dir, int32_t half, int32_t side)
{
    return search_turn(dir + (side * half))
           || ((0u == g_hold) && search_turn(dir - (side * half)));
}

static void
line_task_main(INT stacd, void *exinf)
{
    UINT flgptn;
    uint32_t bits;
    int32_t corr;
    int32_t off;
    int32_t prev_off;
    int32_t run;
    int32_t last_side;
    int32_t d;
    int32_t base;
    int32_t white_since;
    int32_t straight_until;
    int32_t both_since;
    int32_t r_since;
    bool b_junc_ok;
    int32_t target;
    bool b_c_on;
    bool b_r_on;
    bool b_holding;
    int32_t hold_until;
    bool b_after_hold;
    int32_t hold_dir = 0;
    int32_t line_dir;
    int32_t prev_d;
    int32_t step;

    (void)stacd;
    (void)exinf;

    while (1)
    {
        (void)tk_wai_flg(g_line_flg, LINE_FLG_RUN, TWF_ORW | TWF_CLR, &flgptn,
                         TMO_FEVR);

        corr = 0;
        prev_off = 0;
        run = 0;
        last_side = 1;
        white_since = -1;
        straight_until = -1;
        both_since = -1;
        r_since = -1;
        b_junc_ok = false;
        b_holding = false;
        b_after_hold = false;
        hold_until = -1;
        line_dir = 0;
        g_line_dir = 0;
        prev_d = 0;
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
            if ((g_max_mm > 0) && (d >= g_max_mm))
            {
                g_end = LINE_END_DISTANCE;
                break;
            }

            bits = ir_read();
            b_c_on = (0u != (bits & IR_BIT_CENTRE));
            b_r_on = (0u != (bits & IR_BIT_RIGHT));

            /* Junction report: when the right IR's black stretch ends, if
             * it was arm-sized and C and R were black together for
             * LINE_JUNC_CONFIRM_MM within it */
            if (b_r_on)
            {
                if (r_since < 0)
                {
                    r_since = d;
                    b_junc_ok = false;
                }
            }
            else if (r_since >= 0)
            {
                if (b_junc_ok && ((d - r_since) <= LINE_JUNC_MAX_MM))
                {
                    ctrl_evt_post(EVT_JUNCTION, 0u, (int16_t)r_since);
                }
                r_since = -1;
                b_junc_ok = false;
            }

            /* Ramp hold: drive along the line's direction on the encoders,
             * ignore the IR */
            if (0u != g_hold)
            {
                if (!b_holding)
                {
                    b_holding = true;
                    hold_dir = line_dir;
                }
                hold_until = d + HOLD_AFTER_MM;
            }
            if (b_holding && (d >= hold_until))
            {
                /* Past the ramp: pick the line up again from scratch */
                b_holding = false;
                b_after_hold = true;
                run = 0;
                prev_off = 0;
                white_since = -1;
                last_side = 1;
            }
            if (b_holding)
            {
                target = (hold_dir - heading_cdeg()) / HOLD_GAIN_DIV;
                target = (target > HOLD_CORR_MAX_MM_S) ? HOLD_CORR_MAX_MM_S
                       : ((target < -HOLD_CORR_MAX_MM_S) ? -HOLD_CORR_MAX_MM_S
                                                          : target);
                corr += (target > corr + LINE_SLEW_MM_S) ? LINE_SLEW_MM_S
                      : ((target < corr - LINE_SLEW_MM_S) ? -LINE_SLEW_MM_S
                                                          : (target - corr));
                (void)motion_set_velocity((int16_t)(base - corr),
                                          (int16_t)(base + corr));
                prev_d = d;
                trace(d, corr, 0, bits, LINE_TR_HOLD, LINE_TR_NO_OFF);
                tk_dly_tsk(LINE_PERIOD_MS);
                continue;
            }

            /* Just off the ramp and the line is not under C or R: search
             * on the spot now, only within +/-45 deg of the line's
             * direction (steering left blind could reach a junction arm) */
            if (b_after_hold)
            {
                b_after_hold = false;
                if (!b_c_on)
                {
                    motion_stop();
                    tk_dly_tsk(200u);
                    trace(d, 0, 0, bits, LINE_TR_SEARCH,
                  LINE_TR_NO_OFF);
                    if (search_window(hold_dir, SEARCH_RAMP_CDEG, 1)
                        || (0u != g_hold))
                    {
                        g_recoveries++;
                        corr = 0;
                        prev_d = dist_mm();
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

            /* Line direction: average the heading over the distance
             * followed (not while holding or searching) */
            step = d - prev_d;
            step = (step < 0) ? 0 : ((step > LINE_DIR_MM) ? LINE_DIR_MM : step);
            line_dir += ((heading_cdeg() - line_dir) * step) / LINE_DIR_MM;
            g_line_dir = line_dir;
            prev_d = d;

            /* Right-edge following (mat runs 2026-10-23: with the centre on
             * the line and the right spot 20 mm away, "neither black" could
             * mean the line was left of centre OR between the two spots, and
             * the guess made the car weave). The centre rides the line's
             * right edge, so each reading has one meaning:
             *   C R
             *   1 0  centre on the line   -> steer right, toward the edge
             *   0 0  centre on the white  -> steer left, toward the edge
             *   0 1  line under the right spot (car overshot left) -> hard
             *        right, limited by LINE_R_ONLY_MAX_MM and
             *        LINE_R_DEV_MAX_CDEG (a black ramp beside the line)
             *   1 1  black under both: a junction arm (the right spot is
             *        20 mm out over white on a plain line) -> straight on,
             *        unless R has been black longer than an arm (a ramp
             *        beside the line) -> as 1 0 */
            if (d < straight_until)
            {
                off = 0;
                white_since = -1;
            }
            else if (b_c_on && b_r_on && ((d - r_since) <= LINE_JUNC_MAX_MM))
            {
                /* Both black: hold straight; once it has lasted
                 * LINE_JUNC_CONFIRM_MM it may be a junction arm (reported
                 * above once R's black ends, if it was arm-sized) */
                if (both_since < 0)
                {
                    both_since = d;
                }
                if ((d - both_since) >= LINE_JUNC_CONFIRM_MM)
                {
                    b_junc_ok = true;      /* reported when R's black ends */
                    straight_until = d + LINE_JUNCTION_MM;
                    both_since = -1;
                }
                off = 0;
                white_since = -1;
            }
            else if (b_c_on)
            {
                off = -1;
                white_since = -1;
            }
            else if (b_r_on && ((d - r_since) <= LINE_R_ONLY_MAX_MM))
            {
                off = (heading_cdeg() > (line_dir - LINE_R_DEV_MAX_CDEG))
                      ? -2 : 0;
                last_side = -1;
                white_since = -1;
            }
            else
            {
                off = 1;
                if (white_since < 0)
                {
                    white_since = d;
                }
                if ((d - white_since) > LINE_LOST_MM)
                {
                    /* Lost: stop, then look on the spot only (raised mats) */
                    motion_stop();
                    tk_dly_tsk(200u);
                    trace(d, 0, 0, bits, LINE_TR_SEARCH,
                  LINE_TR_NO_OFF);
                    if (search_window(line_dir, SEARCH_HALF_CDEG, last_side)
                        || (0u != g_hold))
                    {
                        g_recoveries++;
                        prev_d = dist_mm();
                        corr = 0;
                        run = 0;
                        prev_off = 0;
                        white_since = -1;
                        last_side = 1;
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
            if (!(b_c_on && b_r_on))
            {
                both_since = -1;
            }
            if (off >= 0)
            {
                last_side = 1;           /* default: the line is to the left */
            }

            /* Steering grows the longer it stays on one side (for curves)
             * and restarts small whenever the side changes, so it can never
             * keep pushing the wrong way */
            if (off != prev_off)
            {
                run = 0;
            }
            prev_off = off;
            if (0 == off)
            {
                target = 0;
            }
            else
            {
                run++;
                target = LINE_KP_MM_S + (run * LINE_KI_MM_S);
                if (target > LINE_TRIM_MAX_MM_S)
                {
                    target = LINE_TRIM_MAX_MM_S;
                }
                target = (off < 0) ? -(target * ((off < -1) ? 2 : 1))
                                   : target;
                target = (target < -LINE_CORR_MAX_MM_S) ? -LINE_CORR_MAX_MM_S
                                                        : target;
            }
            /* Slew-limit the steering so it never jerks */
            corr += (target > corr + LINE_SLEW_MM_S) ? LINE_SLEW_MM_S
                  : ((target < corr - LINE_SLEW_MM_S) ? -LINE_SLEW_MM_S
                                                      : (target - corr));

            (void)motion_set_velocity((int16_t)(base - corr),
                                      (int16_t)(base + corr));
            trace(d, corr, run, bits, LINE_TR_FOLLOW, off);
            tk_dly_tsk(LINE_PERIOD_MS);
        }

        motion_stop();
        g_stop_req = 0u;
        ctrl_evt_post(EVT_LINE_END, g_end, (int16_t)dist_mm());
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
    g_hold = 0u;
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

int32_t
line_direction_cdeg(void)
{
    return g_line_dir;
}

void
line_set_hold(bool b_hold)
{
    g_hold = b_hold ? 1u : 0u;
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
