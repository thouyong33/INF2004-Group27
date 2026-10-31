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
/* Right-edge following by heading (29 Oct). Steering straight from the
 * IR reading (a wheel speed difference starting at 20 mm/s and growing 2
 * per period on one side) turned back late at every side change: each
 * swing grew (-23 then +13..+20 deg, ctrl_course_v25) and the car left
 * the mat; the old 40 mm lost-line stop had been damping it. Now each
 * reading picks a target heading just to one side of the line's direction
 * (line_dir, the heading averaged over LINE_DIR_MM): C black -> right of
 * it, C white -> left, by LINE_BIAS_CDEG, growing LINE_BIAS_GROW_CDEG a
 * period while on the same side (curves) up to LINE_BIAS_MAX_CDEG. The
 * wheel difference is the heading error / LINE_HDG_GAIN_DIV: at a side
 * change the error flips at once, so the car turns back at once. */
#define LINE_BIAS_CDEG       200
#define LINE_BIAS_GROW_CDEG  8
#define LINE_BIAS_MAX_CDEG   4500
#define LINE_HDG_GAIN_DIV    15      /* mm/s per 0.15 deg of heading error */
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
/* ...and is only reported once C has stayed black this far past the
 * end of R's black. At a cross C stays on the line 25 mm or more; on a
 * right curve the line sweeps under C and R together (it looks like
 * an arm) and then leaves C within ~7 mm (v34, 30 Oct): that is a
 * curve, so the straight hold is dropped and no junction is sent */
#define LINE_JUNC_HOLD_C_MM  10
/* Where the line went when both IRs read white: right of C if R was
 * the last to see black (R alone, or C and R together) and C has not
 * since been black alone this far; otherwise left. The weave only
 * leaves the line to the left (v34: after a right curve the car took
 * the white for "line to the left" and turned 90 deg left). v38 left
 * out C and R together (a mat seam reads so too), but every right
 * curve on the circuit enters that way, with no R-alone reading after
 * the right IR was retrimmed (v38 runs, 30 Oct): all turned left. */
#define LINE_RIGHT_CLEAR_MM  10
/* ...unless the left IR also saw black within this of C and R both
 * black: then it was a cross (both arms) or a seam across the mat,
 * not a right curve (the left IR, 65 mm left, never sees a right
 * curve). v41 runs 7-8 (30 Oct, new circuit): a cross where C left
 * the line early was taken for a right curve and the car turned off
 * the mat; the left IR logged its bar at the same place both times. */
#define LINE_CROSS_WINDOW_MM 40
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
/* Centre white this far (steering left): lost. 40 mm was shorter than a
 * swing under the old steering (ctrl_course_v23); with the smaller heading
 * weave (v29) a swing back from the right can take exactly 60 mm of white
 * and tripped a needless search 1-2 times a run, so 100 mm. */
#define LINE_LOST_MM         100
/* Curves (29 Oct). The steering above keeps the car beside the line's
 * direction averaged over LINE_DIR_MM and widens its aim by only
 * LINE_BIAS_GROW_CDEG a period: on the circuit's curve mat it turned
 * ~5 deg per 100 mm, lost the line, and after the search had found
 * it 20 deg left it steered back to the old straight (c run, v32).
 * A curve now adds a feed-forward turn (curve_ff, + left) that grows
 * LINE_CURVE_GROW_MM_S a period while the line is on the curve's
 * inside and decays LINE_CURVE_DECAY_MM_S a period otherwise, so it
 * settles where the two balance:
 *   left curve: starts after LINE_CURVE_WHITE_MM of centre white
 *     (a straight's weave is 18-25 mm); grows while C is white.
 *   right curve: starts when the line is still under R alone after
 *     the car has turned LINE_R_DEV_MAX_CDEG right (that cap used to
 *     send it straight on, off the curve); grows while C or R alone
 *     is black. A long C-black start (v33) also fired on straights.
 * While it is on, the reference direction follows the car and the
 * bias stays at LINE_BIAS_CDEG (v33 added the growing bias to the
 * curve turn and overturned right), barcode read mode and junction
 * arms are ignored (on a left curve the left IR crosses the line and
 * started a "read" that cancelled the curve, v33 runs 1 and 2).
 * LINE_CURVE_MAX_MM_S a side is a ~14 cm radius at 100 mm/s.
 * v35 circuit runs (30 Oct): leaving a right curve the turn took ~40 mm
 * to fade (decay 2 a period), the car turned past the straight, C saw
 * 45 mm of white and that started a left curve: a weave that grew
 * with every swing (run 3). A straight's weave also once held white
 * for exactly 40 mm and turned the car 150 deg left (run 2). So the
 * turn now fades in ~13 mm and a left curve needs 60 mm of white. */
#define LINE_CURVE_WHITE_MM  60
#define LINE_CURVE_GROW_MM_S 2
#define LINE_CURVE_DECAY_MM_S 6
#define LINE_CURVE_MAX_MM_S  40
/* On a left curve the centre is white while the car turns toward the
 * line, so "lost" is this much turning, or distance, without finding
 * it (not LINE_LOST_MM) */
#define LINE_CURVE_LOST_CDEG 12000
#define LINE_CURVE_LOST_MM   400
/* Blind right turn: both IRs white with the line known to be right (a
 * right curve's inner edge). v39 ran ~30 mm almost straight, then
 * stacked the curve turn on a hard right (66 mm/s, ~8.5 cm radius)
 * against ~15 cm corners: it spiralled inside the top-left corner,
 * the IRs never reaching the line (v39 runs 1, 2, 4). Now the turn
 * starts at once at LINE_BLIND_RIGHT_START_MM_S, the single bias is
 * used, and curve_ff is capped at LINE_BLIND_RIGHT_MAX_MM_S: ~40 mm/s
 * in all, a ~14 cm radius. Past LINE_BLIND_RIGHT_LOST_CDEG of turning
 * without the line it stops and searches, left first. */
#define LINE_BLIND_RIGHT_START_MM_S 14
#define LINE_BLIND_RIGHT_MAX_MM_S   27
#define LINE_BLIND_RIGHT_LOST_CDEG  9000
/* Just after a curve (v36 circuit runs 4 and 5, 30 Oct): the car came
 * back to the line slowly (bias growing LINE_BIAS_GROW_CDEG a period),
 * 60 mm of white built up and opened a small opposite curve, which
 * overshot and opened the next one: a +/-13 deg weave every ~250 mm
 * on the straight. For LINE_AFTER_CURVE_MM after a curve ends the bias
 * grows LINE_BIAS_GROW_AFTER_CDEG a period instead (a firm return to
 * the edge), and a new curve needs LINE_AFTER_CURVE_WHITE_MM of white
 * (still under LINE_LOST_MM) or the line under R alone does not
 * start one at all. */
#define LINE_AFTER_CURVE_MM  100
#define LINE_AFTER_CURVE_WHITE_MM 90
#define LINE_BIAS_GROW_AFTER_CDEG 30
/* A search that finds the line further than this from the line's
 * direction found a curve: carry on turning that way (curve_ff at
 * half) from the found heading, not back to the old straight */
#define SEARCH_CURVE_CDEG    1000
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
/* Barcode read mode (line_set_straight): the left IR is ~130 mm ahead of
 * the axle, so the weave swings it sideways (+/-18 mm at +/-8 deg) and it
 * left the bars (A4 reads with 19-53 mm "spaces", 29 Oct). A blind
 * straight hold fixed the heading but kept whatever sideways offset the
 * car had at the first bar (v27/v28). While a read is in progress the car
 * now keeps following the edge, with a fixed +/-LINE_READ_BIAS_CDEG aim
 * (no growth): the centre IR stays on the edge and the heading within
 * ~+/-2 deg. Never longer than this: */
#define STRAIGHT_MAX_MM      400
/* ...and read mode ends once C has been white this far: a code is
 * read with C on the line's edge, and off the line the fixed aim only
 * kept the car from steering back (v36 runs 1 and 2: ~50 mm held
 * straight just as the line curved away) */
#define READ_WHITE_MAX_MM    30   /* 20 dropped a read at a curve exit (31 Oct) */
#define LINE_READ_BIAS_CDEG  100
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
static volatile uint8_t g_straight = 0u;
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

    /* rolling: a continuous run keeps its last LINE_TRACE_LEN periods,
       where it failed (v35 runs failed past the first 830 mm) */
    p_t = &g_trace[g_trace_n % LINE_TRACE_LEN];
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
    int32_t bias;
    int32_t lim;
    bool b_c_on;
    bool b_r_on;
    bool b_holding;
    bool b_straight;
    bool b_reading;
    int32_t str_start;
    int32_t hold_until;
    bool b_after_hold;
    int32_t hold_dir = 0;
    int32_t line_dir;
    int32_t prev_d;
    int32_t step;
    int32_t curve_ff;        /* curve feed-forward, mm/s, + left */
    int32_t white_hdg = 0;   /* heading when the centre went white */
    int32_t found;
    bool b_lost;
    bool b_blind_right;      /* both white, the line to the right */
    int32_t left_d;          /* where the left IR last saw black */
    int32_t cr_d;            /* where C and R were last both black */
    bool b_line_right;       /* white: the line is right of C */
    int32_t c_only_since;    /* C black, R white, since */
    int32_t junc_pending;    /* junction waiting for C to confirm */
    int32_t junc_until = 0;
    int32_t curve_end;       /* where the last curve ended */
    int32_t ff_prev;
    bool b_after_curve;
    int32_t run_grow = LINE_BIAS_GROW_CDEG;   /* this side's growth */

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
        b_straight = false;
        b_reading = false;
        str_start = 0;
        b_holding = false;
        b_after_hold = false;
        hold_until = -1;
        line_dir = 0;
        g_line_dir = 0;
        prev_d = 0;
        curve_ff = 0;
        b_line_right = false;
        c_only_since = -1;
        left_d = -10000;
        cr_d = -10000;
        junc_pending = -1;
        curve_end = -10000;
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
            b_blind_right = false;
            ff_prev = curve_ff;
            if (0u != (bits & IR_BIT_LEFT))
            {
                left_d = d;
            }
            if (b_c_on && b_r_on)
            {
                cr_d = d;
            }
            b_after_curve = (0 == curve_ff)
                            && ((d - curve_end) < LINE_AFTER_CURVE_MM);

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
                    junc_pending = r_since;   /* see LINE_JUNC_HOLD_C_MM */
                    junc_until = d + LINE_JUNC_HOLD_C_MM;
                }
                r_since = -1;
                b_junc_ok = false;
            }
            if (junc_pending >= 0)
            {
                if (!b_c_on && ((left_d - cr_d) <= LINE_CROSS_WINDOW_MM)
                    && ((cr_d - left_d) <= LINE_CROSS_WINDOW_MM))
                {
                    /* C left early, but the left IR saw the other arm: a
                     * cross all the same */
                    ctrl_evt_post(EVT_JUNCTION, 0u, (int16_t)junc_pending);
                    junc_pending = -1;
                }
                else if (!b_c_on)
                {
                    junc_pending = -1;        /* a right curve */
                    straight_until = -1;
                }
                else if (d >= junc_until)
                {
                    ctrl_evt_post(EVT_JUNCTION, 0u, (int16_t)junc_pending);
                    junc_pending = -1;
                }
                else
                {
                    /* still confirming */
                }
            }

            /* Which side the line is on when both read white (see
             * LINE_RIGHT_CLEAR_MM) */
            if (b_r_on)
            {
                b_line_right = true;
                c_only_since = -1;
            }
            else if (b_c_on)
            {
                if (c_only_since < 0)
                {
                    c_only_since = d;
                }
                if ((d - c_only_since) >= LINE_RIGHT_CLEAR_MM)
                {
                    b_line_right = false;
                }
            }
            else
            {
                c_only_since = -1;
            }
            if (b_line_right && ((d - cr_d) <= LINE_CROSS_WINDOW_MM)
                && ((left_d - cr_d) <= LINE_CROSS_WINDOW_MM)
                && ((cr_d - left_d) <= LINE_CROSS_WINDOW_MM))
            {
                /* a cross or seam (see LINE_CROSS_WINDOW_MM) */
                b_line_right = false;
                curve_ff = (curve_ff < 0) ? 0 : curve_ff;
            }

            /* Barcode read mode (see LINE_READ_BIAS_CDEG), at most
             * STRAIGHT_MAX_MM per read */
            if ((0u != g_straight) && (!b_straight))
            {
                b_straight = true;
                str_start = d;
            }
            else if ((0u == g_straight) && b_straight)
            {
                b_straight = false;
                run = 0;
            }
            b_reading = b_straight && ((d - str_start) <= STRAIGHT_MAX_MM)
                        && (0 == curve_ff)
                        && ((white_since < 0)
                            || ((d - white_since) < READ_WHITE_MAX_MM));

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
                curve_ff = 0;
                b_line_right = false;
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
            if (0 != curve_ff)
            {
                line_dir = heading_cdeg();
            }
            else
            {
                line_dir += ((heading_cdeg() - line_dir) * step)
                            / LINE_DIR_MM;
            }
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
            if ((d < straight_until) && !b_c_on)
            {
                /* C left the line during the arm's straight hold: a
                 * right curve, not a cross (v40 run 2 held straight
                 * 12 mm with the line under R alone) */
                straight_until = -1;
            }
            if (d < straight_until)
            {
                off = 0;
                white_since = -1;
            }
            else if (b_c_on && b_r_on && ((d - r_since) <= LINE_JUNC_MAX_MM)
                     && (0 == curve_ff))
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
            else if (b_r_on && (((d - r_since) <= LINE_R_ONLY_MAX_MM)
                                || (curve_ff < 0)))
            {
                if (curve_ff > 0)
                {
                    curve_ff = 0;   /* overshot left: the curve is over */
                }
                else if ((0 == curve_ff) && !b_reading && !b_after_curve
                         && (heading_cdeg()
                             <= (line_dir - LINE_R_DEV_MAX_CDEG)))
                {
                    curve_ff = -LINE_CURVE_GROW_MM_S;   /* right curve */
                }
                else
                {
                    /* weave, or already on a right curve */
                }
                off = -2;
                last_side = -1;
                white_since = -1;
            }
            else
            {
                /* the line is left of C, or right of it (between C and
                 * R, or past R) after R saw it last */
                off = b_line_right ? -1 : 1;
                b_blind_right = b_line_right;
                if (b_line_right)
                {
                    last_side = -1;
                }
                if (white_since < 0)
                {
                    white_since = d;
                    white_hdg = heading_cdeg();
                }
                found = heading_cdeg() - white_hdg;
                found = (found < 0) ? -found : found;
                b_lost = b_blind_right
                    ? ((found > LINE_BLIND_RIGHT_LOST_CDEG)
                       || ((d - white_since) > LINE_CURVE_LOST_MM))
                    : ((0 != curve_ff)
                       ? ((found > LINE_CURVE_LOST_CDEG)
                          || ((d - white_since) > LINE_CURVE_LOST_MM))
                       : ((d - white_since) > LINE_LOST_MM));
                if (b_lost && b_blind_right)
                {
                    last_side = 1;   /* turned past it: look left first */
                }
                if (b_lost)
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
                        curve_ff = 0;
                        found = heading_cdeg() - line_dir;
                        if ((found > SEARCH_CURVE_CDEG)
                            || (found < -SEARCH_CURVE_CDEG))
                        {
                            curve_ff = (found > 0)
                                       ? (LINE_CURVE_MAX_MM_S / 2)
                                       : -(LINE_CURVE_MAX_MM_S / 2);
                            line_dir = heading_cdeg();
                            g_line_dir = line_dir;
                        }
                        run = 0;
                        prev_off = 0;
                        white_since = -1;
                        last_side = 1;
                        b_line_right = false;
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

            /* Target heading beside the line's direction (see LINE_BIAS):
             * the bias grows the longer it stays on one side (curves) and
             * restarts small whenever the side changes */
            if (off != prev_off)
            {
                run = 0;
                /* the rate is kept until the side changes: the end of
                 * the after-curve window mid-stretch dropped the bias
                 * at once and flipped the steering (v37 run 4) */
                run_grow = b_after_curve ? LINE_BIAS_GROW_AFTER_CDEG
                                         : LINE_BIAS_GROW_CDEG;
            }
            prev_off = off;
            if (0 == off)
            {
                bias = 0;
            }
            else if (b_reading)
            {
                bias = (off < 0) ? -(LINE_READ_BIAS_CDEG * ((off < -1) ? 2 : 1))
                                 : LINE_READ_BIAS_CDEG;
            }
            else if (0 != curve_ff)
            {
                bias = (off < 0) ? -(LINE_BIAS_CDEG * ((off < -1) ? 2 : 1))
                                 : LINE_BIAS_CDEG;
            }
            else
            {
                run++;
                bias = LINE_BIAS_CDEG + (run * run_grow);
                bias = (bias > LINE_BIAS_MAX_CDEG) ? LINE_BIAS_MAX_CDEG : bias;
                bias = (off < 0) ? -(bias * ((off < -1) ? 2 : 1)) : bias;
            }
            /* Curve feed-forward (see LINE_CURVE_WHITE_MM) */
            if (b_reading || (d < straight_until))
            {
                curve_ff = 0;          /* barcodes, arms: straights */
            }
            else if ((1 == off) && (white_since >= 0)
                     && ((curve_ff > 0)
                         || ((d - white_since)
                             >= (b_after_curve ? LINE_AFTER_CURVE_WHITE_MM
                                               : LINE_CURVE_WHITE_MM))))
            {
                curve_ff += LINE_CURVE_GROW_MM_S;
            }
            else if (b_blind_right && (white_since >= 0))
            {
                /* right curve: at once, see LINE_BLIND_RIGHT_*, even
                 * just after a curve: the right IR saw the line, so
                 * this is the S-bend's right half, not an overshoot
                 * (v40: held off 90 mm there, the car drove off it) */
                curve_ff = (curve_ff > -LINE_BLIND_RIGHT_START_MM_S)
                           ? -LINE_BLIND_RIGHT_START_MM_S
                           : (curve_ff - LINE_CURVE_GROW_MM_S);
                curve_ff = (curve_ff < -LINE_BLIND_RIGHT_MAX_MM_S)
                           ? -LINE_BLIND_RIGHT_MAX_MM_S : curve_ff;
            }
            else if ((off < 0) && (curve_ff < 0))
            {
                curve_ff -= LINE_CURVE_GROW_MM_S;
            }
            else if (curve_ff > 0)
            {
                curve_ff -= LINE_CURVE_DECAY_MM_S;
                curve_ff = (curve_ff < 0) ? 0 : curve_ff;
            }
            else if (curve_ff < 0)
            {
                curve_ff += LINE_CURVE_DECAY_MM_S;
                curve_ff = (curve_ff > 0) ? 0 : curve_ff;
            }
            else
            {
                /* on a straight */
            }
            curve_ff = (curve_ff > LINE_CURVE_MAX_MM_S) ? LINE_CURVE_MAX_MM_S
                     : ((curve_ff < -LINE_CURVE_MAX_MM_S)
                        ? -LINE_CURVE_MAX_MM_S : curve_ff);
            if ((0 != ff_prev) && (0 == curve_ff))
            {
                curve_end = d;
            }
            target = (((line_dir + bias) - heading_cdeg())
                      / LINE_HDG_GAIN_DIV) + curve_ff;
            lim = (off < -1) ? LINE_CORR_MAX_MM_S : LINE_TRIM_MAX_MM_S;
            target = (target > lim) ? lim : ((target < -lim) ? -lim : target);
            /* Slew-limit the steering so it never jerks */
            corr += (target > corr + LINE_SLEW_MM_S) ? LINE_SLEW_MM_S
                  : ((target < corr - LINE_SLEW_MM_S) ? -LINE_SLEW_MM_S
                                                      : (target - corr));

            (void)motion_set_velocity((int16_t)(base - corr),
                                      (int16_t)(base + corr));
            trace(d, corr, curve_ff, bits, LINE_TR_FOLLOW, off);
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
    g_straight = 0u;
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

void
line_set_straight(bool b_on)
{
    g_straight = b_on ? 1u : 0u;
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
    uint32_t first;

    if (NULL == p_out)
    {
        return 0u;
    }
    first = (n > LINE_TRACE_LEN) ? (n - LINE_TRACE_LEN) : 0u;
    for (i = first; i < n; i++)
    {
        p_out[i - first] = g_trace[i % LINE_TRACE_LEN];
    }
    return n - first;
}
