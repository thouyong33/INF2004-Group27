/**
 * @file barcode_scan.c
 * @brief Buddy 3: barcode scanning task (left IR AO while driving).
 */

#include <stdint.h>
#include <stdbool.h>

#include <tk/tkernel.h>

#include "../common/ctrl_events.h"
#include "../buddy2_motion/motion_task.h"
#include "ir_sensor.h"
#include "barcode.h"
#include "barcode_scan.h"
#include "line.h"

/* Lowest of the car's tasks: the ADC poll is short, and a few ms late
 * only repeats one reading over the steps passed meanwhile */
#define BCS_TASK_PRI         8u
#define BCS_STACK_SIZE       1024u   /* barcode_decode: 512 B histogram */
#define BCS_FLG_RUN          (1u << 0)
#define BCS_POLL_MS          1u      /* tk_dly_tsk(1) = 2 ms */

#define UM_PER_EDGE          328
/* steps: ~360 mm, a 3-character code at the A4 scale (narrow 6 mm,
 * 282 mm) plus margins; the sample scale (narrow 3 mm) is 141 mm */
#define BCS_LEN              1100
/* steps of white kept before the first bar */
#define BCS_LEAD             32
#define BCS_QUIET_STEPS      ((BCS_QUIET_MM * 1000) / UM_PER_EDGE)
#define BCS_RESET_STEPS      300     /* a drop this big = odometry was reset */
/* End of a code: white for BCS_QUIET_MM, or for BCS_QUIET_NARROW_X times
 * the narrowest bar seen so far if that is longer (up to BCS_QUIET_MAX_MM).
 * A wide space is 3 narrow at any scale (1:3), so at the A4 scale (wide
 * space 18 mm) a fixed 15 mm cut codes into pieces; one piece could even
 * look like a cross arm. Bars under BCS_BAR_MIN_STEPS are noise. */
#define BCS_QUIET_NARROW_X   4u
#define BCS_QUIET_MAX_MM     30
#define BCS_QUIET_MAX_STEPS  ((BCS_QUIET_MAX_MM * 1000) / UM_PER_EDGE)
#define BCS_BAR_MIN_STEPS    3u
/* A cross arm under the left IR reads as one bar 18 mm wide. When the
 * car crosses pointing a few degrees left, the right IR runs from the arm
 * straight onto the main line and the line task never reports the
 * junction (ctrl_course_v15 runs 5-6), so the arm seen here reports it
 * too: EVT_JUNCTION arg 1, val = where the right IR meets the arm (the
 * left IR sits LEFT_IR_AHEAD_MM ahead of it, user 28 Oct), in the line
 * task's distance. */
#define ARM_MIN_MM           10
#define ARM_MAX_MM           35
#define LEFT_IR_AHEAD_MM     20
/* ...and only if no other read ended within this before it began (a
 * piece of a code is never an arm) */
#define ARM_CLEAR_MM         30
#define ARM_CLEAR_STEPS      ((ARM_CLEAR_MM * 1000) / UM_PER_EDGE)

/* Distance travelled AT THE LEFT IR, not at the car's centre. While line
 * following the car yaws +/-0.5..2 rad/s, and the left IR sits ~80 mm left
 * of the centreline, so its own speed along the barcode swings by tens of
 * mm/s around the centre's 100 mm/s: bar widths measured with the centre
 * distance came out distorted and no read decoded (line runs, 24 Oct).
 * Along the heading, s_IR = s_centre - theta * 80, theta = (sR - sL) / 114:
 * s_IR = sL * (0.5 + 80/114) + sR * (0.5 - 80/114), weights x4096. */
#define BCS_W_LEFT           4923L   /* 1.2018 x 4096 */
#define BCS_W_RIGHT          (-827L) /* -0.2018 x 4096 */

static ID g_bcs_tsk = 0;
static ID g_bcs_flg = 0;
static volatile uint8_t g_enabled = 0u;
static volatile uint32_t g_reads = 0u;

static uint16_t g_buf[BCS_LEN];
static uint16_t g_lead[BCS_LEAD];
static bc_elem_t g_el[BC_MAX_ELEMS];
static bc_elem_t g_last_el[BC_MAX_ELEMS];
static volatile uint32_t g_last_n = 0u;

static int32_t
steps_now(void)
{
    enc_diag_t l, r;

    (void)motion_get_encoder_diag(&l, &r);
    return ((l.hw_count * BCS_W_LEFT) + (r.hw_count * BCS_W_RIGHT)) / 4096L;
}

/* Where the right IR meets the arm whose bar (w steps) just ended a read
 * (quiet steps of white after it), as the line task's distance (the mean
 * of both wheels, mm) */
static int32_t
arm_start_mm(uint16_t w, uint32_t quiet)
{
    enc_diag_t l, r;
    int32_t now_mm;

    (void)motion_get_encoder_diag(&l, &r);
    now_mm = (((l.hw_count + r.hw_count) / 2) * UM_PER_EDGE) / 1000;
    return now_mm - ((((int32_t)w + (int32_t)quiet) * UM_PER_EDGE) / 1000)
           + LEFT_IR_AHEAD_MM;
}

/* Decode a finished read of n steps (quiet: the white that ended it;
 * b_clear: no other read ended just before it) and post the result */
static void
finish(uint32_t n, uint32_t quiet, bool b_clear)
{
    bc_result_t res;
    uint32_t ne;
    uint32_t i;

    bool b_ok = barcode_read(g_buf, n, g_el, BC_MAX_ELEMS, &ne, NULL, &res);

    /* Keep the last read long enough to be a code (not a junction arm) */
    if (ne >= 9u)
    {
        for (i = 0u; i < ne; i++)
        {
            g_last_el[i] = g_el[i];
        }
        g_last_n = ne;
    }
    if ((!b_ok) && b_clear && (1u == ne) && (0u != g_el[0].bar)
        && (g_el[0].w >= (uint16_t)((ARM_MIN_MM * 1000) / UM_PER_EDGE))
        && (g_el[0].w <= (uint16_t)((ARM_MAX_MM * 1000) / UM_PER_EDGE)))
    {
        ctrl_evt_post(EVT_JUNCTION, 1u,
                      (int16_t)arm_start_mm(g_el[0].w, quiet));
    }
    if (b_ok)
    {
        ctrl_evt_post(EVT_BARCODE, (uint8_t)res.text[0], (int16_t)res.reversed);
    }
    else
    {
        ctrl_evt_post(EVT_BARCODE, 0u, (int16_t)ne);
    }
    g_reads++;
}

static void
bcs_task_main(INT stacd, void *exinf)
{
    UINT flgptn;
    int32_t s;
    int32_t last;
    int32_t k;
    uint32_t n;
    uint32_t lead_i;
    uint32_t lead_n;
    uint32_t quiet;
    uint32_t quiet_need;
    uint32_t bar_run;
    uint32_t min_bar;
    int32_t cap_start;
    int32_t last_end;
    uint32_t i;
    uint16_t mv;
    uint16_t white;
    bool b_capturing;

    (void)stacd;
    (void)exinf;

    while (1)
    {
        (void)tk_wai_flg(g_bcs_flg, BCS_FLG_RUN, TWF_ORW, &flgptn, TMO_FEVR);

        last = steps_now();
        white = (uint16_t)ir_ao_left_mv();
        b_capturing = false;
        n = 0u;
        lead_i = 0u;
        lead_n = 0u;
        quiet = 0u;
        quiet_need = (uint32_t)BCS_QUIET_STEPS;
        bar_run = 0u;
        min_bar = 0xFFFFu;
        cap_start = 0;
        last_end = last - (2 * ARM_CLEAR_STEPS);

        while (0u != g_enabled)
        {
            tk_dly_tsk(BCS_POLL_MS);
            s = steps_now();
            if (s < (last - BCS_RESET_STEPS))
            {
                /* Odometry was reset (line following restarted) */
                last = s;
                b_capturing = false;
                line_set_straight(false);
                lead_n = 0u;
                continue;
            }
            if (s <= last)
            {
                /* Not moved, or the IR swung back a little in a hard turn */
                continue;
            }
            mv = (uint16_t)ir_ao_left_mv();

            for (k = last + 1; k <= s; k++)
            {
                if (!b_capturing)
                {
                    /* Follow the white level slowly, faster downwards */
                    if (mv < white)
                    {
                        white = (uint16_t)((3u * white + mv) / 4u);
                    }
                    else if (mv < (uint16_t)(white + BC_OFF_MV))
                    {
                        white = (uint16_t)((15u * white + mv) / 16u);
                    }

                    if (mv > (uint16_t)(white + BC_ON_MV))
                    {
                        /* First bar: start with the white kept before it */
                        b_capturing = true;
                        n = 0u;
                        for (i = 0u; i < lead_n; i++)
                        {
                            g_buf[n++] = g_lead[(lead_i + BCS_LEAD - lead_n + i)
                                                % BCS_LEAD];
                        }
                        quiet = 0u;
                        quiet_need = (uint32_t)BCS_QUIET_STEPS;
                        bar_run = 0u;
                        min_bar = 0xFFFFu;
                        cap_start = k;
                        line_set_straight(true);
                    }
                    else
                    {
                        g_lead[lead_i] = mv;
                        lead_i = (lead_i + 1u) % BCS_LEAD;
                        if (lead_n < BCS_LEAD)
                        {
                            lead_n++;
                        }
                        continue;
                    }
                }

                g_buf[n++] = mv;
                if (mv < (uint16_t)(white + BC_OFF_MV))
                {
                    /* White: a bar just ended? keep the narrowest */
                    if ((bar_run >= BCS_BAR_MIN_STEPS) && (bar_run < min_bar))
                    {
                        min_bar = bar_run;
                        quiet_need = BCS_QUIET_NARROW_X * min_bar;
                        quiet_need = (quiet_need < (uint32_t)BCS_QUIET_STEPS)
                                     ? (uint32_t)BCS_QUIET_STEPS
                                     : ((quiet_need
                                         > (uint32_t)BCS_QUIET_MAX_STEPS)
                                        ? (uint32_t)BCS_QUIET_MAX_STEPS
                                        : quiet_need);
                    }
                    bar_run = 0u;
                    quiet++;
                }
                else
                {
                    bar_run++;
                    quiet = 0u;
                }
                if ((quiet >= quiet_need) || (n >= BCS_LEN))
                {
                    line_set_straight(false);
                    finish(n, quiet,
                           (cap_start - last_end) >= ARM_CLEAR_STEPS);
                    last_end = k;
                    b_capturing = false;
                    lead_n = 0u;
                    break;
                }
            }
            last = s;
        }
    }
}

ER
barcode_scan_create(void)
{
    T_CFLG cflg;
    T_CTSK ctsk;
    ER er;

    if (g_bcs_tsk > 0)
    {
        return E_OBJ;
    }
    ir_ao_init();

    cflg.flgatr = TA_TFIFO | TA_WSGL;
    cflg.iflgptn = 0u;
    g_bcs_flg = tk_cre_flg(&cflg);
    if (g_bcs_flg <= 0)
    {
        er = g_bcs_flg;
        g_bcs_flg = 0;
        return er;
    }

    ctsk.tskatr = TA_HLNG | TA_RNG3;
    ctsk.task = bcs_task_main;
    ctsk.itskpri = BCS_TASK_PRI;
    ctsk.stksz = BCS_STACK_SIZE;
    g_bcs_tsk = tk_cre_tsk(&ctsk);
    if (g_bcs_tsk <= 0)
    {
        er = g_bcs_tsk;
        g_bcs_tsk = 0;
        (void)tk_del_flg(g_bcs_flg);
        g_bcs_flg = 0;
        return er;
    }
    er = tk_sta_tsk(g_bcs_tsk, 0);
    if (E_OK != er)
    {
        (void)tk_del_tsk(g_bcs_tsk);
        g_bcs_tsk = 0;
        (void)tk_del_flg(g_bcs_flg);
        g_bcs_flg = 0;
    }
    return er;
}

void
barcode_scan_enable(bool b_on)
{
    if (g_bcs_flg <= 0)
    {
        return;
    }
    g_enabled = b_on ? 1u : 0u;
    if (b_on)
    {
        (void)tk_set_flg(g_bcs_flg, BCS_FLG_RUN);
    }
    else
    {
        (void)tk_clr_flg(g_bcs_flg, (UINT)~BCS_FLG_RUN);
        line_set_straight(false);           /* a read in progress is dropped */
    }
}

uint32_t
barcode_scan_count(void)
{
    return g_reads;
}

uint32_t
barcode_scan_last(bc_elem_t *p_out, uint32_t max)
{
    uint32_t i;
    uint32_t n = g_last_n;

    if (NULL == p_out)
    {
        return 0u;
    }
    n = (n < max) ? n : max;
    for (i = 0u; i < n; i++)
    {
        p_out[i] = g_last_el[i];
    }
    return n;
}
