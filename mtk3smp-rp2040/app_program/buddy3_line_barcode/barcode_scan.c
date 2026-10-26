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

/* Lowest of the car's tasks: the ADC poll is short, and a few ms late
 * only repeats one reading over the steps passed meanwhile */
#define BCS_TASK_PRI         8u
#define BCS_STACK_SIZE       1024u   /* barcode_decode: 512 B histogram */
#define BCS_FLG_RUN          (1u << 0)
#define BCS_POLL_MS          1u      /* tk_dly_tsk(1) = 2 ms */

#define UM_PER_EDGE          328
#define BCS_LEN              640     /* steps: ~210 mm (code 141 mm + margins) */
#define BCS_LEAD             32      /* steps of white kept before the first bar */
#define BCS_QUIET_STEPS      ((BCS_QUIET_MM * 1000) / UM_PER_EDGE)
#define BCS_RESET_STEPS      300     /* a drop this big = odometry was reset */

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

static void
finish(uint32_t n)
{
    bc_result_t res;
    uint32_t ne;
    uint32_t i;

    bool ok = barcode_read(g_buf, n, g_el, BC_MAX_ELEMS, &ne, NULL, &res);

    /* Keep the last read long enough to be a code (not a junction arm) */
    if (ne >= 9u)
    {
        for (i = 0u; i < ne; i++)
        {
            g_last_el[i] = g_el[i];
        }
        g_last_n = ne;
    }
    if (ok)
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
    uint32_t i;
    uint16_t mv;
    uint16_t white;
    bool capturing;

    (void)stacd;
    (void)exinf;

    while (1)
    {
        (void)tk_wai_flg(g_bcs_flg, BCS_FLG_RUN, TWF_ORW, &flgptn, TMO_FEVR);

        last = steps_now();
        white = (uint16_t)ir_ao_left_mv();
        capturing = false;
        n = 0u;
        lead_i = 0u;
        lead_n = 0u;
        quiet = 0u;

        while (0u != g_enabled)
        {
            tk_dly_tsk(BCS_POLL_MS);
            s = steps_now();
            if (s < (last - BCS_RESET_STEPS))
            {
                /* Odometry was reset (line following restarted) */
                last = s;
                capturing = false;
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
                if (!capturing)
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
                        capturing = true;
                        n = 0u;
                        for (i = 0u; i < lead_n; i++)
                        {
                            g_buf[n++] = g_lead[(lead_i + BCS_LEAD - lead_n + i) % BCS_LEAD];
                        }
                        quiet = 0u;
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
                quiet = (mv < (uint16_t)(white + BC_OFF_MV)) ? (quiet + 1u) : 0u;
                if ((quiet >= (uint32_t)BCS_QUIET_STEPS) || (n >= BCS_LEN))
                {
                    finish(n);
                    capturing = false;
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
barcode_scan_enable(bool on)
{
    if (g_bcs_flg <= 0)
    {
        return;
    }
    g_enabled = on ? 1u : 0u;
    if (on)
    {
        (void)tk_set_flg(g_bcs_flg, BCS_FLG_RUN);
    }
    else
    {
        (void)tk_clr_flg(g_bcs_flg, (UINT)~BCS_FLG_RUN);
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
