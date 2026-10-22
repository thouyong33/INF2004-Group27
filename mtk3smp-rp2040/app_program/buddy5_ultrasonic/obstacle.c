/**
 * @file obstacle.c
 * @brief Buddy 5: obstacle watch task.
 */

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "../common/faults.h"
#include "../buddy2_motion/motion_task.h"
#include "ultrasonic.h"
#include "servo.h"
#include "scan.h"
#include "obstacle.h"

#define OBS_TASK_PRI         6u      /* just below motion (5) */
#define OBS_STACK_SIZE       2048u   /* the scan runs on this stack */
#define OBS_PING_MS          65u     /* datasheet: >= 60 ms between pings */
#define OBS_ECHO_MAX_US      5830u   /* ~1 m: no-echo pings end early */
#define OBS_DEAD_PINGS       5u      /* echo never started this many times */
#define OBS_HIST_MASK        0x7u    /* last 3 pings, bit 0 = newest */

static ID g_obs_tsk = 0;
static ID g_scan_flg = 0;

/* Shared with other tasks: one byte each, written only by the obstacle
 * task. The history byte is replaced whole (never read-modify-written by
 * readers), so a plain load is a consistent snapshot. */
static volatile uint8_t g_near_hist = 0u;
static volatile int16_t g_last_mm = -1;
static volatile uint32_t g_pings = 0u;
static volatile uint8_t g_reset_req = 0u;

/* Scan request and result. Parameters are written before OBS_FLG_SCAN is
 * set and the result before OBS_FLG_DONE, so the flag orders them. */
static int16_t g_scan_half_deg = 45;
static int16_t g_scan_detect_mm = 600;
static obstacle_profile_t g_scan_prof;
static volatile ER g_scan_er = E_OK;
static volatile uint8_t g_scan_abort = 0u;   /* read by scan.c every step */

/* Ping trace (tests only): written by the obstacle task, read by the test
 * once watching has stopped */
#define UM_PER_EDGE          328
static obstacle_trace_t g_trace[OBS_TRACE_LEN];
static volatile uint32_t g_trace_n = 0u;
static volatile uint8_t g_trace_on = 0u;

static uint8_t
popcount3(uint8_t v)
{
    return (uint8_t)((v & 1u) + ((v >> 1) & 1u) + ((v >> 2) & 1u));
}

static void
obstacle_task_main(INT stacd, void *exinf)
{
    UINT flgptn;
    uint32_t echo_us;
    uint32_t dead = 0u;
    uint8_t hist = 0u;
    bool near;
    ER er;

    (void)stacd;
    (void)exinf;

    scan_set_abort_flag(&g_scan_abort);

    while (1)
    {
        /* Sleep until watching is on or a scan is asked for. WATCH stays
         * set while watching; SCAN is cleared when the scan starts. */
        (void)tk_wai_flg(g_scan_flg, OBS_FLG_WATCH | OBS_FLG_SCAN, TWF_ORW,
                         &flgptn, TMO_FEVR);

        if (0u != (flgptn & OBS_FLG_SCAN))
        {
            /* Watching pauses during a scan: the scan pings for itself */
            (void)tk_clr_flg(g_scan_flg, (UINT)~OBS_FLG_SCAN);
            g_scan_er = scan_obstacle(g_scan_half_deg, g_scan_detect_mm, false,
                                      &g_scan_prof);
            (void)motion_set_velocity(0, 0);
            (void)tk_set_flg(g_scan_flg, OBS_FLG_DONE);
            continue;
        }

        if (0u != g_reset_req)
        {
            g_reset_req = 0u;
            hist = 0u;
            g_near_hist = 0u;
            g_last_mm = -1;
            g_pings = 0u;
        }

        er = ultrasonic_read_us_max(&echo_us, OBS_ECHO_MAX_US);
        near = false;
        if (E_OK == er)
        {
            g_last_mm = (int16_t)ultrasonic_us_to_mm(echo_us);
            near = (g_last_mm > 0) && (g_last_mm <= OBS_STOP_MM);
            dead = 0u;
            if (0u != (fault_get() & FAULT_NO_ECHO))
            {
                fault_clear(FAULT_NO_ECHO);
            }
        }
        else
        {
            g_last_mm = -1;
            if (E_IO == er)
            {
                if (++dead >= OBS_DEAD_PINGS)
                {
                    fault_set(FAULT_NO_ECHO);
                }
            }
            else
            {
                dead = 0u;
            }
        }

        hist = (uint8_t)(((hist << 1) | (near ? 1u : 0u)) & OBS_HIST_MASK);
        g_near_hist = hist;
        g_pings++;

        if (0u != g_trace_on)
        {
            enc_diag_t l, r;
            obstacle_trace_t *p_t = &g_trace[g_trace_n % OBS_TRACE_LEN];

            (void)motion_get_encoder_diag(&l, &r);
            p_t->d_mm = (int16_t)((((l.hw_count + r.hw_count) / 2) * UM_PER_EDGE) / 1000);
            p_t->mm = (E_OK == er) ? g_last_mm : ((E_IO == er) ? -2 : -1);
            p_t->hist = hist;
            g_trace_n++;
        }

        tk_dly_tsk(OBS_PING_MS);
    }
}

ER
obstacle_task_create(void)
{
    T_CFLG cflg;
    T_CTSK ctsk;
    ER er;

    if (g_obs_tsk > 0)
    {
        return E_OBJ;
    }

    ultrasonic_init();
    servo_init(SERVO_CENTRE_US);
    servo_set_limits(SERVO_RIGHT_US, SERVO_LEFT_US);

    cflg.flgatr = TA_TFIFO | TA_WMUL;
    cflg.iflgptn = 0u;
    g_scan_flg = tk_cre_flg(&cflg);
    if (g_scan_flg <= 0)
    {
        return g_scan_flg;
    }

    ctsk.tskatr = TA_HLNG | TA_RNG3;
    ctsk.task = obstacle_task_main;
    ctsk.itskpri = OBS_TASK_PRI;
    ctsk.stksz = OBS_STACK_SIZE;
    g_obs_tsk = tk_cre_tsk(&ctsk);
    if (g_obs_tsk <= 0)
    {
        er = g_obs_tsk;
        g_obs_tsk = 0;
        (void)tk_del_flg(g_scan_flg);
        g_scan_flg = 0;
        return er;
    }

    er = tk_sta_tsk(g_obs_tsk, 0);
    if (E_OK != er)
    {
        (void)tk_del_tsk(g_obs_tsk);
        g_obs_tsk = 0;
        (void)tk_del_flg(g_scan_flg);
        g_scan_flg = 0;
    }
    return er;
}

void
obstacle_watch(bool on)
{
    if (g_scan_flg <= 0)
    {
        return;
    }
    if (on)
    {
        g_near_hist = 0u;
        g_reset_req = 1u;
        (void)tk_set_flg(g_scan_flg, OBS_FLG_WATCH);
    }
    else
    {
        (void)tk_clr_flg(g_scan_flg, (UINT)~OBS_FLG_WATCH);
    }
}

bool
obstacle_blocked(void)
{
    return popcount3(g_near_hist) >= 2u;
}

int16_t
obstacle_last_mm(void)
{
    return g_last_mm;
}

uint32_t
obstacle_ping_count(void)
{
    return g_pings;
}

void
obstacle_trace_start(void)
{
    g_trace_n = 0u;
    g_trace_on = 1u;
}

uint32_t
obstacle_trace_get(obstacle_trace_t *p_out)
{
    uint32_t n = g_trace_n;
    uint32_t first;
    uint32_t i;

    g_trace_on = 0u;
    if (NULL == p_out)
    {
        return 0u;
    }
    first = (n > OBS_TRACE_LEN) ? (n - OBS_TRACE_LEN) : 0u;
    for (i = first; i < n; i++)
    {
        p_out[i - first] = g_trace[i % OBS_TRACE_LEN];
    }
    return n - first;
}

ER
obstacle_scan_start(int16_t half_deg, int16_t detect_mm)
{
    if (g_scan_flg <= 0)
    {
        return E_NOEXS;
    }
    g_scan_half_deg = half_deg;
    g_scan_detect_mm = detect_mm;
    g_scan_abort = 0u;
    (void)tk_clr_flg(g_scan_flg, (UINT)~OBS_FLG_DONE);
    return tk_set_flg(g_scan_flg, OBS_FLG_SCAN);
}

void
obstacle_scan_abort(void)
{
    g_scan_abort = 1u;
}

ER
obstacle_scan_wait(uint32_t timeout_ms, obstacle_profile_t *p_prof)
{
    UINT flgptn;
    ER er;

    if (g_scan_flg <= 0)
    {
        return E_NOEXS;
    }

    er = tk_wai_flg(g_scan_flg, OBS_FLG_DONE, TWF_ORW, &flgptn, (TMO)timeout_ms);
    if (E_TMOUT == er)
    {
        /* Overran: stop the wheels, raise the fault, and wait for the scan
         * to notice the abort (at most one step) before handing back */
        fault_set(FAULT_SCAN_TIMEOUT);
        obstacle_scan_abort();
        motion_stop();
        (void)tk_wai_flg(g_scan_flg, OBS_FLG_DONE, TWF_ORW, &flgptn,
                         (TMO)OBS_ABORT_WAIT_MS);
        return E_TMOUT;
    }
    if (E_OK != er)
    {
        return er;
    }
    if ((NULL != p_prof) && (E_OK == g_scan_er))
    {
        *p_prof = g_scan_prof;
    }
    return g_scan_er;
}
