/**
 * @file ctrl_events.c
 * @brief Event queue to the vehicle controller (a T-Kernel message buffer).
 */

#include <stdint.h>
#include <tk/tkernel.h>

#include "ctrl_events.h"

#define CTRL_EVT_QUEUE_LEN   16u
/* Each queued message also carries the kernel's size header */
#define CTRL_EVT_BUF_SZ      (CTRL_EVT_QUEUE_LEN * (sizeof(ctrl_evt_t) + sizeof(INT)))

static ID g_ctrl_evt_mbf = 0;
static volatile uint32_t g_dropped = 0u;

ER
ctrl_evt_init(void)
{
    T_CMBF cmbf;

    if (g_ctrl_evt_mbf > 0)
    {
        return E_OK;
    }
    cmbf.mbfatr = TA_TFIFO;
    cmbf.bufsz = (SZ)CTRL_EVT_BUF_SZ;
    cmbf.maxmsz = (INT)sizeof(ctrl_evt_t);
    g_ctrl_evt_mbf = tk_cre_mbf(&cmbf);
    if (g_ctrl_evt_mbf <= 0)
    {
        ER er = g_ctrl_evt_mbf;

        g_ctrl_evt_mbf = 0;
        return er;
    }
    return E_OK;
}

void
ctrl_evt_post(uint8_t type, uint8_t arg, int16_t val)
{
    ctrl_evt_t evt;

    if (g_ctrl_evt_mbf <= 0)
    {
        return;
    }
    evt.type = type;
    evt.arg = arg;
    evt.val = val;
    if (E_OK != tk_snd_mbf(g_ctrl_evt_mbf, &evt, (INT)sizeof(evt), TMO_POL))
    {
        g_dropped++;
    }
}

ER
ctrl_evt_wait(ctrl_evt_t *p_evt, TMO tmo)
{
    INT sz;

    if (g_ctrl_evt_mbf <= 0)
    {
        return E_NOEXS;
    }
    sz = tk_rcv_mbf(g_ctrl_evt_mbf, p_evt, tmo);
    if (sz < 0)
    {
        return (ER)sz;
    }
    return E_OK;
}

uint32_t
ctrl_evt_dropped(void)
{
    return g_dropped;
}
