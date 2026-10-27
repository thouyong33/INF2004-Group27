/**
 * @file barcode.c
 * @brief Buddy 3: Code 39 decoding from the left IR's analog profile.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "barcode.h"

#define BC_HIST_BINS         256u    /* 16 mV bins up to 4095 mV */
#define BC_WIDE_RATIO_X10    18      /* widest >= 1.8 x narrowest, else all narrow */
#define BC_STAR              0x094u

/* Code 39: 9 elements (bar, space, ... bar), bit 8 = first, 1 = wide */
static const struct {
    char c;
    uint16_t pat;
} c_c39[] = {
    {'0', 0x034u}, {'1', 0x121u}, {'2', 0x061u}, {'3', 0x160u}, {'4', 0x031u},
    {'5', 0x130u}, {'6', 0x070u}, {'7', 0x025u}, {'8', 0x124u}, {'9', 0x064u},
    {'A', 0x109u}, {'B', 0x049u}, {'C', 0x148u}, {'D', 0x019u}, {'E', 0x118u},
    {'F', 0x058u}, {'G', 0x00Du}, {'H', 0x10Cu}, {'I', 0x04Cu}, {'J', 0x01Cu},
    {'K', 0x103u}, {'L', 0x043u}, {'M', 0x142u}, {'N', 0x013u}, {'O', 0x112u},
    {'P', 0x052u}, {'Q', 0x007u}, {'R', 0x106u}, {'S', 0x046u}, {'T', 0x016u},
    {'U', 0x181u}, {'V', 0x0C1u}, {'W', 0x1C0u}, {'X', 0x091u}, {'Y', 0x190u},
    {'Z', 0x0D0u}, {'-', 0x085u}, {'.', 0x184u}, {' ', 0x0C4u},
    {'$', 0x0A8u}, {'/', 0x0A2u}, {'+', 0x08Au}, {'%', 0x02Au}
};

/* White = 10th percentile, dark = BC_DARK_PCTL percentile of the profile */
static void
levels(const uint16_t *p_mv, uint32_t n, uint16_t *p_white, uint16_t *p_dark)
{
    uint16_t hist[BC_HIST_BINS];
    uint32_t i;
    uint32_t sum = 0u;
    bool got_white = false;

    for (i = 0u; i < BC_HIST_BINS; i++)
    {
        hist[i] = 0u;
    }
    for (i = 0u; i < n; i++)
    {
        uint32_t b = (uint32_t)p_mv[i] / 16u;

        hist[(b < BC_HIST_BINS) ? b : (BC_HIST_BINS - 1u)]++;
    }
    *p_white = 8u;
    *p_dark = 8u;
    for (i = 0u; i < BC_HIST_BINS; i++)
    {
        sum += hist[i];
        if ((!got_white) && ((sum * 10u) >= n))
        {
            *p_white = (uint16_t)((i * 16u) + 8u);
            got_white = true;
        }
        if ((sum * 100u) >= (n * BC_DARK_PCTL))
        {
            *p_dark = (uint16_t)((i * 16u) + 8u);
            break;
        }
    }
}

uint32_t
barcode_elements(const uint16_t *p_mv, uint32_t n, bc_elem_t *p_el,
                 uint32_t max, uint16_t *p_white, bool span_thr)
{
    uint32_t i;
    uint32_t start = 0u;
    uint32_t ne = 0u;
    uint16_t white;
    uint16_t dark;
    uint16_t on;
    uint16_t off;
    uint16_t peak;
    uint32_t span;
    bool black;

    if ((NULL == p_mv) || (NULL == p_el) || (n < 2u) || (0u == max))
    {
        return 0u;
    }
    levels(p_mv, n, &white, &dark);
    if (NULL != p_white)
    {
        *p_white = white;
    }
    /* Threshold: a fixed margin above white (faint thin bars, 24 Oct), or
     * with span_thr a share of the white-to-dark span if that is higher
     * (strong contrast leaves the narrow spaces well above white, 26 Oct) */
    span = (span_thr && (dark > white)) ? (uint32_t)(dark - white) : 0u;
    on = (uint16_t)(white + (((span * BC_ON_PCT) / 100u > BC_ON_MV)
                             ? ((span * BC_ON_PCT) / 100u) : BC_ON_MV));
    off = (uint16_t)(white + (((span * BC_OFF_PCT) / 100u > BC_OFF_MV)
                              ? ((span * BC_OFF_PCT) / 100u) : BC_OFF_MV));

    black = (p_mv[0] > on);
    peak = p_mv[0];
    for (i = 1u; i <= n; i++)
    {
        if ((i == n) || (black ? (p_mv[i] < off) : (p_mv[i] > on)))
        {
            /* Skip the white lead-in; the lead-out is trimmed below */
            if ((ne > 0u) || black)
            {
                if (ne < max)
                {
                    p_el[ne].bar = black ? 1u : 0u;
                    p_el[ne].w = (uint16_t)(i - start);
                    p_el[ne].peak = peak;
                    ne++;
                }
            }
            if (i < n)
            {
                black = !black;
                start = i;
                peak = p_mv[i];
            }
        }
        else if (p_mv[i] > peak)
        {
            peak = p_mv[i];
        }
    }
    if ((ne > 0u) && (0u == p_el[ne - 1u].bar))
    {
        ne--;
    }
    return ne;
}

/* Element k of the window, read forwards or backwards */
static const bc_elem_t *
elem(const bc_elem_t *p_el, uint32_t n, uint32_t k, bool rev)
{
    return rev ? &p_el[n - 1u - k] : &p_el[k];
}

/* Wide/narrow split for one kind (bars or spaces) of a window */
static uint16_t
split(const bc_elem_t *p_el, uint32_t n, uint32_t st, uint32_t ln, bool rev,
      uint8_t bar)
{
    uint32_t k;
    uint16_t lo = 0xFFFFu;
    uint16_t hi = 0u;

    for (k = st; k < (st + ln); k++)
    {
        const bc_elem_t *p_e = elem(p_el, n, k, rev);

        if (p_e->bar == bar)
        {
            lo = (p_e->w < lo) ? p_e->w : lo;
            hi = (p_e->w > hi) ? p_e->w : hi;
        }
    }
    if (((uint32_t)hi * 10u) < ((uint32_t)lo * BC_WIDE_RATIO_X10))
    {
        return 0xFFFFu;          /* all the same width: all narrow */
    }
    return (uint16_t)(((uint32_t)lo + hi) / 2u);
}

static uint32_t
bits_diff(uint16_t a, uint16_t b)
{
    uint16_t x = (uint16_t)(a ^ b);
    uint32_t c = 0u;

    while (0u != x)
    {
        c += (x & 1u);
        x = (uint16_t)(x >> 1);
    }
    return c;
}

static bool
try_window(const bc_elem_t *p_el, uint32_t n, uint32_t st, uint32_t ln, bool rev,
           bc_result_t *p_res)
{
    uint16_t bthr;
    uint16_t sthr;
    uint16_t pat;
    uint32_t c;
    uint32_t k;
    uint32_t i;
    uint32_t nch = (ln + 1u) / 10u;
    uint32_t errs = 0u;
    bool found;

    bthr = split(p_el, n, st, ln, rev, 1u);
    sthr = split(p_el, n, st, ln, rev, 0u);
    p_res->len = 0u;

    for (c = 0u; c < nch; c++)
    {
        pat = 0u;
        for (k = 0u; k < 9u; k++)
        {
            const bc_elem_t *p_e = elem(p_el, n, st + (c * 10u) + k, rev);

            pat = (uint16_t)((pat << 1)
                             | ((p_e->w > ((0u != p_e->bar) ? bthr : sthr)) ? 1u : 0u));
        }
        if ((0u == c) || ((nch - 1u) == c))
        {
            errs += bits_diff(pat, BC_STAR);
            if (bits_diff(pat, BC_STAR) > 1u)
            {
                return false;
            }
            continue;
        }
        found = false;
        for (i = 0u; i < (sizeof(c_c39) / sizeof(c_c39[0])); i++)
        {
            if (c_c39[i].pat == pat)
            {
                p_res->text[p_res->len++] = c_c39[i].c;
                found = true;
                break;
            }
        }
        if (!found)
        {
            return false;
        }
    }
    p_res->text[p_res->len] = '\0';
    p_res->reversed = rev ? 1u : 0u;
    p_res->star_errs = (uint8_t)errs;
    p_res->first = (uint8_t)(rev ? (n - st - ln) : st);
    return true;
}

bool
barcode_read(const uint16_t *p_mv, uint32_t n, bc_elem_t *p_el, uint32_t max,
             uint32_t *p_ne, uint16_t *p_white, bc_result_t *p_res)
{
    uint32_t ne;

    /* Span-based threshold first; the fixed margin rescues faint reads */
    ne = barcode_elements(p_mv, n, p_el, max, p_white, true);
    if (barcode_decode(p_el, ne, p_res))
    {
        *p_ne = ne;
        return true;
    }
    ne = barcode_elements(p_mv, n, p_el, max, p_white, false);
    *p_ne = ne;
    return barcode_decode(p_el, ne, p_res);
}

bool
barcode_decode(const bc_elem_t *p_el, uint32_t n, bc_result_t *p_res)
{
    uint32_t st;
    uint32_t ln;
    uint32_t r;

    if ((NULL == p_el) || (NULL == p_res))
    {
        return false;
    }
    /* Shortest code first (one data character), forwards then backwards */
    for (ln = 29u; ln <= (9u + (10u * (BC_MAX_TEXT + 1u))); ln += 10u)
    {
        for (r = 0u; r < 2u; r++)
        {
            for (st = 0u; (st + ln) <= n; st++)
            {
                if ((0u != elem(p_el, n, st, 0u != r)->bar)
                    && try_window(p_el, n, st, ln, 0u != r, p_res))
                {
                    return true;
                }
            }
        }
    }
    return false;
}
