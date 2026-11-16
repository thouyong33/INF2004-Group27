/**
 * @file barcode_decode_test.c
 * @brief Host-side (PC) unit test of the Buddy 3 Code 39 decoder.
 *
 * Tests barcode_decode() on synthetic element lists and barcode_read() on
 * synthetic millivolt profiles (white 300 mV, bars 1800 mV, narrow = N
 * samples, wide = 3N). Covers forward/reversed reads, scale independence,
 * a read one bar short at either end, both ends short (must fail), garbage
 * and too few elements. The Code 39 patterns are the ones in barcode.c.
 *
 * Build and run (from mtk3smp-rp2040, e.g. in WSL):
 *   gcc -Wall -Wextra -Iapp_program/buddy3_line_barcode \
 *       tests/host/barcode_decode_test.c \
 *       app_program/buddy3_line_barcode/barcode.c -o bdt && ./bdt
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "barcode.h"

#define PAT_STAR        0x094U
#define PAT_A           0x109U
#define PAT_Z           0x0D0U
#define MV_WHITE        300U
#define MV_BAR          1800U
#define LEAD_SAMPLES    30U
#define PROFILE_MAX     1024U
#define GARBAGE_LISTS   20U
#define GARBAGE_ELEMS   40U

static uint8_t g_units[BC_MAX_ELEMS];   /* width in narrow units */
static uint32_t g_nunits;
static uint32_t g_fail;

/* Element k of the code, 0 = first bar; bars sit on even indexes */
static void
build_code(const uint16_t *p_pats, uint32_t n_chars)
{
    uint32_t c;
    uint32_t k;

    g_nunits = 0U;
    for (c = 0U; c < n_chars; c++)
    {
        if (c > 0U)
        {
            g_units[g_nunits++] = 1U;           /* narrow inter-char gap */
        }
        for (k = 0U; k < 9U; k++)
        {
            uint16_t bit = (uint16_t)((p_pats[c] >> (8U - k)) & 1U);

            g_units[g_nunits++] = (0U != bit) ? 3U : 1U;
        }
    }
}

/* Elements off..off+cnt-1 of the code, scaled by n, optionally reversed */
static void
make_elems(bc_elem_t *p_el, uint32_t off, uint32_t cnt, uint32_t n,
           bool b_rev)
{
    uint32_t k;

    for (k = 0U; k < cnt; k++)
    {
        uint32_t src = b_rev ? (off + cnt - 1U - k) : (off + k);

        p_el[k].bar = (0U == (src % 2U)) ? 1U : 0U;
        p_el[k].w = (uint16_t)(g_units[src] * n);
        p_el[k].peak = (uint16_t)MV_BAR;
    }
}

/* Same as a millivolt profile with white lead-in and lead-out */
static uint32_t
make_profile(uint16_t *p_mv, uint32_t off, uint32_t cnt, uint32_t n,
             bool b_rev)
{
    uint32_t k;
    uint32_t j;
    uint32_t len = 0U;

    for (j = 0U; j < LEAD_SAMPLES; j++)
    {
        p_mv[len++] = (uint16_t)MV_WHITE;
    }
    for (k = 0U; k < cnt; k++)
    {
        uint32_t src = b_rev ? (off + cnt - 1U - k) : (off + k);
        uint16_t mv = (0U == (src % 2U)) ? (uint16_t)MV_BAR
                                         : (uint16_t)MV_WHITE;

        for (j = 0U; j < (g_units[src] * n); j++)
        {
            p_mv[len++] = mv;
        }
    }
    for (j = 0U; j < LEAD_SAMPLES; j++)
    {
        p_mv[len++] = (uint16_t)MV_WHITE;
    }
    return len;
}

static void
report(const char *p_name, bool b_pass, const char *p_why)
{
    if (b_pass)
    {
        printf("ok   %s\n", p_name);
    }
    else
    {
        printf("FAIL %s (%s)\n", p_name, p_why);
        g_fail++;
    }
}

static bool
result_ok(const bc_result_t *p_res, char want, bool b_rev, uint8_t cut)
{
    return (1U == p_res->len) && (want == p_res->text[0])
           && ((b_rev ? 1U : 0U) == p_res->reversed)
           && (cut == p_res->cut);
}

/* Run one case through barcode_decode() and barcode_read(). want == 0
 * means: must not decode. */
static void
run_case(const char *p_name, uint32_t off, uint32_t cnt, uint32_t n,
         bool b_rev, char want, uint8_t cut)
{
    bc_elem_t el[BC_MAX_ELEMS];
    bc_elem_t el2[BC_MAX_ELEMS];
    uint16_t mv[PROFILE_MAX];
    bc_result_t res;
    bc_result_t res2;
    uint16_t white = 0U;
    uint32_t ne = 0U;
    uint32_t len;
    bool b_dec;
    bool b_rd;

    memset(&res, 0, sizeof(res));
    memset(&res2, 0, sizeof(res2));
    make_elems(el, off, cnt, n, b_rev);
    len = make_profile(mv, off, cnt, n, b_rev);
    b_dec = barcode_decode(el, cnt, &res);
    b_rd = barcode_read(mv, len, el2, BC_MAX_ELEMS, &ne, &white, &res2);

    if ('\0' == want)
    {
        report(p_name, (!b_dec) && (!b_rd), "decoded but should not");
        return;
    }
    if (!b_dec)
    {
        report(p_name, false, "barcode_decode found nothing");
    }
    else if (!result_ok(&res, want, b_rev, cut))
    {
        printf("     decode: '%s' rev=%u cut=%u errs=%u\n", res.text,
               (unsigned)res.reversed, (unsigned)res.cut,
               (unsigned)res.star_errs);
        report(p_name, false, "barcode_decode wrong result");
    }
    else if (!b_rd)
    {
        report(p_name, false, "barcode_read found nothing");
    }
    else if (!result_ok(&res2, want, b_rev, cut))
    {
        printf("     read: '%s' rev=%u cut=%u errs=%u\n", res2.text,
               (unsigned)res2.reversed, (unsigned)res2.cut,
               (unsigned)res2.star_errs);
        report(p_name, false, "barcode_read wrong result");
    }
    else
    {
        report(p_name, true, "");
    }
}

static void
test_garbage(void)
{
    bc_elem_t el[GARBAGE_ELEMS];
    bc_result_t res;
    uint32_t seed = 12345U;
    uint32_t l;
    uint32_t k;
    uint32_t decoded = 0U;

    for (l = 0U; l < GARBAGE_LISTS; l++)
    {
        for (k = 0U; k < GARBAGE_ELEMS; k++)
        {
            seed = (seed * 1103515245U) + 12345U;
            el[k].bar = (0U == (k % 2U)) ? 1U : 0U;
            el[k].w = (uint16_t)((1U + ((seed >> 16) % 3U)) * 4U);
            el[k].peak = (uint16_t)MV_BAR;
        }
        memset(&res, 0, sizeof(res));
        if (barcode_decode(el, GARBAGE_ELEMS, &res))
        {
            decoded++;
        }
    }
    printf("info garbage: %u of %u lists decoded\n", (unsigned)decoded,
           (unsigned)GARBAGE_LISTS);
    report("garbage lists (informational)", true, "");
}

static void
test_too_few(void)
{
    bc_elem_t el[5];
    bc_result_t res;
    uint32_t k;

    for (k = 0U; k < 5U; k++)
    {
        el[k].bar = (0U == (k % 2U)) ? 1U : 0U;
        el[k].w = (uint16_t)((0U == k) ? 12U : 4U);
        el[k].peak = (uint16_t)MV_BAR;
    }
    memset(&res, 0, sizeof(res));
    report("too few elements (5)", !barcode_decode(el, 5U, &res),
           "decoded");
}

int
main(void)
{
    uint16_t pats_a[3] = { PAT_STAR, PAT_A, PAT_STAR };
    uint16_t pats_z[3] = { PAT_STAR, PAT_Z, PAT_STAR };

    build_code(pats_a, 3U);                     /* 29 elements */
    run_case("*A* forward", 0U, 29U, 4U, false, 'A', BC_CUT_NONE);
    run_case("*A* reversed", 0U, 29U, 4U, true, 'A', BC_CUT_NONE);

    build_code(pats_z, 3U);
    run_case("*Z* forward", 0U, 29U, 4U, false, 'Z', BC_CUT_NONE);
    run_case("*Z* reversed", 0U, 29U, 4U, true, 'Z', BC_CUT_NONE);

    build_code(pats_a, 3U);
    run_case("scale narrow=2 (1:3)", 0U, 29U, 2U, false, 'A', BC_CUT_NONE);
    run_case("scale narrow=6 (1:3)", 0U, 29U, 6U, false, 'A', BC_CUT_NONE);

    /* last narrow bar and the space before it dropped: 27 elements */
    run_case("last bar missing -> BC_CUT_END", 0U, 27U, 4U, false, 'A',
             BC_CUT_END);
    /* first bar and the wide space after it dropped */
    run_case("first bar missing -> BC_CUT_START", 2U, 27U, 4U, false, 'A',
             BC_CUT_START);
    run_case("bars missing at both ends", 2U, 25U, 4U, false, '\0',
             BC_CUT_NONE);

    test_garbage();
    test_too_few();

    if (0U == g_fail)
    {
        printf("ALL PASS\n");
    }
    else
    {
        printf("%u FAILED\n", (unsigned)g_fail);
    }
    return (int)g_fail;
}
