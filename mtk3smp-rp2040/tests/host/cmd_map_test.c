/*
 * cmd_map_test.c
 * Host test of the barcode mapping sent over MQTT (Buddy 1,
 * app_program/buddy1_telemetry/cmd_map.c): packing, validation and
 * lookup. Build and run with tests/host/run_host_tests.sh.
 */

#include <stdio.h>
#include <string.h>

#include "buddy1_telemetry/cmd_map.h"
#include "controller/vehicle_ctrl.h"

static int g_fails = 0;

static void
check(int b_ok, const char *p_name)
{
    printf("%s %s\n", b_ok ? "ok  " : "FAIL", p_name);
    if (!b_ok)
    {
        g_fails++;
    }
}

int
main(void)
{
    /* forward A, U-turn Z, left B, right C, as the dashboard packs it */
    const uint8_t azbc[CMD_MAP_LEN] = {0xCAU, 0xB8U, 0x30U};
    const uint8_t bad_idx[CMD_MAP_LEN] = {0xFFU, 0xFFU, 0xFFU};
    const uint8_t dup[CMD_MAP_LEN] = {0x8AU, 0xA2U, 0x28U};   /* A A A A */
    uint8_t out[CMD_MAP_LEN];
    char chars[CMD_MAP_CHARS];

    check(!cmd_map_get(chars), "no mapping at start");
    check(CTRL_CMD_NONE == cmd_map_lookup('A'), "no command before a map");

    check(cmd_map_set_packed(azbc), "A Z B C accepted");
    check(cmd_map_get(chars) && (0 == memcmp(chars, "AZBC", 4)),
          "unpacks to A Z B C");
    check(CTRL_CMD_STRAIGHT == cmd_map_lookup('A'), "A = forward");
    check(CTRL_CMD_UTURN == cmd_map_lookup('Z'), "Z = U-turn");
    check(CTRL_CMD_LEFT == cmd_map_lookup('B'), "B = left");
    check(CTRL_CMD_RIGHT == cmd_map_lookup('C'), "C = right");
    check(CTRL_CMD_NONE == cmd_map_lookup('Q'), "unmapped Q = none");
    check(cmd_map_get_packed(out) && (0 == memcmp(out, azbc, 3)),
          "packs back to ca b8 30");

    check(!cmd_map_set_packed(bad_idx), "index past the charset rejected");
    check(!cmd_map_set_packed(dup), "same character twice rejected");
    check(!cmd_map_set_chars("ab C"), "lower case rejected");
    check(CTRL_CMD_UTURN == cmd_map_lookup('Z'),
          "a rejected map keeps the old one");

    check(cmd_map_set_chars("1-.%"), "digits and symbols accepted");
    check(CTRL_CMD_RIGHT == cmd_map_lookup('%'), "% = right");
    check(CTRL_CMD_NONE == cmd_map_lookup('A'), "old A no longer mapped");

    printf("%s\n", (0 == g_fails) ? "ALL PASS" : "FAILED");
    return g_fails;
}
