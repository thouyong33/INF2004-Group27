/**
 * @file cmd_map.c
 * @brief Buddy 1: barcode character -> movement mapping (see cmd_map.h).
 */

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>

#include "controller/vehicle_ctrl.h"
#include "cmd_map.h"

/* The movement for each slot of the mapping, in payload order */
static const uint8_t c_slot_cmd[CMD_MAP_CHARS] = {
    CTRL_CMD_STRAIGHT, CTRL_CMD_UTURN, CTRL_CMD_LEFT, CTRL_CMD_RIGHT
};

/* Four characters, slot 0 in the low byte; 0 = no mapping yet */
static volatile uint32_t g_map = 0u;

/* Index of ch in CMD_MAP_CHARSET, or -1 */
static int32_t
charset_index(uint8_t ch)
{
    static const char charset[] = CMD_MAP_CHARSET;
    uint32_t i;

    for (i = 0u; i < CMD_MAP_CHARSET_LEN; i++)
    {
        if ((uint8_t)charset[i] == ch)
        {
            return (int32_t)i;
        }
    }
    return -1;
}

bool
cmd_map_set_chars(const char p_chars[CMD_MAP_CHARS])
{
    uint32_t word = 0u;
    uint32_t i;
    uint32_t j;

    if (NULL == p_chars)
    {
        return false;
    }
    for (i = 0u; i < CMD_MAP_CHARS; i++)
    {
        if (charset_index((uint8_t)p_chars[i]) < 0)
        {
            return false;
        }
        for (j = 0u; j < i; j++)
        {
            if (p_chars[j] == p_chars[i])
            {
                return false;        /* one character, two movements */
            }
        }
        word |= (uint32_t)(uint8_t)p_chars[i] << (8u * i);
    }
    g_map = word;
    return true;
}

bool
cmd_map_set_packed(const uint8_t p_in[CMD_MAP_LEN])
{
    static const char charset[] = CMD_MAP_CHARSET;
    char chars[CMD_MAP_CHARS];
    uint32_t bits;
    uint32_t idx;
    uint32_t i;

    if (NULL == p_in)
    {
        return false;
    }
    bits = (uint32_t)p_in[0] | ((uint32_t)p_in[1] << 8)
           | ((uint32_t)p_in[2] << 16);
    for (i = 0u; i < CMD_MAP_CHARS; i++)
    {
        idx = (bits >> (6u * i)) & 0x3Fu;
        if (idx >= CMD_MAP_CHARSET_LEN)
        {
            return false;
        }
        chars[i] = charset[idx];
    }
    return cmd_map_set_chars(chars);
}

bool
cmd_map_get(char p_chars[CMD_MAP_CHARS])
{
    uint32_t word = g_map;
    uint32_t i;

    if ((0u == word) || (NULL == p_chars))
    {
        return false;
    }
    for (i = 0u; i < CMD_MAP_CHARS; i++)
    {
        p_chars[i] = (char)((word >> (8u * i)) & 0xFFu);
    }
    return true;
}

bool
cmd_map_get_packed(uint8_t p_out[CMD_MAP_LEN])
{
    char chars[CMD_MAP_CHARS];
    uint32_t bits = 0u;
    uint32_t i;

    if ((NULL == p_out) || !cmd_map_get(chars))
    {
        return false;
    }
    for (i = 0u; i < CMD_MAP_CHARS; i++)
    {
        bits |= (uint32_t)charset_index((uint8_t)chars[i]) << (6u * i);
    }
    p_out[0] = (uint8_t)bits;
    p_out[1] = (uint8_t)(bits >> 8);
    p_out[2] = (uint8_t)(bits >> 16);
    return true;
}

uint8_t
cmd_map_lookup(uint8_t ch)
{
    uint32_t word = g_map;
    uint32_t i;

    if (0u != word)
    {
        for (i = 0u; i < CMD_MAP_CHARS; i++)
        {
            if (((word >> (8u * i)) & 0xFFu) == (uint32_t)ch)
            {
                return c_slot_cmd[i];
            }
        }
    }
    return CTRL_CMD_NONE;
}
