/**
 * @file telemetry_codec.c
 * @brief Buddy 1: Telemetry V2 packing and unpacking.
 */

#include "telemetry_codec.h"
#include <stddef.h>
#include <string.h>

/*
 * Telemetry V2 - 88-bit binary format (version 2; see the header).
 *
 * Fields are packed consecutively, least-significant bit first.
 * Signed wheel speeds use an offset of +550.
 */

/* Write a value into the next available bits. */
static void
write_bits(uint8_t *p_buffer, uint32_t value, unsigned int *p_bit_pos,
           unsigned int width)
{
    unsigned int i;
    unsigned int pos;

    for (i = 0U; i < width; i++)
    {
        pos = *p_bit_pos + i;

        if (0U != ((value >> i) & 1U))
        {
            p_buffer[pos / 8U] |= (uint8_t)(1U << (pos % 8U));
        }
    }

    *p_bit_pos += width;
}

/* Read a value from the next available bits. */
static uint32_t
read_bits(const uint8_t *p_buffer, unsigned int *p_bit_pos,
          unsigned int width)
{
    uint32_t value = 0U;
    unsigned int i;
    unsigned int pos;

    for (i = 0U; i < width; i++)
    {
        pos = *p_bit_pos + i;

        if (0U != (p_buffer[pos / 8U] & (uint8_t)(1U << (pos % 8U))))
        {
            value |= (uint32_t)1U << i;
        }
    }

    *p_bit_pos += width;
    return value;
}

bool
telemetry_encode(const telemetry_data_t *p_data,
                 uint8_t output[TELEMETRY_V2_SIZE])
{
    unsigned int bit = 0U;

    if ((NULL == p_data) || (NULL == output))
    {
        return false;
    }

    /* Each field must fit its bits */
    if ((TELEMETRY_V2_VERSION != p_data->version)
        || (p_data->state > 15U)
        || (p_data->left_speed_mm_s < -550)
        || (p_data->left_speed_mm_s > 550)
        || (p_data->right_speed_mm_s < -550)
        || (p_data->right_speed_mm_s > 550)
        || (p_data->barcode_command > 7U)
        || (p_data->obstacle_distance_mm > 4095U))
    {
        return false;
    }

    memset(output, 0, TELEMETRY_V2_SIZE);

    write_bits(output, p_data->version, &bit, 2U);
    write_bits(output, p_data->state, &bit, 4U);

    /* Offset encoding: -550 becomes 0, +550 becomes 1100. */
    write_bits(output, (uint32_t)(p_data->left_speed_mm_s + 550), &bit,
               11U);
    write_bits(output, (uint32_t)(p_data->right_speed_mm_s + 550), &bit,
               11U);

    write_bits(output, p_data->distance_mm, &bit, 16U);
    write_bits(output, p_data->barcode_command, &bit, 3U);
    write_bits(output, p_data->barcode_valid ? 1U : 0U, &bit, 1U);
    write_bits(output, p_data->max_hump_mm, &bit, 8U);
    write_bits(output, p_data->obstacle_detected ? 1U : 0U, &bit, 1U);
    write_bits(output, p_data->obstacle_distance_mm, &bit, 12U);
    write_bits(output, p_data->fault_flags, &bit, 8U);
    write_bits(output, p_data->heartbeat, &bit, 8U);
    write_bits(output, 0U, &bit, 3U);              /* reserved */

    return (TELEMETRY_V2_SIZE * 8U) == bit;
}

bool
telemetry_decode(const uint8_t input[TELEMETRY_V2_SIZE],
                 telemetry_data_t *p_data)
{
    unsigned int bit = 0U;
    telemetry_data_t decoded = {0};
    uint32_t left;
    uint32_t right;

    if ((NULL == input) || (NULL == p_data))
    {
        return false;
    }

    decoded.version = (uint8_t)read_bits(input, &bit, 2U);
    decoded.state = (uint8_t)read_bits(input, &bit, 4U);

    left = read_bits(input, &bit, 11U);
    right = read_bits(input, &bit, 11U);

    if ((TELEMETRY_V2_VERSION != decoded.version) || (left > 1100U)
        || (right > 1100U))
    {
        return false;
    }

    decoded.left_speed_mm_s = (int16_t)((int32_t)left - 550);
    decoded.right_speed_mm_s = (int16_t)((int32_t)right - 550);

    decoded.distance_mm = (uint16_t)read_bits(input, &bit, 16U);
    decoded.barcode_command = (uint8_t)read_bits(input, &bit, 3U);
    decoded.barcode_valid = (0U != read_bits(input, &bit, 1U));
    decoded.max_hump_mm = (uint8_t)read_bits(input, &bit, 8U);
    decoded.obstacle_detected = (0U != read_bits(input, &bit, 1U));
    decoded.obstacle_distance_mm = (uint16_t)read_bits(input, &bit, 12U);
    decoded.fault_flags = (uint8_t)read_bits(input, &bit, 8U);
    decoded.heartbeat = (uint8_t)read_bits(input, &bit, 8U);
    (void)read_bits(input, &bit, 3U);              /* reserved */

    if ((TELEMETRY_V2_SIZE * 8U) != bit)
    {
        return false;
    }

    *p_data = decoded;
    return true;
}
