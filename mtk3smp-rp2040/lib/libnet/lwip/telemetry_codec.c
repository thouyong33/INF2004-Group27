
#include "telemetry_codec.h"
#include <stddef.h>
#include <string.h>

/*
 * Telemetry V2 - 88-bit binary format (version 2; see the header).
 *
 * Fields are packed consecutively, least-significant bit first.
 * Signed wheel speeds use an offset of +550.
 *
 * Actual ranges must be confirmed before final integration.
 */

/* Write a value into the next available bits. */
static void write_bits(
    uint8_t *buffer,
    uint32_t value,
    unsigned int *bit_pos,
    unsigned int width)
{
    for (unsigned int i = 0; i < width; i++) {
        unsigned int pos = *bit_pos + i;

        if ((value >> i) & 1U) {
            buffer[pos / 8U] |=
                (uint8_t)(1U << (pos % 8U));
        }
    }

    *bit_pos += width;
}

/* Read a value from the next available bits. */
static uint32_t read_bits(
    const uint8_t *buffer,
    unsigned int *bit_pos,
    unsigned int width)
{
    uint32_t value = 0U;

    for (unsigned int i = 0; i < width; i++) {
        unsigned int pos = *bit_pos + i;

        if (buffer[pos / 8U] &
            (uint8_t)(1U << (pos % 8U))) {
            value |= (uint32_t)1U << i;
        }
    }

    *bit_pos += width;
    return value;
}

bool telemetry_encode(
    const telemetry_data_t *data,
    uint8_t output[TELEMETRY_V2_SIZE])
{
    if (data == NULL || output == NULL) {
        return false;
    }

    /* Validate provisional application ranges. */
    if (data->version != TELEMETRY_V2_VERSION ||
        data->state > 15U ||
        data->left_speed_mm_s < -550 ||
        data->left_speed_mm_s > 550 ||
        data->right_speed_mm_s < -550 ||
        data->right_speed_mm_s > 550 ||
        data->barcode_command > 7U ||
        data->obstacle_distance_mm > 4095U) {
        return false;
    }

    memset(output, 0, TELEMETRY_V2_SIZE);

    unsigned int bit = 0U;

    write_bits(output, data->version, &bit, 2U);
    write_bits(output, data->state, &bit, 4U);

    /* Offset encoding: -550 becomes 0, +550 becomes 1100. */
    write_bits(output,
               (uint32_t)(data->left_speed_mm_s + 550),
               &bit, 11U);

    write_bits(output,
               (uint32_t)(data->right_speed_mm_s + 550),
               &bit, 11U);

    write_bits(output, data->distance_mm, &bit, 16U);
    write_bits(output, data->barcode_command, &bit, 3U);
    write_bits(output, data->barcode_valid ? 1U : 0U, &bit, 1U);
    write_bits(output, data->max_hump_mm, &bit, 8U);
    write_bits(output, data->obstacle_detected ? 1U : 0U, &bit, 1U);
    write_bits(output, data->obstacle_distance_mm, &bit, 12U);
    write_bits(output, data->fault_flags, &bit, 8U);
    write_bits(output, data->heartbeat, &bit, 8U);
    write_bits(output, 0U, &bit, 3U);              /* reserved */

    return bit == (TELEMETRY_V2_SIZE * 8U);
}

bool telemetry_decode(
    const uint8_t input[TELEMETRY_V2_SIZE],
    telemetry_data_t *data)
{
    if (input == NULL || data == NULL) {
        return false;
    }

    unsigned int bit = 0U;
    telemetry_data_t decoded = {0};

    decoded.version = (uint8_t)read_bits(input, &bit, 2U);
    decoded.state = (uint8_t)read_bits(input, &bit, 4U);

    uint32_t left = read_bits(input, &bit, 11U);
    uint32_t right = read_bits(input, &bit, 11U);

    if (decoded.version != TELEMETRY_V2_VERSION ||
        left > 1100U || right > 1100U) {
        return false;
    }

    decoded.left_speed_mm_s = (int16_t)((int32_t)left - 550);
    decoded.right_speed_mm_s = (int16_t)((int32_t)right - 550);

    decoded.distance_mm = (uint16_t)read_bits(input, &bit, 16U);
    decoded.barcode_command = (uint8_t)read_bits(input, &bit, 3U);
    decoded.barcode_valid = read_bits(input, &bit, 1U) != 0U;
    decoded.max_hump_mm = (uint8_t)read_bits(input, &bit, 8U);
    decoded.obstacle_detected = read_bits(input, &bit, 1U) != 0U;
    decoded.obstacle_distance_mm =
        (uint16_t)read_bits(input, &bit, 12U);
    decoded.fault_flags = (uint8_t)read_bits(input, &bit, 8U);
    decoded.heartbeat = (uint8_t)read_bits(input, &bit, 8U);
    (void)read_bits(input, &bit, 3U);              /* reserved */

    if (bit != (TELEMETRY_V2_SIZE * 8U)) {
        return false;
    }

    *data = decoded;
    return true;
}
