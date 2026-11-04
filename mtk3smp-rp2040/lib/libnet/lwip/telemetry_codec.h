/**
 * @file telemetry_codec.h
 * @brief Buddy 1: Telemetry V2 packet (88 bits, 11 bytes).
 */

#ifndef TELEMETRY_CODEC_H
#define TELEMETRY_CODEC_H

#include <stdint.h>
#include <stdbool.h>

/* 88 bits: state 4 bits (9 controller states), barcode command 3 bits
 * (5 commands), faults 8 bits (8 fault bits), 3 reserved bits to fill
 * the 11th byte. The 2-bit version field tells a decoder which layout
 * it holds (2 since the 31 Oct integration). */
#define TELEMETRY_V2_SIZE 11U
#define TELEMETRY_V2_VERSION 2U

typedef struct
{
    uint8_t version;
    uint8_t state;
    int16_t left_speed_mm_s;
    int16_t right_speed_mm_s;
    uint16_t distance_mm;
    uint8_t barcode_command;
    bool barcode_valid;
    uint8_t max_hump_mm;
    bool obstacle_detected;
    uint16_t obstacle_distance_mm;
    uint8_t fault_flags;
    uint8_t heartbeat;
} telemetry_data_t;

/** @brief Pack p_data into output; false if a field is out of range. */
bool telemetry_encode(const telemetry_data_t *p_data,
                      uint8_t output[TELEMETRY_V2_SIZE]);

/** @brief Unpack input into p_data; false if it is not a V2 packet. */
bool telemetry_decode(const uint8_t input[TELEMETRY_V2_SIZE],
                      telemetry_data_t *p_data);

#endif /* TELEMETRY_CODEC_H */
