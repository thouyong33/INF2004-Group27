#ifndef TELEMETRY_CODEC_H
#define TELEMETRY_CODEC_H

#include <stdint.h>
#include <stdbool.h>

#define TELEMETRY_V2_SIZE 10U
#define TELEMETRY_V2_VERSION 1U

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

bool telemetry_encode(
    const telemetry_data_t *data,
    uint8_t output[TELEMETRY_V2_SIZE]
);

bool telemetry_decode(
    const uint8_t input[TELEMETRY_V2_SIZE],
    telemetry_data_t *data
);

#endif