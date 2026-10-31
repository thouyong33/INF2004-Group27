#include <stdio.h>
#include "telemetry_codec.h"

int main(void)
{
    telemetry_data_t original = {
        .version = TELEMETRY_V2_VERSION,
        .state = 2,
        .left_speed_mm_s = 300,
        .right_speed_mm_s = 295,
        .distance_mm = 1500,
        .barcode_command = 1,
        .barcode_valid = true,
        .max_hump_mm = 24,
        .obstacle_detected = true,
        .obstacle_distance_mm = 200,
        .fault_flags = 0,
        .heartbeat = 5
    };

    uint8_t packet[TELEMETRY_V2_SIZE];
    telemetry_data_t decoded = {0};

    if (!telemetry_encode(&original, packet)) {
        printf("Encoding failed!\n");
        return 1;
    }

    printf("Encoded packet: ");
    for (unsigned int i = 0; i < TELEMETRY_V2_SIZE; i++) {
        printf("%02X ", packet[i]);
    }
    printf("\n");

    if (!telemetry_decode(packet, &decoded)) {
        printf("Decoding failed!\n");
        return 1;
    }

    printf("Robot state: %u\n", decoded.state);
    printf("Left speed: %d mm/s\n", decoded.left_speed_mm_s);
    printf("Right speed: %d mm/s\n", decoded.right_speed_mm_s);
    printf("Distance: %u mm\n", decoded.distance_mm);
    printf("Barcode: %u (valid=%u)\n",
           decoded.barcode_command, decoded.barcode_valid);
    printf("Maximum hump: %u mm\n", decoded.max_hump_mm);
    printf("Obstacle: %u (distance=%u mm)\n",
           decoded.obstacle_detected, decoded.obstacle_distance_mm);
    printf("Fault flags: %u\n", decoded.fault_flags);
    printf("Heartbeat: %u\n", decoded.heartbeat);


    /* Test maximum supported values */
telemetry_data_t max_test = {
    .version = TELEMETRY_V2_VERSION,
    .state = 15,
    .left_speed_mm_s = 550,
    .right_speed_mm_s = 550,
    .distance_mm = 65535,
    .barcode_command = 7,
    .barcode_valid = true,
    .max_hump_mm = 255,
    .obstacle_detected = true,
    .obstacle_distance_mm = 4095,
    .fault_flags = 255,
    .heartbeat = 255
};

if (!telemetry_encode(&max_test, packet) ||
    !telemetry_decode(packet, &decoded) ||
    decoded.state != 15 ||
    decoded.left_speed_mm_s != 550 ||
    decoded.right_speed_mm_s != 550 ||
    decoded.distance_mm != 65535 ||
    decoded.barcode_command != 7 ||
    !decoded.barcode_valid ||
    decoded.max_hump_mm != 255 ||
    !decoded.obstacle_detected ||
    decoded.obstacle_distance_mm != 4095 ||
    decoded.fault_flags != 255 ||
    decoded.heartbeat != 255) {
    printf("MAXIMUM TEST FAILED\n");
    return 1;
}

printf("MAXIMUM TEST PASSED\n");

/* Test negative wheel speed boundary */
max_test.left_speed_mm_s = -550;
max_test.right_speed_mm_s = -550;

if (!telemetry_encode(&max_test, packet) ||
    !telemetry_decode(packet, &decoded) ||
    decoded.left_speed_mm_s != -550 ||
    decoded.right_speed_mm_s != -550) {
    printf("NEGATIVE SPEED TEST FAILED\n");
    return 1;
}

printf("NEGATIVE SPEED TEST PASSED\n");

/* Test rejection of an invalid value */
max_test.left_speed_mm_s = 551;

if (telemetry_encode(&max_test, packet)) {
    printf("INVALID RANGE TEST FAILED\n");
    return 1;
}

printf("INVALID RANGE TEST PASSED\n");

    return 0;
}