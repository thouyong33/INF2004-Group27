/*!
 * @file ultrasonic.h
 *
 * @brief HC-SR04 ultrasonic sensor interface
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2024 Your Name. All rights reserved.
 */

#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <stdint.h>

// Measurement parameters
#define ULTRASONIC_TIMEOUT_US       (30000u)  // 30ms timeout
#define ULTRASONIC_MIN_DISTANCE_CM  (2u)      // Minimum measurable distance
#define ULTRASONIC_MAX_DISTANCE_CM  (400u)    // Maximum measurable distance

// Public API
void     ultrasonic_init(void);
uint16_t ultrasonic_get_distance_cm(void);

#endif /* ULTRASONIC_H */

/*** end of file ***/
