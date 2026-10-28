/**
 * @file faults.h
 * @brief Shared fault bitmask: one bit per fault, one byte for all of them
 *        (Tribunal 1 change requirement: minimum-bit data).
 *
 * Any task may raise or clear a fault. Set and clear are read-modify-write
 * on the shared byte, so both run inside a short critical section (DI/EI,
 * plus a spinlock on an SMP build) and never lose another task's bit.
 * Reading is a single byte load and needs no lock.
 */

#ifndef COMMON_FAULTS_H
#define COMMON_FAULTS_H

#include <stdint.h>

#define FAULT_MOTOR_STALL    (1u << 0)  /**< both wheels well below setpoint */
#define FAULT_ENC_LEFT       (1u << 1)  /**< left encoder silent while driven */
/* *< right encoder silent while driven */
#define FAULT_ENC_RIGHT      (1u << 2)
#define FAULT_IMU            (1u << 3)  /**< IMU not answering */
#define FAULT_NO_ECHO        (1u << 4)  /**< ultrasonic gave no echo */
#define FAULT_WIFI_LOSS      (1u << 5)  /**< Buddy 1: link to the broker lost */
#define FAULT_SCAN_TIMEOUT   (1u << 6)  /**< obstacle scan ran over its limit */
/* *< Buddy 3: line lost, on-spot search failed */
#define FAULT_LINE_LOST      (1u << 7)

/** Faults that make the motion task stop and refuse to drive until cleared */
#define FAULT_MOTION_MASK    (FAULT_MOTOR_STALL | FAULT_ENC_LEFT \
                              | FAULT_ENC_RIGHT)

/** @brief Raise the given fault bits (others are kept). */
/* also posts EVT_FAULT for new bits; tasks only */
void fault_set(uint8_t bits);

/** @brief Clear the given fault bits (others are kept). */
void fault_clear(uint8_t bits);

/** @brief Current fault bits. */
uint8_t fault_get(void);

#endif /* COMMON_FAULTS_H */
