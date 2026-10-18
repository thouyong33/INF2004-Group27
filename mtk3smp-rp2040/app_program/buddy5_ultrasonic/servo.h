/**
 * @file servo.h
 * @brief Buddy 5: scan servo on GP12 (PWM slice 6, channel A), 50 Hz.
 *
 * The mount only allows a small sweep (< +/-30 deg) before the sensor hits
 * the chassis plates, so commands are clamped to soft limits measured with
 * the jog test, never the servo's full range.
 */

#ifndef BUDDY5_SERVO_H
#define BUDDY5_SERVO_H

#include <stdint.h>
#include <tk/tkernel.h>

#define SERVO_ABS_MIN_US   500u    /* hard clamp, the servo's widest pulse */
#define SERVO_ABS_MAX_US   2500u

/* Measured with the jog test (2026-10-09): larger pulse turns the sensor
 * LEFT. Each limit was marked a step before the mount touches a plate. */
#define SERVO_CENTRE_US    1540u   /* sensor straight ahead */
#define SERVO_LEFT_US      1720u   /* left limit, +180 us */
#define SERVO_RIGHT_US     1340u   /* right limit, -200 us */

/** @brief Start 50 Hz PWM on GP12 and move to start_us. */
void servo_init(uint16_t start_us);

/** @brief Set the soft limits (from the jog test); pulses are clamped to them. */
void servo_set_limits(uint16_t min_us, uint16_t max_us);

/** @brief Command a pulse width in microseconds (clamped). Returns the pulse used. */
uint16_t servo_set_us(uint16_t pulse_us);

/** @brief Last commanded pulse width. */
uint16_t servo_get_us(void);

/** @brief Stop pulses so the servo goes limp (no holding current). */
void servo_relax(void);

#endif /* BUDDY5_SERVO_H */
