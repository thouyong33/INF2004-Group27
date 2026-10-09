/**
 * @file ultrasonic.h
 * @brief Buddy 5: HC-SR04 range sensor (TRIG GP7, ECHO GP28).
 *
 * The sensor is powered from the Robo Pico's 3V3 Grove strip, so ECHO
 * swings 0..3.3 V and can go straight into GP28.
 *
 * Echo timing polls the RP2040 hardware microsecond timer (no
 * interrupts). A reading blocks the caller for up to the echo timeout, so
 * call it from a task that may busy-wait, and give that task a priority at
 * least as high as anything that must not stretch the measured pulse.
 */

#ifndef BUDDY5_ULTRASONIC_H
#define BUDDY5_ULTRASONIC_H

#include <stdint.h>
#include <tk/tkernel.h>

/** @brief Start the 1 MHz hardware timer and set up the TRIG/ECHO pins. */
void ultrasonic_init(void);

/** @brief Microseconds from the hardware timer (wraps every ~71 min). */
uint32_t us_now(void);

/** @brief Busy-wait for us microseconds on the hardware timer. */
void us_delay(uint32_t us);

/**
 * @brief One measurement.
 * @param p_echo_us echo pulse width in microseconds
 * @return E_OK, or E_TMOUT if no echo came back (nothing in range, or a
 *         soft/angled target) or ECHO was stuck high from the last ping
 */
ER ultrasonic_read_us(uint32_t *p_echo_us);

/**
 * @brief Echo time to distance with the speed of sound at ~20 C,
 * before any calibration: mm = us x 343 / 2000.
 */
int32_t ultrasonic_us_to_mm(uint32_t echo_us);

#endif /* BUDDY5_ULTRASONIC_H */
