/*!
 * @file gpio.h
 *
 * @brief GPIO control interface for Raspberry Pi Pico W
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2024 Your Name. All rights reserved.
 */

#ifndef GPIO_H
#define GPIO_H

#include <stdint.h>
#include <stdbool.h>

// GPIO pin definitions for motor control
#define MOTOR1_PIN_A    (8u)
#define MOTOR1_PIN_B    (9u)
#define MOTOR2_PIN_A    (10u)
#define MOTOR2_PIN_B    (11u)

// GPIO pin definitions for ultrasonic sensor
#define ULTRASONIC_TRIG_PIN    (7u)
#define ULTRASONIC_ECHO_PIN    (28u)

// Public API
void gpio_init_pin(uint8_t pin, bool b_output);
void gpio_set_high(uint8_t pin);
void gpio_set_low(uint8_t pin);
bool gpio_read(uint8_t pin);

#endif /* GPIO_H */

/*** end of file ***/
