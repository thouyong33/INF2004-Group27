/** @file motor.c
 *
 * @brief DC motor control implementation for basic robot movement.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2024. All rights reserved.
 */

#include "pico/stdlib.h"
#include "motor.h"

#define MOTOR1_PIN_A    (8u)
#define MOTOR1_PIN_B    (9u)
#define MOTOR2_PIN_A    (10u)
#define MOTOR2_PIN_B    (11u)

/*!
 * @brief Initialize motor GPIO pins.
 */
void
motor_init (void)
{
    gpio_init(MOTOR1_PIN_A);
    gpio_set_dir(MOTOR1_PIN_A, GPIO_OUT);

    gpio_init(MOTOR1_PIN_B);
    gpio_set_dir(MOTOR1_PIN_B, GPIO_OUT);

    gpio_init(MOTOR2_PIN_A);
    gpio_set_dir(MOTOR2_PIN_A, GPIO_OUT);

    gpio_init(MOTOR2_PIN_B);
    gpio_set_dir(MOTOR2_PIN_B, GPIO_OUT);

    motor_stop_all();
}

/*!
 * @brief Stop both motors.
 */
void
motor_stop_all (void)
{
    gpio_put(MOTOR1_PIN_A, 0);
    gpio_put(MOTOR1_PIN_B, 0);
    gpio_put(MOTOR2_PIN_A, 0);
    gpio_put(MOTOR2_PIN_B, 0);
}

/*!
 * @brief Drive both motors forward.
 *
 * @par
 * WARNING: If the robot moves backward, your motor polarity is reversed.
 * Swap the motor wires at the motor terminals, not in software first.
 */
void
motor_forward (void)
{
    gpio_put(MOTOR1_PIN_A, 1);
    gpio_put(MOTOR1_PIN_B, 0);

    gpio_put(MOTOR2_PIN_A, 1);
    gpio_put(MOTOR2_PIN_B, 0);
}

/*!
 * @brief Turn the robot to the right.
 *
 * @par
 * NOTE: This implementation drives the left motor forward and stops
 * the right motor. If your chassis responds poorly, this strategy can
 * be changed later.
 */
void
motor_turn_right (void)
{
    gpio_put(MOTOR1_PIN_A, 1);
    gpio_put(MOTOR1_PIN_B, 0);

    gpio_put(MOTOR2_PIN_A, 0);
    gpio_put(MOTOR2_PIN_B, 0);
}

/*** end of file ***/
