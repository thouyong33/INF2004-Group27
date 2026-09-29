/*!
 * @file motor.c
 *
 * @brief DC motor control implementation
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2024 Your Name. All rights reserved.
 */

#include "motor.h"
#include "gpio.h"

/*!
 * @brief Initialize motor control GPIO pins
 */
void
motor_init (void)
{
    // Configure motor 1 pins as outputs
    gpio_init_pin(MOTOR1_PIN_A, true);
    gpio_init_pin(MOTOR1_PIN_B, true);
    
    // Configure motor 2 pins as outputs
    gpio_init_pin(MOTOR2_PIN_A, true);
    gpio_init_pin(MOTOR2_PIN_B, true);
    
    // Ensure motors start in stopped state
    motor_stop_all();
}

/*!
 * @brief Stop both motors (brake mode)
 */
void
motor_stop_all (void)
{
    gpio_set_low(MOTOR1_PIN_A);
    gpio_set_low(MOTOR1_PIN_B);
    gpio_set_low(MOTOR2_PIN_A);
    gpio_set_low(MOTOR2_PIN_B);
}

/*!
 * @brief Drive both motors forward
 */
void
motor_forward (void)
{
    // Motor 1 forward
    gpio_set_high(MOTOR1_PIN_A);
    gpio_set_low(MOTOR1_PIN_B);
    
    // Motor 2 forward
    gpio_set_high(MOTOR2_PIN_A);
    gpio_set_low(MOTOR2_PIN_B);
}

/*!
 * @brief Drive both motors backward
 */
void
motor_backward (void)
{
    // Motor 1 backward
    gpio_set_low(MOTOR1_PIN_A);
    gpio_set_high(MOTOR1_PIN_B);
    
    // Motor 2 backward
    gpio_set_low(MOTOR2_PIN_A);
    gpio_set_high(MOTOR2_PIN_B);
}

/*** end of file ***/
