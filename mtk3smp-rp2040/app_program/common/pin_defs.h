#ifndef COMMON_PIN_DEFS_H
#define COMMON_PIN_DEFS_H

/**
 * @file pin_defs.h
 * @brief Pin definitions for the robot hardware.
 */

// Motor control pins (TB6612FNG on Robo Pico)
#define MOTOR_LEFT_FWD   8   // M1A (PWM4A)
#define MOTOR_LEFT_REV   9   // M1B (PWM4B)
#define MOTOR_RIGHT_FWD 10   // M2A (PWM5A) *** THIS IS GP10, NOT GP11! ***
#define MOTOR_RIGHT_REV 11   // M2B (PWM5B) *** THIS IS GP11, NOT GP10! ***

// Encoder pins
#define ENC_LEFT_A      14
#define ENC_LEFT_B      19
#define ENC_RIGHT_A     15
#define ENC_RIGHT_B     17

// IR sensor pins
#define IR_CENTER_DOUT   4
#define IR_LEFT_DOUT     6
#define IR_RIGHT_DOUT   26

// Ultrasonic sensor pins
#define ULTRASONIC_TRIG  7
#define ULTRASONIC_ECHO 28

// Servo control
#define SERVO_PWM       12

// I2C pins
#define I2C_SDA          2
#define I2C_SCL          3

// PWM slice allocation
#define PWM_SLICE_MOTOR_L  4
#define PWM_SLICE_MOTOR_R  5

#endif /* COMMON_PIN_DEFS_H */
