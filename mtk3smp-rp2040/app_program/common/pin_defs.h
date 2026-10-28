#ifndef COMMON_PIN_DEFS_H
#define COMMON_PIN_DEFS_H

/**
 * @file pin_defs.h
 * @brief Pin definitions for the robot hardware.
 */

/* Motor control pins (TB6612FNG on Robo Pico) */
#define MOTOR_LEFT_FWD   8u  /* M1A (PWM4A) */
#define MOTOR_LEFT_REV   9u  /* M1B (PWM4B) */
#define MOTOR_RIGHT_FWD 10u  /* M2A (PWM5A) *** THIS IS GP10, NOT GP11! *** */
#define MOTOR_RIGHT_REV 11u  /* M2B (PWM5B) *** THIS IS GP11, NOT GP10! *** */

/* Encoder pins */
#define ENC_LEFT_A      14u
#define ENC_LEFT_B      19u
#define ENC_RIGHT_A     15u
#define ENC_RIGHT_B     17u

/* IR sensor pins */
#define IR_CENTER_DOUT   4u
#define IR_LEFT_DOUT     6u
#define IR_RIGHT_DOUT   26u
#define IR_LEFT_AOUT    27u  /* ADC1: left IR analog out (barcode) */

/* Ultrasonic sensor pins */
#define ULTRASONIC_TRIG  7u
#define ULTRASONIC_ECHO 28u

/* Servo control */
#define SERVO_PWM       12u

/* I2C pins */
#define I2C_SDA          2u
#define I2C_SCL          3u

/* PWM slice allocation */
#define PWM_SLICE_MOTOR_L  4u
#define PWM_SLICE_MOTOR_R  5u

#endif /* COMMON_PIN_DEFS_H */
