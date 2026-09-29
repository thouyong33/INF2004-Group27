/*!
 * @file motor.h
 *
 * @brief DC motor control interface
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2024 Your Name. All rights reserved.
 */

#ifndef MOTOR_H
#define MOTOR_H

// Motor control API
void motor_init(void);
void motor_stop_all(void);
void motor_forward(void);
void motor_backward(void);

#endif /* MOTOR_H */

/*** end of file ***/
