/** @file motor.h
 *
 * @brief DC motor control interface for basic robot movement.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2024. All rights reserved.
 */

#ifndef MOTOR_H
#define MOTOR_H

void motor_init(void);
void motor_stop_all(void);
void motor_forward(void);
void motor_turn_right(void);

#endif /* MOTOR_H */

/*** end of file ***/
