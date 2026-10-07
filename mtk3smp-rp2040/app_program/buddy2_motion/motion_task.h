/**
 * @file motion_task.h
 * @brief Motion control with speed calculation and distance tracking
 */

#ifndef MOTION_TASK_H
#define MOTION_TASK_H

#include <stdint.h>
#include <tk/tkernel.h>
#include <stdbool.h>

/**
 * @brief Motor command structure.
 */
typedef struct {
    int8_t left_speed;  /**< -100 to +100 % PWM duty (open loop) */
    int8_t right_speed; /**< -100 to +100 % PWM duty (open loop) */
} motor_cmd_t;

/**
 * @brief Odometry data structure.
 */
typedef struct {
    int32_t left_pulses;        /**< Cumulative left encoder pulses */
    int32_t right_pulses;       /**< Cumulative right encoder pulses */
    float left_speed_cm_s;      /**< Left wheel speed in cm/s */
    float right_speed_cm_s;     /**< Right wheel speed in cm/s */
    float distance_cm;          /**< Average distance traveled in cm */
    uint32_t timestamp_ms;      /**< Timestamp of measurement */
} odometry_t;

/**
 * @brief Encoder diagnostic counters (reset with motion_reset_odometry).
 *
 * For a clean quadrature signal, a_edges + b_edges equals the number of
 * valid steps, so it should match |count| when the wheel turned one way.
 * invalid counts polls where A and B both changed at once: a skipped
 * state (polling too slow) or noise. Those polls add 0 to count.
 */
typedef struct {
    int32_t  count;    /**< Signed quadrature count */
    uint32_t a_edges;  /**< Raw transitions seen on channel A */
    uint32_t b_edges;  /**< Raw transitions seen on channel B */
    uint32_t invalid;  /**< Polls where both channels changed */
    uint32_t polls;    /**< Encoder polls since reset (same for both) */
    uint32_t hw_edges; /**< Rising edges counted by the PWM slice */
    int32_t  hw_count; /**< hw_edges signed by commanded direction */
} enc_diag_t;

/**
 * @brief Initialize and start motion control task.
 */
ER motion_task_create(void);

/**
 * @brief Set motor speeds (non-blocking).
 * 
 * Values are % PWM duty, open loop; the motion task ramps toward them.
 */
ER motion_set_speed(int8_t left, int8_t right);

/**
 * @brief Get current odometry data.
 */
ER motion_get_odometry(odometry_t *p_odom);

/**
 * @brief Reset odometry counters to zero.
 */
void motion_reset_odometry(void);

/**
 * @brief Snapshot encoder diagnostic counters for both wheels.
 */
ER motion_get_encoder_diag(enc_diag_t *p_left, enc_diag_t *p_right);

/**
 * @brief Print encoder diagnostic counters for both wheels on one line.
 */
void motion_print_encoder_diag(const char *p_label);

/**
 * @brief Move forward a specified distance (blocking).
 * 
 * Speed is % PWM duty, open loop (no PID yet).
 */
ER motion_move_forward_cm(uint16_t distance_cm, uint8_t speed);

#endif /* MOTION_TASK_H */
