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
    int16_t left_speed;   /**< % duty (open loop) or mm/s (closed loop) */
    int16_t right_speed;  /**< % duty (open loop) or mm/s (closed loop) */
    uint8_t closed_loop;  /**< 0 = open-loop duty, 1 = PI speed control */
} motor_cmd_t;

/**
 * @brief One closed-loop control period, recorded by the motion task.
 * Speeds are in encoder edges/s, duty in tenths of a percent.
 */
typedef struct {
    uint16_t t_ms;     /**< ms since motion_trace_start() */
    int16_t  sp_l;     /**< Left setpoint (slewed) */
    int16_t  meas_l;   /**< Left measured speed */
    int16_t  duty_l;   /**< Left PWM duty x10 */
    int16_t  sp_r;
    int16_t  meas_r;
    int16_t  duty_r;
} motion_trace_t;

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
 * @brief Set wheel speeds in mm/s under closed-loop PI control (non-blocking).
 *
 * Each wheel has its own PI loop with feed-forward from the open-loop duty
 * sweep. Targets are clamped to +/- the max PI speed (left motor headroom).
 */
ER motion_set_velocity(int16_t left_mm_s, int16_t right_mm_s);

/**
 * @brief Start recording one motion_trace_t per control period into RAM.
 * Recording stops when the buffer is full.
 */
void motion_trace_start(void);

/**
 * @brief Stop recording and print the trace as CSV lines prefixed "[TRACE]".
 * Call only once the motors are stopped: printing is slow.
 */
void motion_trace_dump(void);

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
 * @brief Drive straight forward a specified distance (blocking).
 *
 * Runs under PI speed control at speed_mm_s (clamped to 50..550 mm/s) and
 * starts braking early by the computed stopping distance. Prints the
 * encoder-measured distance and error once the wheels have stopped.
 */
ER motion_move_forward_cm(uint16_t distance_cm, uint16_t speed_mm_s);

#endif /* MOTION_TASK_H */
