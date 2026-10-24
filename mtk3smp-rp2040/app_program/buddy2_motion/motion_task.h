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
 * @brief Hardware edge counters for one wheel (reset with
 * motion_reset_odometry). Read as one consistent snapshot.
 */
typedef struct {
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
 * @brief True while driving under PI at a constant, non-zero target with
 * both setpoints settled (not speeding up, slowing down or stopped).
 * Speed changes read as tilt on the accelerometer, so Buddy 4 only trusts
 * pitch for hump detection while this is true.
 */
bool motion_is_steady(void);

/**
 * @brief Stop both wheels now, from any task. Bypasses the command queue
 * (and discards what is queued), so it works even while another task owns
 * the motors, e.g. during an obstacle scan. Under PI the wheels brake along
 * the normal slew; in open loop the duty ramps to zero.
 */
void motion_stop(void);

/**
 * @brief Tell the motion task the car is on a hump (Buddy 4). While set,
 * the stall and encoder checks are masked: wheels slow and slip there.
 * Driving with the mask on for more than 4 s raises FAULT_MOTOR_STALL
 * (a car stuck on the ramp with a wheel spinning).
 */
void motion_set_hump_mask(bool on_hump);

/**
 * @brief Passes of the 1 ms motion loop per second, over the last second.
 * About 995 or more means the loop keeps its period (timing evidence).
 */
uint32_t motion_loop_rate(void);

/**
 * @brief CPU used by the motion loop over the last second: microseconds of
 * work per second (divide by 10000 for percent) and the longest pass.
 * Measured with the 1 MHz hardware timer, which runs once
 * ultrasonic_init() has started it; kernel tick and task switch time are
 * not included.
 */
void motion_get_cpu(uint32_t *p_busy_us_per_s, uint32_t *p_max_pass_us);

/** @brief Deepest the motion task's stack has reached, bytes. */
uint32_t motion_stack_used(void);

/** @brief Size of the motion task's stack, bytes. */
uint32_t motion_stack_size(void);

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
 * @brief Snapshot both wheels' hardware counters in one go.
 */
ER motion_get_encoder_diag(enc_diag_t *p_left, enc_diag_t *p_right);

/**
 * @brief Print both wheels' hardware counters on one line. Not while the
 * motors run under a higher-priority caller: printing is slow.
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

/**
 * @brief Turn on the spot by cdeg hundredths of a degree (left positive)
 * at speed_mm_s per wheel, measured by the encoders (blocking, no prints).
 * @return the angle actually turned, x0.01 deg (accuracy ~1-2 deg)
 */
int32_t motion_turn_cdeg(int32_t cdeg, uint16_t speed_mm_s);

#endif /* MOTION_TASK_H */
