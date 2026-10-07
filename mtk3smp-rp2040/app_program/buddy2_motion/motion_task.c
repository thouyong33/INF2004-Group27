/**
 * @file motion_task.c
 * @brief Motion control with speed calculation and distance tracking
 *
 * Current implementation notes:
 * - Motor drive is open-loop 20 kHz PWM: command -100..100 = % duty,
 *   ramped at MOTOR_RAMP_STEP_X10 per loop to limit inrush. No PID yet.
 * - Odometry uses PWM-slice hardware edge counters (one channel per
 *   wheel), signed by the applied motor direction.
 * - Distance move uses encoder count progress only.
 */

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include <sys/sysdef.h>

#include "../common/pin_defs.h"
#include "motion_task.h"

/*----------------------------------------------------------------------------
 * Calibrated Encoder / Wheel Constants
 *---------------------------------------------------------------------------*/
/* Hardware edge counter: rising edges of one encoder channel per wheel turn.
 * Measured 2026-10-07, 10 hand turns x 2 per wheel: L 6301/6363, R 6349/6333. */
#define EDGES_PER_REV             634u
#define WHEEL_DIAMETER_MM         65u
#define WHEEL_CIRCUMFERENCE_MM    204u

/* 204 mm / 634 edges = 0.322 mm per edge */
#define UM_PER_EDGE               322L

/*----------------------------------------------------------------------------
 * Configuration Constants
 *---------------------------------------------------------------------------*/
#define TASK_PERIOD_MS            1u
#define SPEED_CALC_INTERVAL_MS    100u
#define ODOM_PRINT_INTERVAL_MS    500u
#define MOTOR_BRAKE_DELAY_MS      500u
#define GPIO_FUNCTION_SIO         5u
#define MOTION_MSGBUF_DEPTH       4u
#define MOTION_STACK_SIZE         4096u

/*----------------------------------------------------------------------------
 * SIO GPIO Registers
 *---------------------------------------------------------------------------*/
#ifndef GPIO_OUT_SET
#define GPIO_OUT_SET    (SIO_BASE + 0x014u)
#endif

#ifndef GPIO_OUT_CLR
#define GPIO_OUT_CLR    (SIO_BASE + 0x018u)
#endif

#ifndef GPIO_OE_SET
#define GPIO_OE_SET     (SIO_BASE + 0x024u)
#endif

/*----------------------------------------------------------------------------
 * Module State
 *---------------------------------------------------------------------------*/
static ID motion_task_id = 0;
static ID motion_msgbuf_id = 0;
static void *motion_task_stack = NULL;

/* Encoder state */
static volatile int32_t g_enc_left_count = 0;
static volatile int32_t g_enc_right_count = 0;
static uint8_t g_enc_left_last_state = 0u;
static uint8_t g_enc_right_last_state = 0u;

/* Encoder diagnostics: raw per-channel edges and invalid (both-changed) polls */
static volatile uint32_t g_enc_left_a_edges = 0u;
static volatile uint32_t g_enc_left_b_edges = 0u;
static volatile uint32_t g_enc_right_a_edges = 0u;
static volatile uint32_t g_enc_right_b_edges = 0u;
static volatile uint32_t g_enc_left_invalid = 0u;
static volatile uint32_t g_enc_right_invalid = 0u;
static volatile uint32_t g_enc_polls = 0u;

/* Hardware edge counters: a PWM slice in "B pin rising edge" mode counts
 * one encoder channel in hardware; the task only reads the counter. The
 * pin must be a PWM B pin (odd GPIO). Direction comes from the motor
 * command, so these are only signed correctly while the motor is driven. */
#define ENC_HW_LEFT_PIN           ENC_LEFT_B    /* GP19 = PWM1 B */
#define ENC_HW_RIGHT_PIN          ENC_RIGHT_A   /* GP15 = PWM7 B */
#define PWM_SLICE_OF(pin)         (((pin) >> 1) & 0x7u)
#define PWM_CSR_EN                (1u << 0)
#define PWM_CSR_DIVMODE_B_RISE    (2u << 4)
#define PWM_DIV_INT_1             (1u << 4)
#define PWM_SLICE_REG(slice, off) (PWM_BASE + ((slice) * 0x14u) + (off))

#if ((ENC_HW_LEFT_PIN & 1) == 0) || ((ENC_HW_RIGHT_PIN & 1) == 0)
#error "Hardware encoder pins must be PWM B pins (odd GPIO numbers)"
#endif

static uint16_t g_hw_left_last_ctr = 0u;
static uint16_t g_hw_right_last_ctr = 0u;
static volatile uint32_t g_hw_left_edges = 0u;
static volatile uint32_t g_hw_right_edges = 0u;
static volatile int32_t g_hw_left_count = 0;
static volatile int32_t g_hw_right_count = 0;
static int8_t g_hw_left_dir = 1;
static int8_t g_hw_right_dir = 1;

/* Motor PWM: Robo Pico driver PWM is specified at 20 kHz */
#define MOTOR_PWM_FREQ_HZ         20000u
#define MOTOR_PWM_TOP             ((SYSCLK * 1000000u / MOTOR_PWM_FREQ_HZ) - 1u)
#define MOTOR_DUTY_MAX_X10        1000
#define MOTOR_RAMP_STEP_X10       5     /* 0.5 % per loop, 0-100 % in ~200 loops */

static int16_t g_applied_left_x10 = 0;
static int16_t g_applied_right_x10 = 0;

/* Duty the ramp moves toward: the open-loop command, or the PI output */
static int16_t g_target_left_x10 = 0;
static int16_t g_target_right_x10 = 0;

/*----------------------------------------------------------------------------
 * Closed-loop speed control: per-wheel PI with feed-forward
 *
 * Feed-forward slopes come from the lifted duty sweep (2026-10-07): both
 * motors are linear at ~25 edges/s per 1 % duty with almost no dead band.
 * The left motor saturates at ~750 mm/s (lifted), so targets are capped
 * below that to leave the PI headroom to hold both wheels equal.
 *---------------------------------------------------------------------------*/
#define PID_PERIOD_MS             20u
#define PID_MAX_SPEED_MM_S        550
#define FF_EPS_PER_PCT_X10_L      245L  /* 24.5 edges/s per 1 % duty */
#define FF_EPS_PER_PCT_X10_R      258L  /* 25.8 edges/s per 1 % duty */
#define PID_KP_X1000              300L  /* duty_x10 per (edge/s) of error */
#define PID_KI_X1000              60L   /* duty_x10 per (edge/s) per period */
#define PID_I_LIMIT_X10           300L  /* integral clamp: +/- 30 % duty */
#define PID_SLEW_EPS              50L   /* setpoint change per period */

typedef struct {
    int32_t target_eps;    /* requested speed, edges/s */
    int32_t setpoint_eps;  /* slewed toward target to limit inrush */
    int32_t meas_eps;      /* measured over the last period */
    int32_t integ_x10;     /* integral term, duty x10 */
    int32_t last_count;    /* hw count at the last period */
    int16_t duty_x10;      /* PI output */
} wheel_pi_t;

static bool g_closed_loop = false;
static wheel_pi_t g_pi_left;
static wheel_pi_t g_pi_right;
static uint32_t g_last_pi_ms = 0u;

/* RAM trace of control periods, printed after a test */
#define TRACE_LEN                 300u  /* 6 s at 20 ms */
static motion_trace_t g_trace[TRACE_LEN];
static volatile uint32_t g_trace_n = 0u;
static volatile bool g_trace_on = false;
static uint32_t g_trace_t0_ms = 0u;

/* Speed state */
static int32_t g_last_left_count = 0;
static int32_t g_last_right_count = 0;
static int32_t g_left_speed_tenths_cm_s = 0;   /* scaled x10 */
static int32_t g_right_speed_tenths_cm_s = 0;  /* scaled x10 */
static uint32_t g_last_speed_calc_ms = 0u;

/* Current commanded state (% duty or mm/s, see g_closed_loop) */
static int16_t g_current_left_speed_cmd = 0;
static int16_t g_current_right_speed_cmd = 0;

static const int8_t g_quad_decode_table[16] = {
    0,  -1,   1,   0,
    1,   0,   0,  -1,
   -1,   0,   0,   1,
    0,   1,  -1,   0
};

/*----------------------------------------------------------------------------
 * Helper Functions
 *---------------------------------------------------------------------------*/
static int8_t
quadrature_decode(uint8_t prev_state, uint8_t curr_state)
{
    uint8_t index;

    index = (uint8_t)((prev_state << 2) | curr_state);
    return g_quad_decode_table[index];
}

static inline uint8_t
gpio_read(uint8_t pin)
{
    return (in_w(GPIO_IN) & (1u << pin)) ? 1u : 0u;
}

static int32_t
abs_i32(int32_t value)
{
    return (value < 0) ? -value : value;
}

static void
format_signed_tenths(int32_t value_x10, int32_t *whole, int32_t *frac)
{
    int32_t abs_value;

    if ((NULL == whole) || (NULL == frac))
    {
        return;
    }

    abs_value = abs_i32(value_x10);

    *whole = abs_value / 10;
    *frac = abs_value % 10;
}

static int8_t
sign_i16(int16_t value)
{
    return (value > 0) ? 1 : ((value < 0) ? -1 : 0);
}

static int32_t
min_i32(int32_t a, int32_t b)
{
    return (a < b) ? a : b;
}

static int32_t
minimum_progress_counts(void)
{
    int32_t left_progress;
    int32_t right_progress;

    left_progress = abs_i32(g_hw_left_count);
    right_progress = abs_i32(g_hw_right_count);

    return min_i32(left_progress, right_progress);
}

/*----------------------------------------------------------------------------
 * Motor Control (PWM)
 *
 * Each motor uses one PWM slice: channel A on the FWD pin, channel B on the
 * REV pin. Forward drives A with the duty and holds B low; reverse is the
 * opposite. Duty is in tenths of a percent (0..1000). The applied duty is
 * ramped toward the command every loop to limit inrush (brownouts).
 *---------------------------------------------------------------------------*/
#if ((MOTOR_LEFT_FWD & 1) != 0) || (MOTOR_LEFT_REV != (MOTOR_LEFT_FWD + 1)) || \
    ((MOTOR_RIGHT_FWD & 1) != 0) || (MOTOR_RIGHT_REV != (MOTOR_RIGHT_FWD + 1))
#error "Motor FWD/REV pins must be the A/B pair of one PWM slice"
#endif

static void
pwm_block_release(void)
{
    /* USE_PTMR is 0, so the kernel leaves the PWM block in reset */
    if (0u != (in_w(RESETS_RESET) & RESETS_RESET_PWM))
    {
        clr_w(RESETS_RESET, RESETS_RESET_PWM);
    }

    while (0u == (in_w(RESETS_RESET_DONE) & RESETS_RESET_PWM))
    {
        ;
    }
}

static void
motor_pwm_slice_init(uint8_t fwd_pin)
{
    uint32_t slice;

    slice = PWM_SLICE_OF(fwd_pin);

    out_w(PWM_SLICE_REG(slice, PWM_CHx_CSR), 0u);
    out_w(PWM_SLICE_REG(slice, PWM_CHx_DIV), PWM_DIV_INT_1);
    out_w(PWM_SLICE_REG(slice, PWM_CHx_TOP), MOTOR_PWM_TOP);
    out_w(PWM_SLICE_REG(slice, PWM_CHx_CC), 0u);
    out_w(PWM_SLICE_REG(slice, PWM_CHx_CTR), 0u);
    out_w(PWM_SLICE_REG(slice, PWM_CHx_CSR), PWM_CSR_EN);

    out_w(GPIO_CTRL(fwd_pin), GPIO_CTRL_FUNCSEL_PWM);
    out_w(GPIO_CTRL(fwd_pin + 1u), GPIO_CTRL_FUNCSEL_PWM);
}

static void
motor_pwm_apply(uint8_t fwd_pin, int16_t duty_x10)
{
    uint32_t level;
    uint32_t cc;

    if (duty_x10 > MOTOR_DUTY_MAX_X10)
    {
        duty_x10 = MOTOR_DUTY_MAX_X10;
    }
    else if (duty_x10 < -MOTOR_DUTY_MAX_X10)
    {
        duty_x10 = -MOTOR_DUTY_MAX_X10;
    }

    if (duty_x10 >= 0)
    {
        level = ((uint32_t)duty_x10 * (MOTOR_PWM_TOP + 1u)) / 1000u;
        cc = level;                     /* A = FWD */
    }
    else
    {
        level = ((uint32_t)(-duty_x10) * (MOTOR_PWM_TOP + 1u)) / 1000u;
        cc = level << 16;               /* B = REV */
    }

    out_w(PWM_SLICE_REG(PWM_SLICE_OF(fwd_pin), PWM_CHx_CC), cc);
}

static void
motor_gpio_init(void)
{
    pwm_block_release();

    motor_pwm_slice_init(MOTOR_LEFT_FWD);
    motor_pwm_slice_init(MOTOR_RIGHT_FWD);

    tm_printf((UB *)"[MOTION] Motor PWM initialized (%u Hz, TOP=%u)\n",
              MOTOR_PWM_FREQ_HZ, MOTOR_PWM_TOP);
}

/* Move one applied duty a step toward its target. */
static int16_t
motor_ramp(int16_t applied_x10, int16_t target_x10)
{
    if (applied_x10 < target_x10)
    {
        applied_x10 += MOTOR_RAMP_STEP_X10;
        if (applied_x10 > target_x10)
        {
            applied_x10 = target_x10;
        }
    }
    else if (applied_x10 > target_x10)
    {
        applied_x10 -= MOTOR_RAMP_STEP_X10;
        if (applied_x10 < target_x10)
        {
            applied_x10 = target_x10;
        }
    }

    return applied_x10;
}

/* Open loop: ramp toward the commanded duty. Closed loop: apply the PI
 * output directly, since the PI setpoint is already slew-limited and a
 * ramp inside the loop would only add lag. */
static void
motor_ramp_update(void)
{
    if (g_closed_loop)
    {
        g_applied_left_x10 = g_target_left_x10;
        g_applied_right_x10 = g_target_right_x10;
    }
    else
    {
        g_applied_left_x10 = motor_ramp(g_applied_left_x10, g_target_left_x10);
        g_applied_right_x10 = motor_ramp(g_applied_right_x10, g_target_right_x10);
    }

    motor_pwm_apply(MOTOR_LEFT_FWD, g_applied_left_x10);
    motor_pwm_apply(MOTOR_RIGHT_FWD, g_applied_right_x10);
}

/*----------------------------------------------------------------------------
 * Closed-loop PI
 *---------------------------------------------------------------------------*/
static int32_t
clamp_i32(int32_t value, int32_t lo, int32_t hi)
{
    return (value < lo) ? lo : ((value > hi) ? hi : value);
}

static int32_t
mm_s_to_eps(int32_t mm_s)
{
    return (mm_s * 1000L) / UM_PER_EDGE;
}

/* Restart a wheel's loop from its current speed so switching mode or
 * target does not kick the motor. */
static void
pi_reset(wheel_pi_t *p_pi, int32_t count_now)
{
    p_pi->setpoint_eps = p_pi->meas_eps;
    p_pi->integ_x10 = 0;
    p_pi->last_count = count_now;
}

static void
pi_step(wheel_pi_t *p_pi, int32_t count_now, uint32_t dt_ms,
        int32_t ff_eps_per_pct_x10)
{
    int32_t err;
    int32_t out;

    p_pi->meas_eps = ((count_now - p_pi->last_count) * 1000L) / (int32_t)dt_ms;
    p_pi->last_count = count_now;

    p_pi->setpoint_eps += clamp_i32(p_pi->target_eps - p_pi->setpoint_eps,
                                    -PID_SLEW_EPS, PID_SLEW_EPS);

    if ((0 == p_pi->setpoint_eps) && (0 == p_pi->target_eps))
    {
        /* Stopped: coast, and do not let the integral wind up */
        p_pi->integ_x10 = 0;
        p_pi->duty_x10 = 0;
        return;
    }

    err = p_pi->setpoint_eps - p_pi->meas_eps;

    /* Integrate only once the setpoint has stopped slewing. While it ramps,
     * the wheel always lags it, and integrating that lag wound the integral
     * up, giving ~7 % overshoot that took ~0.5 s to bleed off (lifted step
     * test, 2026-10-07). Feed-forward carries the ramp instead. */
    if (p_pi->setpoint_eps == p_pi->target_eps)
    {
        p_pi->integ_x10 += (err * PID_KI_X1000) / 1000L;
    }
    p_pi->integ_x10 = clamp_i32(p_pi->integ_x10, -PID_I_LIMIT_X10, PID_I_LIMIT_X10);

    out = (p_pi->setpoint_eps * 100L) / ff_eps_per_pct_x10
        + (err * PID_KP_X1000) / 1000L
        + p_pi->integ_x10;

    /* Never drive against the setpoint direction: the edge counter takes
     * its sign from the applied duty, so a sign flip while the wheel still
     * turns would corrupt the measurement. Let it coast down instead. */
    if (p_pi->setpoint_eps > 0)
    {
        out = clamp_i32(out, 0, MOTOR_DUTY_MAX_X10);
    }
    else
    {
        out = clamp_i32(out, -MOTOR_DUTY_MAX_X10, 0);
    }

    p_pi->duty_x10 = (int16_t)out;
}

static void
trace_record(uint32_t now_ms)
{
    motion_trace_t *p_t;

    if ((!g_trace_on) || (g_trace_n >= TRACE_LEN))
    {
        g_trace_on = false;
        return;
    }

    p_t = &g_trace[g_trace_n];
    p_t->t_ms = (uint16_t)(now_ms - g_trace_t0_ms);
    p_t->sp_l = (int16_t)g_pi_left.setpoint_eps;
    p_t->meas_l = (int16_t)g_pi_left.meas_eps;
    p_t->duty_l = g_pi_left.duty_x10;
    p_t->sp_r = (int16_t)g_pi_right.setpoint_eps;
    p_t->meas_r = (int16_t)g_pi_right.meas_eps;
    p_t->duty_r = g_pi_right.duty_x10;
    g_trace_n++;
}

static void
pi_update(uint32_t now_ms)
{
    uint32_t dt_ms;

    dt_ms = now_ms - g_last_pi_ms;
    if (dt_ms < PID_PERIOD_MS)
    {
        return;
    }
    g_last_pi_ms = now_ms;

    pi_step(&g_pi_left, g_hw_left_count, dt_ms, FF_EPS_PER_PCT_X10_L);
    pi_step(&g_pi_right, g_hw_right_count, dt_ms, FF_EPS_PER_PCT_X10_R);

    g_target_left_x10 = g_pi_left.duty_x10;
    g_target_right_x10 = g_pi_right.duty_x10;

    trace_record(now_ms);
}


/*----------------------------------------------------------------------------
 * Encoder Handling
 *---------------------------------------------------------------------------*/
static void
encoder_init(void)
{
    uint8_t l_a;
    uint8_t l_b;
    uint8_t r_a;
    uint8_t r_b;

    out_w(GPIO_CTRL(ENC_LEFT_A), GPIO_FUNCTION_SIO);
    out_w(GPIO_CTRL(ENC_LEFT_B), GPIO_FUNCTION_SIO);
    out_w(GPIO_CTRL(ENC_RIGHT_A), GPIO_FUNCTION_SIO);
    out_w(GPIO_CTRL(ENC_RIGHT_B), GPIO_FUNCTION_SIO);

    l_a = gpio_read(ENC_LEFT_A);
    l_b = gpio_read(ENC_LEFT_B);
    r_a = gpio_read(ENC_RIGHT_A);
    r_b = gpio_read(ENC_RIGHT_B);

    g_enc_left_last_state = (uint8_t)((l_a << 1) | l_b);
    g_enc_right_last_state = (uint8_t)((r_a << 1) | r_b);

    tm_printf((UB *)"[MOTION] Encoders initialized (%u edges/rev, %ld um/edge)\n",
              EDGES_PER_REV,
              UM_PER_EDGE);
}

/* State is (A << 1) | B. Count each channel's edges separately, and flag
 * polls where both channels changed: quadrature never does that in one
 * step, so it means a state was skipped or a line glitched. */
static void
encoder_diag_update(uint8_t prev_state, uint8_t curr_state,
                    volatile uint32_t *p_a_edges,
                    volatile uint32_t *p_b_edges,
                    volatile uint32_t *p_invalid)
{
    uint8_t changed;

    changed = (uint8_t)(prev_state ^ curr_state);

    if (0u != (changed & 0x2u))
    {
        (*p_a_edges)++;
    }

    if (0u != (changed & 0x1u))
    {
        (*p_b_edges)++;
    }

    if (0x3u == changed)
    {
        (*p_invalid)++;
    }
}

static void
encoder_update(void)
{
    uint8_t l_a;
    uint8_t l_b;
    uint8_t r_a;
    uint8_t r_b;
    uint8_t left_state;
    uint8_t right_state;
    int8_t left_delta;
    int8_t right_delta;

    l_a = gpio_read(ENC_LEFT_A);
    l_b = gpio_read(ENC_LEFT_B);
    r_a = gpio_read(ENC_RIGHT_A);
    r_b = gpio_read(ENC_RIGHT_B);

    left_state = (uint8_t)((l_a << 1) | l_b);
    right_state = (uint8_t)((r_a << 1) | r_b);

    left_delta = quadrature_decode(g_enc_left_last_state, left_state);
    right_delta = quadrature_decode(g_enc_right_last_state, right_state);

    encoder_diag_update(g_enc_left_last_state, left_state,
                        &g_enc_left_a_edges, &g_enc_left_b_edges,
                        &g_enc_left_invalid);
    encoder_diag_update(g_enc_right_last_state, right_state,
                        &g_enc_right_a_edges, &g_enc_right_b_edges,
                        &g_enc_right_invalid);

    g_enc_left_count += (int32_t)left_delta;
    g_enc_right_count += (int32_t)right_delta;

    g_enc_left_last_state = left_state;
    g_enc_right_last_state = right_state;
    g_enc_polls++;
}

static void
encoder_hw_slice_init(uint8_t pin)
{
    uint32_t slice;

    slice = PWM_SLICE_OF(pin);

    out_w(PWM_SLICE_REG(slice, PWM_CHx_CSR), 0u);
    out_w(PWM_SLICE_REG(slice, PWM_CHx_DIV), PWM_DIV_INT_1);
    out_w(PWM_SLICE_REG(slice, PWM_CHx_TOP), 0xFFFFu);
    out_w(PWM_SLICE_REG(slice, PWM_CHx_CTR), 0u);
    out_w(PWM_SLICE_REG(slice, PWM_CHx_CSR),
          PWM_CSR_DIVMODE_B_RISE | PWM_CSR_EN);

    /* Route the pin to PWM. SIO GPIO_IN still reads the pad, so the
     * polled decoder keeps working on this pin for comparison. */
    out_w(GPIO_CTRL(pin), GPIO_CTRL_FUNCSEL_PWM);
}

static void
encoder_hw_init(void)
{
    pwm_block_release();

    encoder_hw_slice_init(ENC_HW_LEFT_PIN);
    encoder_hw_slice_init(ENC_HW_RIGHT_PIN);

    g_hw_left_last_ctr = (uint16_t)in_w(
        PWM_SLICE_REG(PWM_SLICE_OF(ENC_HW_LEFT_PIN), PWM_CHx_CTR));
    g_hw_right_last_ctr = (uint16_t)in_w(
        PWM_SLICE_REG(PWM_SLICE_OF(ENC_HW_RIGHT_PIN), PWM_CHx_CTR));

    tm_printf((UB *)"[MOTION] HW edge counters: L=GP%u (PWM%u) R=GP%u (PWM%u)\n",
              ENC_HW_LEFT_PIN, PWM_SLICE_OF(ENC_HW_LEFT_PIN),
              ENC_HW_RIGHT_PIN, PWM_SLICE_OF(ENC_HW_RIGHT_PIN));
}

/* Read one hardware counter and fold the edges since the last read into
 * the totals. 16-bit wrap is handled by the unsigned subtraction; it only
 * needs reading more often than every 65535 edges. */
static void
encoder_hw_read(uint8_t pin, uint16_t *p_last_ctr, int8_t cmd, int8_t *p_dir,
                volatile uint32_t *p_edges, volatile int32_t *p_count)
{
    uint16_t ctr;
    uint16_t delta;

    ctr = (uint16_t)in_w(PWM_SLICE_REG(PWM_SLICE_OF(pin), PWM_CHx_CTR));
    delta = (uint16_t)(ctr - *p_last_ctr);
    *p_last_ctr = ctr;

    /* Keep the last driven direction so coasting after a stop is signed */
    if (cmd > 0)
    {
        *p_dir = 1;
    }
    else if (cmd < 0)
    {
        *p_dir = -1;
    }

    *p_edges += (uint32_t)delta;
    *p_count += (int32_t)(*p_dir) * (int32_t)delta;
}

static void
encoder_hw_update(void)
{
    /* Sign edges by the duty actually applied, not the target, so a wheel
     * still ramping down after a reversal command keeps its old sign */
    encoder_hw_read(ENC_HW_LEFT_PIN, &g_hw_left_last_ctr,
                    sign_i16(g_applied_left_x10), &g_hw_left_dir,
                    &g_hw_left_edges, &g_hw_left_count);
    encoder_hw_read(ENC_HW_RIGHT_PIN, &g_hw_right_last_ctr,
                    sign_i16(g_applied_right_x10), &g_hw_right_dir,
                    &g_hw_right_edges, &g_hw_right_count);
}

/*----------------------------------------------------------------------------
 * Speed / Odometry
 *---------------------------------------------------------------------------*/
static void
speed_calculate(uint32_t dt_ms)
{
    int32_t delta_left;
    int32_t delta_right;

    if (0u == dt_ms)
    {
        return;
    }

    delta_left = g_hw_left_count - g_last_left_count;
    delta_right = g_hw_right_count - g_last_right_count;

    /* edges * um/edge / ms = mm/s, which is also tenths of cm/s */
    g_left_speed_tenths_cm_s = (delta_left * UM_PER_EDGE) / (int32_t)dt_ms;
    g_right_speed_tenths_cm_s = (delta_right * UM_PER_EDGE) / (int32_t)dt_ms;

    g_last_left_count = g_hw_left_count;
    g_last_right_count = g_hw_right_count;
}

static int32_t
distance_cm_from_counts(void)
{
    int32_t avg_counts;
    int32_t distance_um;
    int32_t distance_cm;

    avg_counts = (g_hw_left_count + g_hw_right_count) / 2;
    distance_um = avg_counts * UM_PER_EDGE;
    distance_cm = distance_um / 10000L;

    return distance_cm;
}

/*----------------------------------------------------------------------------
 * Motion Task
 *---------------------------------------------------------------------------*/
static void
motion_task_main(INT stacd, void *exinf)
{
    motor_cmd_t cmd;
    INT msg_size;
    uint32_t loop_count = 0u;
    uint32_t print_divider;
    SYSTIM now;

    (void)stacd;
    (void)exinf;

    motor_gpio_init();
    encoder_init();
    encoder_hw_init();

    tm_printf((UB *)"[MOTION] Task started (period=%u ms)\n", TASK_PERIOD_MS);
    tm_printf((UB *)"[MOTION] Wheel: %u mm diameter, %u mm circumference\n",
              WHEEL_DIAMETER_MM,
              WHEEL_CIRCUMFERENCE_MM);

    tk_get_tim(&now);
    g_last_speed_calc_ms = now.lo;

    print_divider = ODOM_PRINT_INTERVAL_MS / SPEED_CALC_INTERVAL_MS;
    if (0u == print_divider)
    {
        print_divider = 1u;
    }

    while (1)
    {
        /* Read hardware counters before a new command can change the
         * direction used to sign them */
        encoder_hw_update();

        msg_size = tk_rcv_mbf(motion_msgbuf_id, &cmd, TMO_POL);
        if ((INT)sizeof(cmd) == msg_size)
        {
            g_current_left_speed_cmd = cmd.left_speed;
            g_current_right_speed_cmd = cmd.right_speed;

            if (0u != cmd.closed_loop)
            {
                if (!g_closed_loop)
                {
                    tk_get_tim(&now);
                    g_last_pi_ms = now.lo;
                    pi_reset(&g_pi_left, g_hw_left_count);
                    pi_reset(&g_pi_right, g_hw_right_count);
                    g_closed_loop = true;
                }

                g_pi_left.target_eps = mm_s_to_eps(
                    clamp_i32(cmd.left_speed, -PID_MAX_SPEED_MM_S, PID_MAX_SPEED_MM_S));
                g_pi_right.target_eps = mm_s_to_eps(
                    clamp_i32(cmd.right_speed, -PID_MAX_SPEED_MM_S, PID_MAX_SPEED_MM_S));
            }
            else
            {
                g_closed_loop = false;
                g_target_left_x10 = (int16_t)(clamp_i32(cmd.left_speed, -100, 100) * 10);
                g_target_right_x10 = (int16_t)(clamp_i32(cmd.right_speed, -100, 100) * 10);
            }

            tm_printf((UB *)"[MOTOR] Cmd received: L=%d R=%d %s\n",
                      (int)cmd.left_speed,
                      (int)cmd.right_speed,
                      (0u != cmd.closed_loop) ? "mm/s (PI)" : "% duty");
        }
        else if (E_TMOUT != msg_size)
        {
            tm_printf((UB *)"[MOTOR] tk_rcv_mbf error: %d\n", msg_size);
        }

        if (g_closed_loop)
        {
            tk_get_tim(&now);
            pi_update(now.lo);
        }

        motor_ramp_update();
        encoder_update();

        if ((loop_count % (SPEED_CALC_INTERVAL_MS / TASK_PERIOD_MS)) == 0u)
        {
            int32_t dist_cm_int;
            int32_t left_whole;
            int32_t left_frac;
            int32_t right_whole;
            int32_t right_frac;
            int32_t left_abs;
            int32_t right_abs;

            tk_get_tim(&now);
            speed_calculate(now.lo - g_last_speed_calc_ms);
            g_last_speed_calc_ms = now.lo;

            /* Skip the slow status print while a trace is recording */
            if ((!g_trace_on) &&
                (((loop_count / (SPEED_CALC_INTERVAL_MS / TASK_PERIOD_MS)) % print_divider) == 0u))
            {
                dist_cm_int = distance_cm_from_counts();

                format_signed_tenths(g_left_speed_tenths_cm_s, &left_whole, &left_frac);
                format_signed_tenths(g_right_speed_tenths_cm_s, &right_whole, &right_frac);

                left_abs = abs_i32(g_left_speed_tenths_cm_s);
                right_abs = abs_i32(g_right_speed_tenths_cm_s);

                tm_printf((UB *)"[ODOM] Dist=%ld cm Spd: L=%s%ld.%ld R=%s%ld.%ld cm/s "
                          "Pulse: L=%ld R=%ld Cmd: L=%d R=%d\n",
                          dist_cm_int,
                          (left_abs != g_left_speed_tenths_cm_s) ? "-" : "",
                          left_whole,
                          left_frac,
                          (right_abs != g_right_speed_tenths_cm_s) ? "-" : "",
                          right_whole,
                          right_frac,
                          g_hw_left_count,
                          g_hw_right_count,
                          (int)g_current_left_speed_cmd,
                          (int)g_current_right_speed_cmd);
            }
        }

        loop_count++;
        tk_dly_tsk(TASK_PERIOD_MS);
    }
}

/*----------------------------------------------------------------------------
 * Public API
 *---------------------------------------------------------------------------*/
ER
motion_task_create(void)
{
    T_CTSK ctsk;
    T_CMBF cmbf;
    ER err;

    if ((motion_task_id > 0) || (motion_msgbuf_id > 0))
    {
        return E_OBJ;
    }

    cmbf.mbfatr = TA_TFIFO;
    cmbf.maxmsz = (INT)sizeof(motor_cmd_t);
    cmbf.bufsz = (INT)(sizeof(motor_cmd_t) * MOTION_MSGBUF_DEPTH);

    motion_msgbuf_id = tk_cre_mbf(&cmbf);
    if (motion_msgbuf_id <= 0)
    {
        tm_printf((UB *)"[MOTION] tk_cre_mbf failed: %d\n", motion_msgbuf_id);
        return motion_msgbuf_id;
    }

    motion_task_stack = Kmalloc(MOTION_STACK_SIZE);
    if (NULL == motion_task_stack)
    {
        tk_del_mbf(motion_msgbuf_id);
        motion_msgbuf_id = 0;
        return E_NOMEM;
    }

    ctsk.tskatr = TA_HLNG | TA_RNG3 | TA_USERBUF;
    ctsk.task = motion_task_main;
    ctsk.itskpri = 5;
    ctsk.stksz = MOTION_STACK_SIZE;
    ctsk.bufptr = motion_task_stack;

    motion_task_id = tk_cre_tsk(&ctsk);
    if (motion_task_id <= 0)
    {
        tm_printf((UB *)"[MOTION] tk_cre_tsk failed: %d\n", motion_task_id);
        Kfree(motion_task_stack);
        motion_task_stack = NULL;
        tk_del_mbf(motion_msgbuf_id);
        motion_msgbuf_id = 0;
        return motion_task_id;
    }

    err = tk_sta_tsk(motion_task_id, 0);
    if (E_OK != err)
    {
        tm_printf((UB *)"[MOTION] tk_sta_tsk failed: %d\n", err);
        tk_del_tsk(motion_task_id);
        motion_task_id = 0;
        Kfree(motion_task_stack);
        motion_task_stack = NULL;
        tk_del_mbf(motion_msgbuf_id);
        motion_msgbuf_id = 0;
        return err;
    }

    tm_printf((UB *)"[MOTION] Task created successfully\n");
    return E_OK;
}

/* The motion task prints each command it receives. No print here: callers
 * may run at a higher priority than the motion task, and a blocking UART
 * print would stall the control loop. */
static ER
motion_send_cmd(int16_t left, int16_t right, uint8_t closed_loop)
{
    motor_cmd_t cmd;
    ER err;

    if (motion_msgbuf_id <= 0)
    {
        return E_NOEXS;
    }

    cmd.left_speed = left;
    cmd.right_speed = right;
    cmd.closed_loop = closed_loop;

    err = tk_snd_mbf(motion_msgbuf_id, &cmd, (INT)sizeof(cmd), TMO_FEVR);
    if (E_OK != err)
    {
        tm_printf((UB *)"[MOTOR] tk_snd_mbf failed: %d\n", err);
    }

    return err;
}

ER
motion_set_speed(int8_t left, int8_t right)
{
    return motion_send_cmd((int16_t)left, (int16_t)right, 0u);
}

ER
motion_set_velocity(int16_t left_mm_s, int16_t right_mm_s)
{
    return motion_send_cmd(left_mm_s, right_mm_s, 1u);
}

void
motion_trace_start(void)
{
    SYSTIM now;

    tk_get_tim(&now);
    g_trace_n = 0u;
    g_trace_t0_ms = now.lo;
    g_trace_on = true;
}

void
motion_trace_dump(void)
{
    uint32_t i;
    motion_trace_t *p_t;

    g_trace_on = false;

    tm_printf((UB *)"[TRACE] t_ms,sp_l,meas_l,duty_l,sp_r,meas_r,duty_r"
              "  (speeds in edges/s, duty in tenths of a percent)\n");
    for (i = 0u; i < g_trace_n; i++)
    {
        p_t = &g_trace[i];
        tm_printf((UB *)"[TRACE] %u,%d,%d,%d,%d,%d,%d\n",
                  (unsigned int)p_t->t_ms,
                  (int)p_t->sp_l, (int)p_t->meas_l, (int)p_t->duty_l,
                  (int)p_t->sp_r, (int)p_t->meas_r, (int)p_t->duty_r);
    }
}

ER
motion_get_odometry(odometry_t *p_odom)
{
    SYSTIM now;
    int32_t avg_counts;
    int32_t distance_um;

    if (NULL == p_odom)
    {
        return E_PAR;
    }

    p_odom->left_pulses = g_hw_left_count;
    p_odom->right_pulses = g_hw_right_count;

    p_odom->left_speed_cm_s = ((float)g_left_speed_tenths_cm_s) / 10.0f;
    p_odom->right_speed_cm_s = ((float)g_right_speed_tenths_cm_s) / 10.0f;

    avg_counts = (g_hw_left_count + g_hw_right_count) / 2;
    distance_um = avg_counts * UM_PER_EDGE;
    p_odom->distance_cm = ((float)distance_um) / 10000.0f;

    tk_get_tim(&now);
    p_odom->timestamp_ms = now.lo;

    return E_OK;
}

void
motion_reset_odometry(void)
{
    g_enc_left_count = 0;
    g_enc_right_count = 0;
    g_last_left_count = 0;
    g_last_right_count = 0;
    g_left_speed_tenths_cm_s = 0;
    g_right_speed_tenths_cm_s = 0;

    g_enc_left_a_edges = 0u;
    g_enc_left_b_edges = 0u;
    g_enc_right_a_edges = 0u;
    g_enc_right_b_edges = 0u;
    g_enc_left_invalid = 0u;
    g_enc_right_invalid = 0u;
    g_enc_polls = 0u;

    g_hw_left_edges = 0u;
    g_hw_right_edges = 0u;
    g_hw_left_count = 0;
    g_hw_right_count = 0;

    /* Keep the PI's last-count in step, or its next speed sample would
     * see the reset as a huge jump */
    g_pi_left.last_count = 0;
    g_pi_right.last_count = 0;

    tm_printf((UB *)"[ODOM] Reset\n");
}

ER
motion_get_encoder_diag(enc_diag_t *p_left, enc_diag_t *p_right)
{
    if ((NULL == p_left) || (NULL == p_right))
    {
        return E_PAR;
    }

    p_left->count = g_enc_left_count;
    p_left->a_edges = g_enc_left_a_edges;
    p_left->b_edges = g_enc_left_b_edges;
    p_left->invalid = g_enc_left_invalid;

    p_right->count = g_enc_right_count;
    p_right->a_edges = g_enc_right_a_edges;
    p_right->b_edges = g_enc_right_b_edges;
    p_right->invalid = g_enc_right_invalid;
    p_left->polls = g_enc_polls;
    p_right->polls = g_enc_polls;

    p_left->hw_edges = g_hw_left_edges;
    p_left->hw_count = g_hw_left_count;
    p_right->hw_edges = g_hw_right_edges;
    p_right->hw_count = g_hw_right_count;

    return E_OK;
}

void
motion_print_encoder_diag(const char *p_label)
{
    enc_diag_t left;
    enc_diag_t right;

    (void)motion_get_encoder_diag(&left, &right);

    tm_printf((UB *)"[ENC] %s L: cnt=%ld A=%lu B=%lu inv=%lu | "
              "R: cnt=%ld A=%lu B=%lu inv=%lu | polls=%lu\n",
              (NULL != p_label) ? p_label : "",
              left.count, left.a_edges, left.b_edges, left.invalid,
              right.count, right.a_edges, right.b_edges, right.invalid,
              left.polls);
    tm_printf((UB *)"[HW]  %s L: edges=%lu cnt=%ld | R: edges=%lu cnt=%ld\n",
              (NULL != p_label) ? p_label : "",
              left.hw_edges, left.hw_count,
              right.hw_edges, right.hw_count);
}

ER
motion_move_forward_cm(uint16_t distance_cm, uint8_t speed)
{
    ER err;
    int32_t target_counts;
    int32_t distance_mm;
    int32_t min_progress;

    if (0u == distance_cm)
    {
        tm_printf((UB *)"[MOTION] Invalid distance: 0 cm\n");
        return E_PAR;
    }

    if (speed > 100u)
    {
        speed = 100u;
    }

    distance_mm = (int32_t)distance_cm * 10;
    target_counts = (distance_mm * 1000L) / UM_PER_EDGE;

    if (target_counts <= 0)
    {
        tm_printf((UB *)"[MOTION] Invalid target count calculation\n");
        return E_SYS;
    }

    tm_printf((UB *)"[MOTION] move_forward_cm: target=%u cm, speed=%u\n",
              distance_cm,
              speed);
    tm_printf((UB *)"[MOTION] Target edges=%ld um_per_edge=%ld\n",
              target_counts,
              UM_PER_EDGE);

    motion_reset_odometry();

    err = motion_set_speed((int8_t)speed, (int8_t)speed);
    if (E_OK != err)
    {
        tm_printf((UB *)"[MOTION] motion_set_speed failed: %d\n", err);
        return err;
    }

    while (1)
    {
        min_progress = minimum_progress_counts();

        if (min_progress >= target_counts)
        {
            break;
        }

        tk_dly_tsk(20);
    }

    err = motion_set_speed(0, 0);
    if (E_OK != err)
    {
        tm_printf((UB *)"[MOTION] stop command failed: %d\n", err);
        return err;
    }

    tm_printf((UB *)"[MOTION] Target reached. Final: L=%ld R=%ld Min=%ld\n",
              g_hw_left_count,
              g_hw_right_count,
              min_progress);
    motion_print_encoder_diag("move_forward_cm");

    return E_OK;
}