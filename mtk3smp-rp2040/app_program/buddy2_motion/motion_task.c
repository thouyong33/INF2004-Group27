/**
 * @file motion_task.c
 * @brief Motion control with speed calculation and distance tracking
 *
 * - Motor drive: 20 kHz PWM, either open-loop % duty (ramped to limit
 *   inrush) or per-wheel PI speed control in mm/s.
 * - Odometry: PWM-slice hardware edge counters (one channel per wheel),
 *   signed by the applied motor direction. The old polled quadrature
 *   decoder is gone: it missed edges at speed and the counters replaced it.
 * - Safety (Tribunal 1): STOP is a flag the loop checks every pass, so it
 *   never waits behind queued commands. A stall or a silent encoder raises
 *   a bit in the shared fault mask and stops the motors until cleared.
 * - Nothing in the loop prints: a UART print blocks for milliseconds and
 *   would stretch the 1 ms control period. Counts shared with other tasks
 *   are written and read inside a short critical section.
 */

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include <sys/sysdef.h>

#include "../common/pin_defs.h"
#include "../common/critical.h"
#include "../common/faults.h"
#include "motion_task.h"

/*----------------------------------------------------------------------------
 * Calibrated Encoder / Wheel Constants
 *---------------------------------------------------------------------------*/
/* Hardware edge counter: rising edges of one encoder channel per wheel turn.
 * Measured 2026-10-07, 10 hand turns x 2 per wheel: L 6301/6363, R 6349/6333. */
#define EDGES_PER_REV             634u
#define WHEEL_DIAMETER_MM         65u
#define WHEEL_CIRCUMFERENCE_MM    204u

/* Nominal 204 mm / 634 edges = 0.322 mm per edge, but floor tape tests
 * (2026-10-07: 509/501, 305/298, 516/507 mm tape/encoder) show the car
 * travels 1.9 % further, i.e. an effective rolling circumference of
 * ~208 mm on the floor. */
#define UM_PER_EDGE               328L

/*----------------------------------------------------------------------------
 * Configuration Constants
 *---------------------------------------------------------------------------*/
#define TASK_PERIOD_MS            1u
#define SPEED_CALC_INTERVAL_MS    100u
#define LOOP_RATE_WINDOW_MS       1000u
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

/* Guards the counts and speeds that other tasks read */
static crit_lock_t g_odom_lock;

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
#define PID_KI_X1000              120L  /* duty_x10 per (edge/s) per period */
#define PID_I_LIMIT_X10           300L  /* integral clamp: +/- 30 % duty */
#define PID_SLEW_EPS              50L   /* setpoint change per period */

/* Fault checks, run every PI period while both wheels hold a settled,
 * non-zero setpoint. Provisional thresholds (Tribunal 1, R4), to be tuned
 * from logs. Masked for PULLAWAY_MASK_MS after the setpoint settles and
 * while Buddy 4 reports the car is on a hump. */
#define PULLAWAY_MASK_MS          300u
#define STALL_PCT                 30L   /* both wheels under 30 % of setpoint */
#define STALL_MS                  200u
#define ENC_FAULT_DUTY_X10        400   /* wheel driven at >= 40 % duty... */
#define ENC_FAULT_OTHER_PCT       50L   /* ...other wheel >= 50 % of setpoint */
#define ENC_FAULT_MS              200u  /* ...and no edges for this long */

/* motion_move_forward_cm() */
#define MOVE_MIN_SPEED_MM_S       50
#define MOVE_POLL_MS              10u
#define MOVE_STILL_MS             100u  /* no edges for this long = stopped */
#define MOVE_SETTLE_MAX_MS        2000u

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

/* Safety */
static volatile uint8_t g_stop_req = 0u;    /* byte store: no lock needed */
static volatile uint8_t g_hump_mask = 0u;
static uint32_t g_settled_ms = 0u;          /* settled for this long */
static uint32_t g_stall_ms = 0u;
static uint32_t g_enc_l_silent_ms = 0u;
static uint32_t g_enc_r_silent_ms = 0u;

/* Loop rate: passes of the 1 ms loop counted over LOOP_RATE_WINDOW_MS */
static volatile uint32_t g_loop_rate = 0u;

/* CPU evidence: time the loop spends working (not sleeping), from the
 * 1 MHz hardware timer, per second and the longest single pass. Excludes
 * kernel tick/dispatch overhead. The timer only runs once something has
 * released it (ultrasonic_init); before that these read 0. */
static uint32_t g_busy_acc_us = 0u;
static uint32_t g_busy_pass_max_us = 0u;
static volatile uint32_t g_busy_us_per_s = 0u;
static volatile uint32_t g_busy_max_us = 0u;

#define STACK_FILL_WORD           0xA5A5A5A5u

static void motion_checks(uint32_t dt_ms);

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


/*----------------------------------------------------------------------------
 * Helper Functions
 *---------------------------------------------------------------------------*/
static int32_t
abs_i32(int32_t value)
{
    return (value < 0) ? -value : value;
}

static int8_t
sign_i16(int16_t value)
{
    return (value > 0) ? 1 : ((value < 0) ? -1 : 0);
}

/* Distance progress as the average of both wheels. (The old minimum-of-
 * both workaround is no longer needed: the hardware counts are trusted.) */
static int32_t
average_progress_counts(void)
{
    UINT sts;
    int32_t l;
    int32_t r;

    CRIT_ENTER(&g_odom_lock, sts);
    l = g_hw_left_count;
    r = g_hw_right_count;
    CRIT_EXIT(&g_odom_lock, sts);

    return (abs_i32(l) + abs_i32(r)) / 2;
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
    motion_checks(dt_ms);
}


/*----------------------------------------------------------------------------
 * Encoder Handling
 *---------------------------------------------------------------------------*/
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
    UINT sts;

    /* Sign edges by the duty actually applied, not the target, so a wheel
     * still ramping down after a reversal command keeps its old sign.
     * Locked so a reader never sees the left and right counts from
     * different passes. */
    CRIT_ENTER(&g_odom_lock, sts);
    encoder_hw_read(ENC_HW_LEFT_PIN, &g_hw_left_last_ctr,
                    sign_i16(g_applied_left_x10), &g_hw_left_dir,
                    &g_hw_left_edges, &g_hw_left_count);
    encoder_hw_read(ENC_HW_RIGHT_PIN, &g_hw_right_last_ctr,
                    sign_i16(g_applied_right_x10), &g_hw_right_dir,
                    &g_hw_right_edges, &g_hw_right_count);
    CRIT_EXIT(&g_odom_lock, sts);
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

/*----------------------------------------------------------------------------
 * Safety: stop request and fault checks
 *---------------------------------------------------------------------------*/
/* Stop both wheels. Under PI the setpoints ramp down (controlled braking);
 * in open loop the duty ramps to zero. Commands queued before the stop are
 * discarded so they cannot restart the car. */
static void
motion_apply_stop(void)
{
    motor_cmd_t junk;

    while (tk_rcv_mbf(motion_msgbuf_id, &junk, TMO_POL) > 0)
    {
        ;
    }

    if (g_closed_loop)
    {
        g_pi_left.target_eps = 0;
        g_pi_right.target_eps = 0;
    }
    else
    {
        g_target_left_x10 = 0;
        g_target_right_x10 = 0;
    }
}

/* A fault drops to open loop with zero duty: the PI cannot be trusted when
 * a wheel's speed reading is wrong. The duty still ramps down. */
static void
motion_fault_stop(uint8_t bits)
{
    fault_set(bits);
    g_closed_loop = false;
    g_target_left_x10 = 0;
    g_target_right_x10 = 0;
}

static bool
below_pct(int32_t meas, int32_t setpoint, int32_t pct)
{
    return (abs_i32(meas) * 100L) < (abs_i32(setpoint) * pct);
}

static void
motion_checks(uint32_t dt_ms)
{
    bool settled;

    settled = g_closed_loop
        && (0 != g_pi_left.target_eps) && (0 != g_pi_right.target_eps)
        && (g_pi_left.setpoint_eps == g_pi_left.target_eps)
        && (g_pi_right.setpoint_eps == g_pi_right.target_eps);

    if (!settled)
    {
        g_settled_ms = 0u;
    }
    else if (g_settled_ms < PULLAWAY_MASK_MS)
    {
        g_settled_ms += dt_ms;
    }

    if ((!settled) || (g_settled_ms < PULLAWAY_MASK_MS) || (0u != g_hump_mask))
    {
        g_stall_ms = 0u;
        g_enc_l_silent_ms = 0u;
        g_enc_r_silent_ms = 0u;
        return;
    }

    /* Stall: both wheels far below their setpoints (blocked car) */
    if (below_pct(g_pi_left.meas_eps, g_pi_left.setpoint_eps, STALL_PCT)
        && below_pct(g_pi_right.meas_eps, g_pi_right.setpoint_eps, STALL_PCT))
    {
        g_stall_ms += dt_ms;
    }
    else
    {
        g_stall_ms = 0u;
    }

    /* Encoder plausibility: one wheel shows no edges at all while it is
     * driven hard and the other wheel turns normally (unplugged or broken
     * encoder, or a jammed wheel). The PI would otherwise run it flat out. */
    if ((0 == g_pi_left.meas_eps)
        && (abs_i32(g_pi_left.duty_x10) >= ENC_FAULT_DUTY_X10)
        && (!below_pct(g_pi_right.meas_eps, g_pi_right.setpoint_eps, ENC_FAULT_OTHER_PCT)))
    {
        g_enc_l_silent_ms += dt_ms;
    }
    else
    {
        g_enc_l_silent_ms = 0u;
    }

    if ((0 == g_pi_right.meas_eps)
        && (abs_i32(g_pi_right.duty_x10) >= ENC_FAULT_DUTY_X10)
        && (!below_pct(g_pi_left.meas_eps, g_pi_left.setpoint_eps, ENC_FAULT_OTHER_PCT)))
    {
        g_enc_r_silent_ms += dt_ms;
    }
    else
    {
        g_enc_r_silent_ms = 0u;
    }

    if (g_stall_ms >= STALL_MS)
    {
        motion_fault_stop(FAULT_MOTOR_STALL);
    }
    else if (g_enc_l_silent_ms >= ENC_FAULT_MS)
    {
        motion_fault_stop(FAULT_ENC_LEFT);
    }
    else if (g_enc_r_silent_ms >= ENC_FAULT_MS)
    {
        motion_fault_stop(FAULT_ENC_RIGHT);
    }
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
    uint32_t rate_loops = 0u;
    uint32_t rate_t0_ms;
    uint32_t t_pass_us;
    uint32_t busy_us;
    SYSTIM now;
    UINT sts;

    (void)stacd;
    (void)exinf;

    motor_gpio_init();
    encoder_hw_init();

    /* Before the loop starts, so this one print cannot stretch a period */
    tm_printf((UB *)"[MOTION] Task started: %u ms loop, 20 kHz PWM, HW edge counters "
              "L=GP%u R=GP%u, %ld um/edge\n", TASK_PERIOD_MS,
              ENC_HW_LEFT_PIN, ENC_HW_RIGHT_PIN, UM_PER_EDGE);

    tk_get_tim(&now);
    g_last_speed_calc_ms = now.lo;
    rate_t0_ms = now.lo;

    while (1)
    {
        t_pass_us = in_w(TIMER_TIMERAWL);

        /* Read hardware counters before a new command can change the
         * direction used to sign them */
        encoder_hw_update();

        /* STOP first: it must never wait behind queued commands */
        if (0u != g_stop_req)
        {
            g_stop_req = 0u;
            motion_apply_stop();
        }

        msg_size = tk_rcv_mbf(motion_msgbuf_id, &cmd, TMO_POL);
        if (((INT)sizeof(cmd) == msg_size)
            && ((0u == (fault_get() & FAULT_MOTION_MASK))
                || ((0 == cmd.left_speed) && (0 == cmd.right_speed))))
        {
            /* While a motion fault is latched only zero commands pass */
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
        }

        if (g_closed_loop)
        {
            tk_get_tim(&now);
            pi_update(now.lo);
        }

        motor_ramp_update();

        if ((loop_count % (SPEED_CALC_INTERVAL_MS / TASK_PERIOD_MS)) == 0u)
        {
            tk_get_tim(&now);
            CRIT_ENTER(&g_odom_lock, sts);
            speed_calculate(now.lo - g_last_speed_calc_ms);
            CRIT_EXIT(&g_odom_lock, sts);
            g_last_speed_calc_ms = now.lo;

            /* Timing evidence: passes and busy time per second */
            if ((now.lo - rate_t0_ms) >= LOOP_RATE_WINDOW_MS)
            {
                g_loop_rate = (rate_loops * 1000u) / (now.lo - rate_t0_ms);
                g_busy_us_per_s = (g_busy_acc_us * 1000u) / (now.lo - rate_t0_ms);
                g_busy_max_us = g_busy_pass_max_us;
                g_busy_acc_us = 0u;
                g_busy_pass_max_us = 0u;
                rate_loops = 0u;
                rate_t0_ms = now.lo;
            }
        }

        loop_count++;
        rate_loops++;

        busy_us = in_w(TIMER_TIMERAWL) - t_pass_us;
        g_busy_acc_us += busy_us;
        if (busy_us > g_busy_pass_max_us)
        {
            g_busy_pass_max_us = busy_us;
        }

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
    {
        /* Paint the stack so motion_stack_used() can find the high-water mark */
        uint32_t *p_w = (uint32_t *)motion_task_stack;
        uint32_t i;

        for (i = 0u; i < (MOTION_STACK_SIZE / 4u); i++)
        {
            p_w[i] = STACK_FILL_WORD;
        }
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

/* No print on success: callers may run at a higher priority than the
 * motion task, and a blocking UART print would stall the control loop. */
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
motion_stop(void)
{
    g_stop_req = 1u;
}

void
motion_set_hump_mask(bool on_hump)
{
    g_hump_mask = on_hump ? 1u : 0u;
}

uint32_t
motion_loop_rate(void)
{
    return g_loop_rate;
}

void
motion_get_cpu(uint32_t *p_busy_us_per_s, uint32_t *p_max_pass_us)
{
    if (NULL != p_busy_us_per_s)
    {
        *p_busy_us_per_s = g_busy_us_per_s;
    }
    if (NULL != p_max_pass_us)
    {
        *p_max_pass_us = g_busy_max_us;
    }
}

uint32_t
motion_stack_used(void)
{
    const uint32_t *p_w = (const uint32_t *)motion_task_stack;
    uint32_t untouched = 0u;

    if (NULL == p_w)
    {
        return 0u;
    }
    /* The stack grows down from the top of the buffer, so the painted
     * words left at the bottom were never used */
    while ((untouched < (MOTION_STACK_SIZE / 4u)) && (STACK_FILL_WORD == p_w[untouched]))
    {
        untouched++;
    }
    return MOTION_STACK_SIZE - (untouched * 4u);
}

uint32_t
motion_stack_size(void)
{
    return MOTION_STACK_SIZE;
}

bool
motion_is_steady(void)
{
    return g_closed_loop
        && (0 != g_pi_left.target_eps) && (0 != g_pi_right.target_eps)
        && (g_pi_left.setpoint_eps == g_pi_left.target_eps)
        && (g_pi_right.setpoint_eps == g_pi_right.target_eps);
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
    UINT sts;
    int32_t l;
    int32_t r;
    int32_t l_spd;
    int32_t r_spd;

    if (NULL == p_odom)
    {
        return E_PAR;
    }

    /* One consistent snapshot: both counts and speeds from the same pass */
    CRIT_ENTER(&g_odom_lock, sts);
    l = g_hw_left_count;
    r = g_hw_right_count;
    l_spd = g_left_speed_tenths_cm_s;
    r_spd = g_right_speed_tenths_cm_s;
    CRIT_EXIT(&g_odom_lock, sts);

    p_odom->left_pulses = l;
    p_odom->right_pulses = r;
    p_odom->left_speed_cm_s = ((float)l_spd) / 10.0f;
    p_odom->right_speed_cm_s = ((float)r_spd) / 10.0f;
    p_odom->distance_cm = ((float)(((l + r) / 2) * UM_PER_EDGE)) / 10000.0f;

    tk_get_tim(&now);
    p_odom->timestamp_ms = now.lo;

    return E_OK;
}

void
motion_reset_odometry(void)
{
    UINT sts;

    /* The PI's last-count moves with the counts, or its next speed sample
     * would see the reset as a huge jump */
    CRIT_ENTER(&g_odom_lock, sts);
    g_last_left_count = 0;
    g_last_right_count = 0;
    g_left_speed_tenths_cm_s = 0;
    g_right_speed_tenths_cm_s = 0;
    g_hw_left_edges = 0u;
    g_hw_right_edges = 0u;
    g_hw_left_count = 0;
    g_hw_right_count = 0;
    g_pi_left.last_count = 0;
    g_pi_right.last_count = 0;
    CRIT_EXIT(&g_odom_lock, sts);
}

ER
motion_get_encoder_diag(enc_diag_t *p_left, enc_diag_t *p_right)
{
    UINT sts;

    if ((NULL == p_left) || (NULL == p_right))
    {
        return E_PAR;
    }

    CRIT_ENTER(&g_odom_lock, sts);
    p_left->hw_edges = g_hw_left_edges;
    p_left->hw_count = g_hw_left_count;
    p_right->hw_edges = g_hw_right_edges;
    p_right->hw_count = g_hw_right_count;
    CRIT_EXIT(&g_odom_lock, sts);

    return E_OK;
}

void
motion_print_encoder_diag(const char *p_label)
{
    enc_diag_t left;
    enc_diag_t right;

    (void)motion_get_encoder_diag(&left, &right);

    tm_printf((UB *)"[HW]  %s L: edges=%lu cnt=%ld | R: edges=%lu cnt=%ld\n",
              (NULL != p_label) ? p_label : "",
              left.hw_edges, left.hw_count,
              right.hw_edges, right.hw_count);
}

/*
 * Drive straight for distance_cm under PI control, then stop.
 *
 * The stop is commanded early by the braking distance: after a 0 mm/s
 * command the PI setpoint ramps down at PID_SLEW_EPS per period, so the
 * wheels cover v^2 / (2a) while slowing, plus about two control periods of
 * lag behind the setpoint. No printing while the motors run: the caller
 * usually outranks the motion task and a blocking print would stall it.
 */
ER
motion_move_forward_cm(uint16_t distance_cm, uint16_t speed_mm_s)
{
    ER err;
    int32_t target_edges;
    int32_t v_mm_s;
    int32_t v_eps;
    int32_t decel_eps_per_s;
    int32_t brake_edges;
    int32_t progress;
    int32_t last_progress;
    int32_t final_mm;
    uint32_t still_ms;
    uint32_t waited_ms;

    if (0u == distance_cm)
    {
        tm_printf((UB *)"[MOTION] Invalid distance: 0 cm\n");
        return E_PAR;
    }

    v_mm_s = clamp_i32((int32_t)speed_mm_s, MOVE_MIN_SPEED_MM_S, PID_MAX_SPEED_MM_S);
    v_eps = mm_s_to_eps(v_mm_s);
    target_edges = ((int32_t)distance_cm * 10L * 1000L) / UM_PER_EDGE;

    decel_eps_per_s = (PID_SLEW_EPS * 1000L) / (int32_t)PID_PERIOD_MS;
    brake_edges = (v_eps * v_eps) / (2L * decel_eps_per_s)
                + (v_eps * 2L * (int32_t)PID_PERIOD_MS) / 1000L;

    tm_printf((UB *)"[MOTION] move_forward_cm: %u cm at %ld mm/s "
              "(target %ld edges, brake at -%ld)\n",
              distance_cm, v_mm_s, target_edges, brake_edges);

    motion_reset_odometry();

    err = motion_set_velocity((int16_t)v_mm_s, (int16_t)v_mm_s);
    if (E_OK != err)
    {
        return err;
    }

    do
    {
        tk_dly_tsk(MOVE_POLL_MS);
        progress = average_progress_counts();
    } while ((progress + brake_edges) < target_edges);

    err = motion_set_velocity(0, 0);
    if (E_OK != err)
    {
        return err;
    }

    /* Wait until the wheels have stopped, or give up after MOVE_SETTLE_MAX_MS */
    last_progress = average_progress_counts();
    still_ms = 0u;
    waited_ms = 0u;
    while ((still_ms < MOVE_STILL_MS) && (waited_ms < MOVE_SETTLE_MAX_MS))
    {
        tk_dly_tsk(MOVE_POLL_MS);
        waited_ms += MOVE_POLL_MS;
        progress = average_progress_counts();
        still_ms = (progress == last_progress) ? (still_ms + MOVE_POLL_MS) : 0u;
        last_progress = progress;
    }

    final_mm = (progress * UM_PER_EDGE) / 1000L;
    tm_printf((UB *)"[MOTION] Done: target %u mm, encoders %ld mm "
              "(error %ld mm)\n",
              (unsigned int)(distance_cm * 10u), final_mm,
              final_mm - ((int32_t)distance_cm * 10L));

    return E_OK;
}