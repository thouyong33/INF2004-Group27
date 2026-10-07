/**
 * @file motion_task.c
 * @brief Motion control with speed calculation and distance tracking
 *
 * Current implementation notes:
 * - Motor drive is DIGITAL only, not PWM yet.
 * - Any positive command = full forward.
 * - Any negative command = full reverse.
 * - Zero = stop.
 * - Distance move currently uses encoder count progress only.
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
#define COUNTS_PER_REV_LEFT       21u
#define COUNTS_PER_REV_RIGHT      21u
#define WHEEL_DIAMETER_MM         65u
#define WHEEL_CIRCUMFERENCE_MM    204u

/* Measured from manual calibration */
#define MM_PER_COUNT_LEFT_X100    953L   /* 9.53 mm/count */
#define MM_PER_COUNT_RIGHT_X100   976L   /* 9.76 mm/count */
#define MM_PER_COUNT_AVG_X100     ((MM_PER_COUNT_LEFT_X100 + MM_PER_COUNT_RIGHT_X100) / 2L)

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

/* Speed state */
static int32_t g_last_left_count = 0;
static int32_t g_last_right_count = 0;
static int32_t g_left_speed_tenths_cm_s = 0;   /* scaled x10 */
static int32_t g_right_speed_tenths_cm_s = 0;  /* scaled x10 */
static uint32_t g_last_speed_calc_ms = 0u;

/* Current commanded state */
static int8_t g_current_left_speed_cmd = 0;
static int8_t g_current_right_speed_cmd = 0;

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

    left_progress = abs_i32(g_enc_left_count);
    right_progress = abs_i32(g_enc_right_count);

    return min_i32(left_progress, right_progress);
}

/*----------------------------------------------------------------------------
 * Motor Control
 *---------------------------------------------------------------------------*/
static void
motor_gpio_init(void)
{
    out_w(GPIO_CTRL(MOTOR_LEFT_FWD), GPIO_FUNCTION_SIO);
    out_w(GPIO_CTRL(MOTOR_LEFT_REV), GPIO_FUNCTION_SIO);
    out_w(GPIO_CTRL(MOTOR_RIGHT_FWD), GPIO_FUNCTION_SIO);
    out_w(GPIO_CTRL(MOTOR_RIGHT_REV), GPIO_FUNCTION_SIO);

    out_w(GPIO_OE_SET,
          (1u << MOTOR_LEFT_FWD) |
          (1u << MOTOR_LEFT_REV) |
          (1u << MOTOR_RIGHT_FWD) |
          (1u << MOTOR_RIGHT_REV));

    out_w(GPIO_OUT_CLR,
          (1u << MOTOR_LEFT_FWD) |
          (1u << MOTOR_LEFT_REV) |
          (1u << MOTOR_RIGHT_FWD) |
          (1u << MOTOR_RIGHT_REV));

    tm_printf((UB *)"[MOTION] GPIO initialized\n");
}

static void
motor_gpio_stop(void)
{
    out_w(GPIO_OUT_CLR,
          (1u << MOTOR_LEFT_FWD) |
          (1u << MOTOR_LEFT_REV) |
          (1u << MOTOR_RIGHT_FWD) |
          (1u << MOTOR_RIGHT_REV));
}

static void
motor_gpio_set(int8_t left, int8_t right)
{
    if (left > 100)
    {
        left = 100;
    }
    else if (left < -100)
    {
        left = -100;
    }

    if (right > 100)
    {
        right = 100;
    }
    else if (right < -100)
    {
        right = -100;
    }

    /* Left motor */
    if (left > 0)
    {
        out_w(GPIO_OUT_SET, (1u << MOTOR_LEFT_FWD));
        out_w(GPIO_OUT_CLR, (1u << MOTOR_LEFT_REV));
    }
    else if (left < 0)
    {
        out_w(GPIO_OUT_CLR, (1u << MOTOR_LEFT_FWD));
        out_w(GPIO_OUT_SET, (1u << MOTOR_LEFT_REV));
    }
    else
    {
        out_w(GPIO_OUT_CLR, (1u << MOTOR_LEFT_FWD) | (1u << MOTOR_LEFT_REV));
    }

    /* Right motor */
    if (right > 0)
    {
        out_w(GPIO_OUT_SET, (1u << MOTOR_RIGHT_FWD));
        out_w(GPIO_OUT_CLR, (1u << MOTOR_RIGHT_REV));
    }
    else if (right < 0)
    {
        out_w(GPIO_OUT_CLR, (1u << MOTOR_RIGHT_FWD));
        out_w(GPIO_OUT_SET, (1u << MOTOR_RIGHT_REV));
    }
    else
    {
        out_w(GPIO_OUT_CLR, (1u << MOTOR_RIGHT_FWD) | (1u << MOTOR_RIGHT_REV));
    }
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

    tm_printf((UB *)"[MOTION] Encoders initialized (L=%u R=%u counts/rev)\n",
              COUNTS_PER_REV_LEFT,
              COUNTS_PER_REV_RIGHT);
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

    delta_left = g_enc_left_count - g_last_left_count;
    delta_right = g_enc_right_count - g_last_right_count;

    /* speed_tenths_cm_s = (delta_counts * mm_per_count * 1000 ms/s) / (dt_ms * 10 mm/cm * 100 scale)
     * Simplified:
     * speed_x10_cm_s = (delta_counts * mm_per_count_x100) / dt_ms
     */
    g_left_speed_tenths_cm_s = (delta_left * MM_PER_COUNT_LEFT_X100) / (int32_t)dt_ms;
    g_right_speed_tenths_cm_s = (delta_right * MM_PER_COUNT_RIGHT_X100) / (int32_t)dt_ms;

    g_last_left_count = g_enc_left_count;
    g_last_right_count = g_enc_right_count;
}

static int32_t
distance_cm_from_counts(void)
{
    int32_t avg_counts;
    int32_t distance_mm_x100;
    int32_t distance_cm;

    avg_counts = (g_enc_left_count + g_enc_right_count) / 2;
    distance_mm_x100 = avg_counts * MM_PER_COUNT_AVG_X100;
    distance_cm = distance_mm_x100 / 1000L;

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
        msg_size = tk_rcv_mbf(motion_msgbuf_id, &cmd, TMO_POL);
        if ((INT)sizeof(cmd) == msg_size)
        {
            motor_gpio_set(cmd.left_speed, cmd.right_speed);
            g_current_left_speed_cmd = cmd.left_speed;
            g_current_right_speed_cmd = cmd.right_speed;

            tm_printf((UB *)"[MOTOR] Cmd received: L=%d R=%d\n",
                      (int)cmd.left_speed,
                      (int)cmd.right_speed);
        }
        else if (E_TMOUT != msg_size)
        {
            tm_printf((UB *)"[MOTOR] tk_rcv_mbf error: %d\n", msg_size);
        }

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

            if (((loop_count / (SPEED_CALC_INTERVAL_MS / TASK_PERIOD_MS)) % print_divider) == 0u)
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
                          g_enc_left_count,
                          g_enc_right_count,
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

ER
motion_set_speed(int8_t left, int8_t right)
{
    motor_cmd_t cmd;
    ER err;

    if (motion_msgbuf_id <= 0)
    {
        return E_NOEXS;
    }

    cmd.left_speed = left;
    cmd.right_speed = right;

    err = tk_snd_mbf(motion_msgbuf_id, &cmd, (INT)sizeof(cmd), TMO_FEVR);
    if (E_OK != err)
    {
        tm_printf((UB *)"[MOTOR] tk_snd_mbf failed: %d\n", err);
    }
    else
    {
        tm_printf((UB *)"[MOTOR] Cmd sent: L=%d R=%d\n",
                  (int)left,
                  (int)right);
    }

    return err;
}

ER
motion_get_odometry(odometry_t *p_odom)
{
    SYSTIM now;
    int32_t avg_counts;
    int32_t distance_mm_x100;

    if (NULL == p_odom)
    {
        return E_PAR;
    }

    p_odom->left_pulses = g_enc_left_count;
    p_odom->right_pulses = g_enc_right_count;

    p_odom->left_speed_cm_s = ((float)g_left_speed_tenths_cm_s) / 10.0f;
    p_odom->right_speed_cm_s = ((float)g_right_speed_tenths_cm_s) / 10.0f;

    avg_counts = (g_enc_left_count + g_enc_right_count) / 2;
    distance_mm_x100 = avg_counts * MM_PER_COUNT_AVG_X100;
    p_odom->distance_cm = ((float)distance_mm_x100) / 1000.0f;

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
    target_counts = (distance_mm * 100L) / MM_PER_COUNT_AVG_X100;

    if (target_counts <= 0)
    {
        tm_printf((UB *)"[MOTION] Invalid target count calculation\n");
        return E_SYS;
    }

    tm_printf((UB *)"[MOTION] move_forward_cm: target=%u cm, speed=%u\n",
              distance_cm,
              speed);
    tm_printf((UB *)"[MOTION] Target counts=%ld avg_mm_per_count_x100=%ld\n",
              target_counts,
              MM_PER_COUNT_AVG_X100);

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
              g_enc_left_count,
              g_enc_right_count,
              min_progress);
    motion_print_encoder_diag("move_forward_cm");

    return E_OK;
}