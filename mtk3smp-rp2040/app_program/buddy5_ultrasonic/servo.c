/**
 * @file servo.c
 * @brief Buddy 5: scan servo, hardware PWM on GP12 at 50 Hz.
 *
 * clk_sys 125 MHz / 125 = 1 MHz counter, so one count is one microsecond:
 * TOP = 19999 gives a 20 ms frame and CC is the pulse width in us.
 */

#include <stdint.h>
#include <tk/tkernel.h>
#include <sys/sysdef.h>

#include "../common/pin_defs.h"
#include "servo.h"

#define SERVO_SLICE              ((SERVO_PWM >> 1) & 0x7u)
#define PWM_REG(off)             (PWM_BASE + (SERVO_SLICE * 0x14u) + (off))
#define PWM_DIV_INT(n)           ((uint32_t)(n) << 4)
#define PWM_CSR_EN               (1u << 0)
#define SERVO_FRAME_TOP          19999u

#if ((SERVO_PWM & 1) != 0)
#error "Servo pin must be a PWM A channel (even GPIO): CC is written to the A half"
#endif

static uint16_t g_min_us = SERVO_ABS_MIN_US;
static uint16_t g_max_us = SERVO_ABS_MAX_US;
static uint16_t g_pulse_us = 1500u;

void
servo_init(uint16_t start_us)
{
    /* The motion task normally releases PWM from reset first; make sure */
    if (0u != (in_w(RESETS_RESET) & RESETS_RESET_PWM))
    {
        clr_w(RESETS_RESET, RESETS_RESET_PWM);
        while (0u == (in_w(RESETS_RESET_DONE) & RESETS_RESET_PWM))
        {
            ;
        }
    }

    out_w(PWM_REG(PWM_CHx_CSR), 0u);
    out_w(PWM_REG(PWM_CHx_DIV), PWM_DIV_INT(SYSCLK));     /* SYSCLK MHz -> 1 MHz */
    out_w(PWM_REG(PWM_CHx_TOP), SERVO_FRAME_TOP);
    out_w(PWM_REG(PWM_CHx_CTR), 0u);
    (void)servo_set_us(start_us);
    out_w(PWM_REG(PWM_CHx_CSR), PWM_CSR_EN);

    out_w(GPIO_CTRL(SERVO_PWM), GPIO_CTRL_FUNCSEL_PWM);
}

void
servo_set_limits(uint16_t min_us, uint16_t max_us)
{
    g_min_us = (min_us < SERVO_ABS_MIN_US) ? SERVO_ABS_MIN_US : min_us;
    g_max_us = (max_us > SERVO_ABS_MAX_US) ? SERVO_ABS_MAX_US : max_us;
}

uint16_t
servo_set_us(uint16_t pulse_us)
{
    uint32_t cc;

    if (pulse_us < g_min_us)
    {
        pulse_us = g_min_us;
    }
    if (pulse_us > g_max_us)
    {
        pulse_us = g_max_us;
    }

    /* Channel A is the low half of CC; keep channel B (GP13, unused) */
    cc = in_w(PWM_REG(PWM_CHx_CC));
    out_w(PWM_REG(PWM_CHx_CC), (cc & 0xFFFF0000u) | pulse_us);
    g_pulse_us = pulse_us;
    return pulse_us;
}

uint16_t
servo_get_us(void)
{
    return g_pulse_us;
}

void
servo_relax(void)
{
    uint32_t cc;

    cc = in_w(PWM_REG(PWM_CHx_CC));
    out_w(PWM_REG(PWM_CHx_CC), cc & 0xFFFF0000u);
}
