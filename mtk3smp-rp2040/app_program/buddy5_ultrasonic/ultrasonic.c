/**
 * @file ultrasonic.c
 * @brief Buddy 5: HC-SR04 driver, polled with the RP2040 microsecond timer.
 */

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>
#include <sys/sysdef.h>

#include "../common/pin_defs.h"
#include "ultrasonic.h"

/*
 * The port's hw_setting.c does not release TIMER from reset, and the
 * timer counts watchdog ticks, which are only generated once the tick
 * divider is enabled. clk_ref runs from the 12 MHz crystal, so 12 cycles
 * per tick gives the 1 MHz the timer expects (RP2040 datasheet 4.6/4.7).
 */
#define WATCHDOG_TICK            (0x40058000u + 0x2Cu)
#define WATCHDOG_TICK_ENABLE     (1u << 9)
#define WATCHDOG_TICK_CYCLES     12u

#define TRIG_PULSE_US            10u
#define ECHO_START_TIMEOUT_US    5000u   /* echo should rise within ~0.5 ms */
#define ECHO_MAX_US              25000u  /* ~4.3 m, the sensor's stated range */

void
ultrasonic_init(void)
{
    if (0u != (in_w(RESETS_RESET) & RESETS_RESET_TIMER))
    {
        clr_w(RESETS_RESET, RESETS_RESET_TIMER);
        while (0u == (in_w(RESETS_RESET_DONE) & RESETS_RESET_TIMER))
        {
            ;
        }
    }
    out_w(WATCHDOG_TICK, WATCHDOG_TICK_ENABLE | WATCHDOG_TICK_CYCLES);

    /* TRIG: SIO output, low */
    out_w(GPIO_CTRL(ULTRASONIC_TRIG), GPIO_CTRL_FUNCSEL_SIO);
    out_w(GPIO_OUT_CLR, 1u << ULTRASONIC_TRIG);
    out_w(GPIO_OE_SET, 1u << ULTRASONIC_TRIG);

    /* ECHO: boot leaves GP28 as an ADC pin with its input buffer off,
     * so turn the input back on. Pull-down keeps it low if unplugged. */
    out_w(GPIO(ULTRASONIC_ECHO), GPIO_IE | GPIO_PDE | GPIO_SHEMITT);
    out_w(GPIO_CTRL(ULTRASONIC_ECHO), GPIO_CTRL_FUNCSEL_SIO);
    out_w(GPIO_OE_CLR, 1u << ULTRASONIC_ECHO);
}

uint32_t
us_now(void)
{
    return (uint32_t)in_w(TIMER_TIMERAWL);
}

void
us_delay(uint32_t us)
{
    uint32_t start = us_now();

    while ((us_now() - start) < us)
    {
        ;
    }
}

static bool
echo_high(void)
{
    return 0u != (in_w(GPIO_IN) & (1u << ULTRASONIC_ECHO));
}

ER
ultrasonic_read_us(uint32_t *p_echo_us)
{
    uint32_t t0;
    uint32_t rise;

    if (NULL == p_echo_us)
    {
        return E_PAR;
    }

    if (echo_high())
    {
        return E_TMOUT;             /* still ringing from the last ping */
    }

    out_w(GPIO_OUT_SET, 1u << ULTRASONIC_TRIG);
    us_delay(TRIG_PULSE_US);
    out_w(GPIO_OUT_CLR, 1u << ULTRASONIC_TRIG);

    t0 = us_now();
    while (!echo_high())
    {
        if ((us_now() - t0) > ECHO_START_TIMEOUT_US)
        {
            return E_TMOUT;
        }
    }

    rise = us_now();
    while (echo_high())
    {
        if ((us_now() - rise) > ECHO_MAX_US)
        {
            return E_TMOUT;
        }
    }

    *p_echo_us = us_now() - rise;
    return E_OK;
}

int32_t
ultrasonic_us_to_mm(uint32_t echo_us)
{
    return (int32_t)((echo_us * 343u) / 2000u);
}
