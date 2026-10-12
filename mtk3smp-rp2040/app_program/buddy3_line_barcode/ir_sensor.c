/**
 * @file ir_sensor.c
 * @brief Buddy 3: IR module inputs, read straight from SIO.
 */

#include <stdint.h>
#include <tk/tkernel.h>
#include <sys/sysdef.h>

#include "../common/pin_defs.h"
#include "ir_sensor.h"

static void
ir_pin_input(uint32_t pin)
{
    /* Input buffer on (GP26-29 boot as ADC pins with it off), Schmitt
     * trigger, no pulls: the modules drive their own outputs. */
    out_w(GPIO(pin), GPIO_IE | GPIO_SHEMITT);
    out_w(GPIO_CTRL(pin), GPIO_CTRL_FUNCSEL_SIO);
    out_w(GPIO_OE_CLR, 1u << pin);
}

void
ir_init(void)
{
    ir_pin_input(IR_LEFT_DOUT);
    ir_pin_input(IR_CENTER_DOUT);
    ir_pin_input(IR_RIGHT_DOUT);
}

uint32_t
ir_read(void)
{
    uint32_t in = in_w(GPIO_IN);
    uint32_t bits = 0u;

    if (0u != (in & (1u << IR_LEFT_DOUT)))
    {
        bits |= IR_BIT_LEFT;
    }
    if (0u != (in & (1u << IR_CENTER_DOUT)))
    {
        bits |= IR_BIT_CENTRE;
    }
    if (0u != (in & (1u << IR_RIGHT_DOUT)))
    {
        bits |= IR_BIT_RIGHT;
    }
    return bits;
}
