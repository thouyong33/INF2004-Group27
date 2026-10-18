/**
 * @file ir_sensor.c
 * @brief Buddy 3: IR module inputs, read straight from SIO.
 */

#include <stdint.h>
#include <tk/tkernel.h>
#include <sys/sysdef.h>

#include "../common/pin_defs.h"
#include "ir_sensor.h"

/* RP2040 ADC (datasheet 4.9). Polled: the kernel's ADC device driver is
 * not used, so its interrupt stays off. clk_adc (48 MHz) is started by
 * the port at boot; one conversion takes 96 cycles = 2 us. */
#define B3_ADC_CS               (0x4004C000u + 0x00u)
#define B3_ADC_RESULT           (0x4004C000u + 0x04u)
#define B3_ADC_CS_EN            (1u << 0)
#define B3_ADC_CS_START_ONCE    (1u << 2)
#define B3_ADC_CS_READY         (1u << 8)
#define B3_ADC_CS_AINSEL_POS    12u
#define B3_ADC_AVG              4u

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

void
ir_ao_init(void)
{
    if (0u != (in_w(RESETS_RESET) & RESETS_RESET_ADC))
    {
        clr_w(RESETS_RESET, RESETS_RESET_ADC);
        while (0u == (in_w(RESETS_RESET_DONE) & RESETS_RESET_ADC))
        {
            ;
        }
    }

    /* Analog pin: no digital function, input buffer and pulls off */
    out_w(GPIO_CTRL(IR_LEFT_AOUT), GPIO_CTRL_FUNCSEL_NULL);
    out_w(GPIO(IR_LEFT_AOUT), 0u);

    out_w(B3_ADC_CS, B3_ADC_CS_EN
                     | ((uint32_t)(IR_LEFT_AOUT - 26) << B3_ADC_CS_AINSEL_POS));
    while (0u == (in_w(B3_ADC_CS) & B3_ADC_CS_READY))
    {
        ;
    }
}

uint32_t
ir_ao_left_mv(void)
{
    uint32_t sum = 0u;
    uint32_t i;

    for (i = 0u; i < B3_ADC_AVG; i++)
    {
        set_w(B3_ADC_CS, B3_ADC_CS_START_ONCE);
        while (0u == (in_w(B3_ADC_CS) & B3_ADC_CS_READY))
        {
            ;
        }
        sum += in_w(B3_ADC_RESULT) & 0x0FFFu;
    }
    return (sum * 3300u) / (B3_ADC_AVG * 4096u);
}
