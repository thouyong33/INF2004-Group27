/**
 * @file console_in.c
 * @brief Non-blocking keystroke input from the UART0 console.
 */

#include <tk/tkernel.h>
#include <sys/sysdef.h>

#include "console_in.h"

/* PL011 UART0, the monitor console (RP2040 datasheet 4.2.8) */
#define UART0_DR                 (0x40034000u + 0x000u)
#define UART0_FR                 (0x40034000u + 0x018u)
#define UART_FR_RXFE             (1u << 4)

INT
console_try_getc(void)
{
    if (0u != (in_w(UART0_FR) & UART_FR_RXFE))
    {
        return -1;
    }
    return (INT)(in_w(UART0_DR) & 0xFFu);
}

void
console_flush_input(void)
{
    while (console_try_getc() >= 0)
    {
        ;
    }
}
