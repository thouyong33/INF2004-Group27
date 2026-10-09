/**
 * @file console_in.h
 * @brief Non-blocking keystroke input from the UART0 console (GP1 RX).
 *
 * Do not use the monitor's tm_getchar() for this: it waits for a character
 * with interrupts disabled, which freezes the kernel tick and every task
 * (including motor control) until a key arrives.
 */

#ifndef COMMON_CONSOLE_IN_H
#define COMMON_CONSOLE_IN_H

#include <tk/tkernel.h>

/** @brief Return the next received character, or -1 if none is waiting. */
INT console_try_getc(void);

/** @brief Discard any characters already received. */
void console_flush_input(void);

#endif /* COMMON_CONSOLE_IN_H */
