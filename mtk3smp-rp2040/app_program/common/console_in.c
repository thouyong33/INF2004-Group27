/**
 * @file console_in.c
 * @brief Non-blocking keystroke input from the UART0 console, plus the
 * laptop's keys over MQTT (remote_keys.h).
 */

#include <tk/tkernel.h>
#include <sys/sysdef.h>

#include "console_in.h"
#include "remote_keys.h"

/* PL011 UART0, the monitor console (RP2040 datasheet 4.2.8) */
#define UART0_DR                 (0x40034000u + 0x000u)
#define UART0_FR                 (0x40034000u + 0x018u)
#define UART_FR_RXFE             (1u << 4)

/* Remote keys: one writer (the radio task), one reader (the test task),
   so a ring with separate head and tail needs no lock. */
#define REMOTE_LEN               16u

static volatile uint8_t g_remote[REMOTE_LEN];
static volatile uint32_t g_remote_head = 0u;   /* written by the radio */
static volatile uint32_t g_remote_tail = 0u;   /* written by the reader */

void
console_inject(uint8_t c)
{
    uint32_t head = g_remote_head;

    if ((head - g_remote_tail) < REMOTE_LEN)
    {
        g_remote[head % REMOTE_LEN] = c;
        __asm__ volatile ("" ::: "memory");     /* key before the index */
        g_remote_head = head + 1u;
    }
}

INT
console_try_getc(void)
{
    uint32_t tail = g_remote_tail;
    INT c;

    if (0u == (in_w(UART0_FR) & UART_FR_RXFE))
    {
        return (INT)(in_w(UART0_DR) & 0xFFu);
    }
    if (tail == g_remote_head)
    {
        return -1;
    }
    c = (INT)g_remote[tail % REMOTE_LEN];
    g_remote_tail = tail + 1u;
    return c;
}

void
console_flush_input(void)
{
    while (console_try_getc() >= 0)
    {
        ;
    }
}
