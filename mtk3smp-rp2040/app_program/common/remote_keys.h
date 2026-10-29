/**
 * @file remote_keys.h
 * @brief Keys typed on the laptop (Buddy 1, topic car27/cmd), fed into the
 * console input so the test menus treat them like serial keystrokes.
 *
 * Plain C types only: called from the radio task's lwIP side, which must
 * not include tk/tkernel.h (see the size_t note in cyw43_utk.h).
 */

#ifndef COMMON_REMOTE_KEYS_H
#define COMMON_REMOTE_KEYS_H

#include <stdint.h>

/** @brief Queue one key for console_try_getc() (dropped if 16 wait). */
void console_inject(uint8_t c);

#endif /* COMMON_REMOTE_KEYS_H */
