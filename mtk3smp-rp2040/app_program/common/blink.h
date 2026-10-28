/*
 * Copyright (c) 2026 Muhamed Fauzi Bin Abbas
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * blink.h
 * Health LED task, independent of the console (from the port's demo/).
 */

#ifndef COMMON_BLINK_H
#define COMMON_BLINK_H

#include <tk/tkernel.h>

/**
 * @brief Create and start the blink task (priority 10, lowest of all).
 * Deliberately touches nothing the console touches: if output stops but the
 * LED keeps blinking, the console is at fault; if the LED freezes, the
 * kernel is.
 */
ER blink_task_create(void);

#endif /* COMMON_BLINK_H */
