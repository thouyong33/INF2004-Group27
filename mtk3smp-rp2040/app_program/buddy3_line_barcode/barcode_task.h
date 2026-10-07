#ifndef BUDDY3_BARCODE_TASK_H
#define BUDDY3_BARCODE_TASK_H

#include <tk/tkernel.h>

/*
 * Barcode sensor test task.
 * Monitors the left IR sensor (GP6) and reports state transitions.
 */

void barcode_test_task(INT stacd, void *exinf);

#endif /* BUDDY3_BARCODE_TASK_H */
