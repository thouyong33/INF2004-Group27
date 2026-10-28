/*
 * test_modes.c
 * Calls the test mode selected in test_modes.h.
 */

#include "test_common.h"

void motion_test_hand(void);
void motion_test_pi_step(void);
void ir_test_log(void);
void motion_test_safety(void);
void motion_test_cpu(void);
void imu_test_ramp_roll(void);
void ctrl_test_course(void);
void barcode_test_drive(void);
void obst_test_width(void);

void
test_mode_run(void)
{
#if MOTION_TEST_MODE == MOTION_TEST_HAND
    motion_test_hand();
#elif MOTION_TEST_MODE == MOTION_TEST_PI_STEP
    motion_test_pi_step();
#elif MOTION_TEST_MODE == IR_TEST_LOG
    ir_test_log();
#elif MOTION_TEST_MODE == MOTION_TEST_SAFETY
    motion_test_safety();
#elif MOTION_TEST_MODE == MOTION_TEST_CPU
    motion_test_cpu();
#elif MOTION_TEST_MODE == IMU_TEST_RAMP_ROLL
    imu_test_ramp_roll();
#elif MOTION_TEST_MODE == CTRL_TEST_COURSE
    ctrl_test_course();
#elif MOTION_TEST_MODE == BARCODE_TEST_DRIVE
    barcode_test_drive();
#elif MOTION_TEST_MODE == OBST_TEST_WIDTH
    obst_test_width();
#else
#error "MOTION_TEST_MODE is not one of the modes in test_modes.h"
#endif
}
