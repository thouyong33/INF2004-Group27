/*
 * test_modes.h
 * Test modes of the robot car. Exactly one is built into each UF2, picked
 * by MOTION_TEST_MODE (default below, or -DMOTION_TEST_MODE=n). usermain()
 * creates the motion task and then calls test_mode_run(). The numbers are
 * the ones the test logs and docs/tribunal1/cr1_evidence.md refer to;
 * modes retired on 28 Oct 2026 are in git history (commit 7e16bec).
 *
 *   MOTION_TEST_HAND       - motors stay OFF; prints encoder counters
 *                            every second while the wheels are turned
 *                            by hand. Ground truth with no motor noise.
 *   MOTION_TEST_PI_STEP    - closed-loop step response: 300, 500, 150,
 *                            0 mm/s forward, then -300 and 0. Logged in
 *                            RAM and printed afterwards as [TRACE] CSV
 *                            plus a [PI] summary per step.
 *   IR_TEST_LOG            - Buddy 3 step 1: live IR readout, then hand-
 *                            pushed passes printing black/white run widths
 *                            per sensor (encoder distance, motors off).
 *   MOTION_TEST_SAFETY     - Tribunal 1 phase 1, car lifted: loop rate and
 *                            PI error, STOP over queued commands, stall
 *                            (both wheels held), encoder check (one held).
 *   MOTION_TEST_CPU        - motion task CPU (busy us/s, longest pass) idle
 *                            and at 150/300/500 mm/s, and stack high-water.
 *   IMU_TEST_RAMP_ROLL     - phase 2 step 1: 50 cm at 150 mm/s with only the
 *                            right wheel over a ramp (flat/small/tall); logs
 *                            roll, pitch, heading, wheel distances, jolt.
 *   CTRL_TEST_COURSE       - vehicle controller task: course runs (line +
 *                            ramp, or line + obstacle stop), event log and
 *                            robot_status every 500 ms.
 *   BARCODE_TEST_DRIVE     - Buddy 3 barcode: driven scan (50 mm/s) of the
 *                            slit-taped left IR AO, decoder v2 (barcode.c)
 *                            reading Code 39 both ways.
 *   OBST_TEST_WIDTH        - Buddy 5: drive, stop at an object, scan; width
 *                            estimate plus every reading, for calibrating
 *                            the width against objects of known size.
 */

#ifndef TEST_MODES_H
#define TEST_MODES_H

#define MOTION_TEST_HAND         3
#define MOTION_TEST_PI_STEP      5
#define IR_TEST_LOG              18
#define MOTION_TEST_SAFETY       20
#define MOTION_TEST_CPU          21
#define IMU_TEST_RAMP_ROLL       22
#define CTRL_TEST_COURSE         31
#define BARCODE_TEST_DRIVE       32
#define OBST_TEST_WIDTH          33

#ifndef MOTION_TEST_MODE
#define MOTION_TEST_MODE         CTRL_TEST_COURSE
#endif

/** @brief Run the selected test mode (does not return for most modes). */
void test_mode_run(void);

#endif /* TEST_MODES_H */
