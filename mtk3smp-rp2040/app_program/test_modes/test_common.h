/*
 * test_common.h
 * Headers every test mode file needs.
 */

#ifndef TEST_COMMON_H
#define TEST_COMMON_H

#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>

#include "common/usb_console_compat.h"
#include "buddy3_line_barcode/ir_sensor.h"
#include "buddy3_line_barcode/line.h"
#include "buddy3_line_barcode/barcode.h"
#include "buddy3_line_barcode/barcode_scan.h"
#include "common/pin_defs.h"
#include "buddy2_motion/motion_task.h"
#include "buddy4_imu/imu.h"
#include "buddy4_imu/terrain.h"
#include "buddy5_ultrasonic/ultrasonic.h"
#include "buddy5_ultrasonic/servo.h"
#include "buddy5_ultrasonic/scan.h"
#include "buddy5_ultrasonic/obstacle.h"
#include "common/console_in.h"
#include "common/faults.h"
#include "common/ctrl_events.h"
#include "controller/vehicle_ctrl.h"

#include "test_modes.h"

#endif /* TEST_COMMON_H */
