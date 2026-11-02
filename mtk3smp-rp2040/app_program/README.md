# app_program: the robot car

The car's application on µT-Kernel 3.0 (Pico W). Each team member ("buddy")
owns one folder; the vehicle controller ties them together. Build and flash
steps are in [`README_QUICKSTART.md`](../../README_QUICKSTART.md).

## Start-up

`app_main.c` starts the blink task (and the Wi-Fi status task on a Wi-Fi
build), then the motion task, then runs the test mode chosen in
`test_modes/test_modes.h`. The default, the course test (mode 31), creates
the vehicle controller, which creates the line, barcode and obstacle tasks.

## Tasks

Lower number = higher priority.

| Task | Priority | File | Job |
|---|---|---|---|
| motion | 5 | `buddy2_motion/motion_task.c` | 2 ms loop: wheel encoders, speed PI control, turns, stall and encoder faults |
| obstacle | 6 | `buddy5_ultrasonic/obstacle.c` | ultrasonic pings while following; posts `EVT_OBSTACLE` |
| controller | 6 | `controller/vehicle_ctrl.c` | the run's state machine: events, ramp, obstacle probe and avoid, barcode commands, `robot_status` |
| line | 7 | `buddy3_line_barcode/line.c` | 20 ms loop: right-edge line following, curves, junctions, ramp hold |
| barcode | 8 | `buddy3_line_barcode/barcode_scan.c` | samples the left IR per encoder step and decodes Code 39 (`barcode.c`) |
| Wi-Fi status | 8 | `buddy1_telemetry/wifi_status.c` | Wi-Fi build only: link and MQTT status |
| blink | 10 | `common/blink.c` | status LED (GP16) |

On a Wi-Fi build the MQTT client (`lib/libnet/lwip/lwip_utk_mqtt.c`, Buddy 1)
runs in the radio task and calls the hooks in `buddy1_telemetry/tel_source.c`
for telemetry, events and run commands.

## Folders

| Folder | Buddy | Contents |
|---|---|---|
| `buddy1_telemetry/` | 1 | MQTT hooks (telemetry packet, events, commands), Wi-Fi status |
| `buddy2_motion/` | 2 | motors, encoders, PI speed control, odometry |
| `buddy3_line_barcode/` | 3 | IR sensors, line following, barcode scanning and decoding |
| `buddy4_imu/` | 4 | GY-511 accelerometer (pitch, roll), ramp height (`terrain.c`) |
| `buddy5_ultrasonic/` | 5 | HC-SR04 ultrasonic, obstacle watch, servo (held centred) |
| `controller/` | all | vehicle controller (`vehicle_ctrl.c`) |
| `common/` | all | pins, fault flags, event queue, console input, blink, per-task CPU meter (`cpu_meter.c`) |
| `test_modes/` | all | test modes; one is built into each UF2 |

## Pins

All in `common/pin_defs.h`:

| Pin | Use |
|---|---|
| GP0, GP1 | UART console (TX, RX) |
| GP2, GP3 | I2C SDA, SCL (IMU) |
| GP4 | centre IR (digital) |
| GP6 | left IR (digital) |
| GP26 | right IR (digital) |
| GP27 | left IR analog (ADC1, barcode) |
| GP7, GP28 | ultrasonic trigger, echo |
| GP8, GP9 | left motor forward, reverse (PWM 4) |
| GP10, GP11 | right motor forward, reverse (PWM 5) |
| GP12 | servo (PWM 6A) |
| GP14, GP19 | left encoder A, B |
| GP15, GP17 | right encoder A, B |
| GP16 | status LED |

## How the parts talk

- Tasks post events (`common/ctrl_events.h`) to the controller's queue: line
  end, junctions, barcodes, obstacles. The controller logs the latest 64 of
  each run; the test modes print them.
- Faults (`common/faults.h`) are bit flags any task can set; the controller
  stops the run on a motion fault.
- The controller publishes a `robot_status` snapshot every period
  (`controller/vehicle_ctrl.h`), which the test log and the MQTT telemetry
  both read.

## Test modes

Listed in `test_modes/test_modes.h`. The course test (31) is the default and
covers the whole car; the others are bench checks (encoders by hand, speed
control step, live IR readout, motion safety, CPU load, barcode drive). The
numbers match the ones in `docs/tribunal1/cr1_evidence.md`.
