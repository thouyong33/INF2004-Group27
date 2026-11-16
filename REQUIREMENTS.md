# Requirements and how to verify them

INF2004 Group 27 robot car on micro T-Kernel 3.0 (Raspberry Pi Pico W on
a Robo Pico board). This file maps each demo requirement to the code that
meets it, the test that shows it, and the evidence log entry.

- **Code:** `mtk3smp-rp2040/app_program/` (one folder per buddy), with
  Buddy 1's MQTT client in `mtk3smp-rp2040/lib/libnet/lwip/`.
- **Evidence log:** [`docs/tribunal1/cr1_evidence.md`](docs/tribunal1/cr1_evidence.md)
  (section numbers below).
- **Build and flash:** [`README_QUICKSTART.md`](README_QUICKSTART.md).
- **Code overview** (tasks, priorities, folders, pins):
  [`mtk3smp-rp2040/app_program/README.md`](mtk3smp-rp2040/app_program/README.md).

## Demo rubric

The final course is an endless circuit in a random order. Each point is
shown by the full-course run: key `f` on the serial menu, or **START** on
the laptop control page. Key `c` is the same run with barcode commands
off, for lapping a test circuit.

| # | Requirement | Where in the code | Test that shows it | Evidence |
|---|---|---|---|---|
| 1 | Detect a barcode | `buddy3_line_barcode/barcode_scan.c`: left IR (GP27 analog) sampled once per encoder step while following | `r` run (read only); `BARCODE_TEST_DRIVE` (mode 32) | §4 |
| 2 | Decode the barcode and obtain the code | `buddy3_line_barcode/barcode.c`: Code 39, both directions, any 1:3 scale, one bar short accepted at either end | `r` run; host test `barcode_decode_test.c` | §4 |
| 3 | Measure hump height | `buddy4_imu/terrain.c`: roll over the one-sided ramp, h = 114 mm × sin(roll) | `l` run (prints the highest ramp) | §5 |
| 4 | Follow the line | `buddy3_line_barcode/line.c`: right-edge following by heading, curves, junctions, lost-line search | `l` and `c` runs | §3 |
| 5 | Follow the instruction from the barcode | `controller/vehicle_ctrl.c` (`junction_turn`); the character-to-movement mapping is sent from the PC over MQTT (`buddy1_telemetry/cmd_map.c`) | `b` run, then `f`; host test `cmd_map_test.c` | §4, §8 |
| 6 | Detect an obstacle | `buddy5_ultrasonic/obstacle.c`: stop when 2 of the last 3 pings read ≤ 300 mm | `o` run | §6 |
| 7 | Measure obstacle width | `controller/vehicle_ctrl.c` (`probe_obstacle`): side-step to each edge | `o` run (prints the width) | §6 |
| 8 | Navigate around the obstacle and rejoin the line | `controller/vehicle_ctrl.c` (`avoid_obstacle`, `avoid_return`) | `a` run | §6 |
| 9 | Send telemetry data via MQTT | `lib/libnet/lwip/lwip_utk_mqtt.c`, `telemetry_codec.c` (11-byte bit-packed status every 500 ms), `buddy1_telemetry/tel_source.c` | Wi-Fi build with `tools/mqtt_viewer.py`; host test `telemetry_codec_test.c` | §8 |
| 10 | Stop the motors when the car is picked up (end of the demo) | `buddy4_imu/terrain.c` (`terrain_lift_update`) and `controller/vehicle_ctrl.c` (`lift_check`, `FAULT_LIFTED`) | lift the car during any run; host test `terrain_lift_test.c` | §5 |

### Other requirements from the brief and the tribunals

| Requirement | Where | Evidence |
|---|---|---|
| Real-time tasks on micro T-Kernel, one controller deciding what the car does from events | `controller/`, `common/ctrl_events.h` | §7 |
| Motor speed control (PI) with stall and encoder faults | `buddy2_motion/motion_task.c` | §2 |
| Minimum-bit data (Change Requirement 1) | `common/faults.h` (one byte), `lib/libnet/lwip/telemetry_codec.c` (88 bits) | §10 |
| Barr-C coding style | all of `app_program/` | §9 |
| Memory and CPU use | `common/cpu_meter.c` | §11 |

## Test cases

### On the car

Each UF2 holds one test mode, picked by `MOTION_TEST_MODE` in
`app_program/test_modes/test_modes.h`. Test builds cited by the evidence
log are in `tests/full_auto/` (`ctrl_course_vNN`, the latest is v55) and
`tests/ir_test/`.

| Mode | Name | What it tests |
|---|---|---|
| 31 (default) | `CTRL_TEST_COURSE` | The course runs, by key: `l` line and ramp, `o` obstacle stop and width, `a` go round and rejoin, `b` barcode command at the cross, `r` barcode read only, `f` full course, `c` full course with commands off, `m` type the barcode mapping. Each run prints its event log, status, timing and CPU use. |
| 32 | `BARCODE_TEST_DRIVE` | Driven barcode scan at 50 mm/s and the decode |
| 20 | `MOTION_TEST_SAFETY` | Car lifted: loop rate, PI error, STOP over queued commands, stall, encoder check, no false trips |
| 21 | `MOTION_TEST_CPU` | Motion task CPU use and stack |
| 5 | `MOTION_TEST_PI_STEP` | Speed control step response |
| 3 | `MOTION_TEST_HAND` | Encoder counts with the wheels turned by hand |
| 18 | `IR_TEST_LOG` | Live IR readout and black/white widths |

### On the PC (no car needed)

`mtk3smp-rp2040/tests/host/` holds unit tests of the modules that do not
need the hardware. Run them all with gcc (Linux or WSL):

```bash
bash mtk3smp-rp2040/tests/host/run_host_tests.sh
```

| Test | Module | Checks |
|---|---|---|
| `telemetry_codec_test.c` (in `lib/libnet/lwip/`) | telemetry packet | Round trip at every field's limit, negative speeds, out-of-range values rejected |
| `cmd_map_test.c` | barcode mapping from MQTT | Packing (A Z B C = `ca b8 30`), lookup, bad and duplicate payloads rejected, the old mapping kept |
| `barcode_decode_test.c` | Code 39 decoder | `A` and `Z` both ways, two scales, one bar short at either end, both ends cut rejected, random patterns |
| `terrain_lift_test.c` | lift check | A bump then a cross, the ramp and its landing, and an old cue do not fire; level, tilted, upside-down and dropped lifts do |
