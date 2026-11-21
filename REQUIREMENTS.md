# Requirements and how to verify them

INF2004 Group 27 (CSC27) robot car on micro T-Kernel 3.0 (Raspberry Pi
Pico W on a Robo Pico board). These are the final requirements from the
Tribunal 2 pre-tribunal submission (A1), for firmware **v71**. Each one is
mapped to the code that meets it, the test that checks it and the measured
result.

- **Code:** `mtk3smp-rp2040/app_program/` (one folder per buddy), with
  Buddy 1's MQTT client in `mtk3smp-rp2040/lib/libnet/lwip/`. Paths below
  are under `app_program/` unless they start with `lib/` or `tools/`.
- **Results:** [`docs/tribunal1/cr1_evidence.md`](docs/tribunal1/cr1_evidence.md),
  section 13 has the table of every test below (18–19 Nov) and section 14
  the full-circuit laps and fixes of 21 Nov (v63–v71); earlier sections
  have the design reasons and older runs.
- **Build and flash:** [`README_QUICKSTART.md`](README_QUICKSTART.md).
- **Code overview** (tasks, priorities, folders, pins):
  [`mtk3smp-rp2040/app_program/README.md`](mtk3smp-rp2040/app_program/README.md).

The course runs are started by key on the serial console or from the
laptop control page (`tools/mqtt_viewer.py`): `l` line and ramp, `o`
obstacle stop and width, `a` go round the obstacle, `b` barcode command at
the cross, `r` barcode read only, `f` full course (MQTT START), `c` full
course with commands off, `m` type the barcode mapping.

## Functional requirements

| ID | Requirement | Code | Test | Result |
|---|---|---|---|---|
| FR1 | Follow the black line course (straights, curves and cross junctions) with the three IR sensors; search for the line and rejoin it after losing it for up to 100 mm of travel. | `buddy3_line_barcode/line.c` | `l`, `c`, `f` runs | 3 laps of the full circuit, 30.3 m (v67) and 2 laps, 22.9 m (v69), curves, S-bend and crosses, no fault (21 Nov) |
| FR2 | Detect a Code 39 barcode left of the line with the left IR's analogue output, sampled once per encoder edge (0.328 mm). | `buddy3_line_barcode/barcode_scan.c` | `r`, `b` runs | codes detected on the circuit and the mats |
| FR3 | Decode the character in either direction, at any bar size with a 1:3 ratio (narrow bar 3 to 6 mm). | `buddy3_line_barcode/barcode.c` | `r` runs; host test `barcode_decode_test.c` | `A`, `Z`, `S`, `F`, `Q` read forward and reversed (50% and full A4 codes) |
| FR4 | At the next cross after a barcode, perform the movement mapped to the character (forward, U-turn, left, right). The mapping is sent from the PC over MQTT and acknowledged, not hard-coded. | `controller/vehicle_ctrl.c` (`junction_turn`), `buddy1_telemetry/cmd_map.c` | `b` runs with the mapping from the control page; host test `cmd_map_test.c` | U-turn 178.7–179.7°, forward crosses straight, left 87.9–89.4°, right 89.2–90.2°; line picked up after every turn on the laps; mapping kept across a car restart |
| FR5 | Detect a hump with the IMU and report the peak height in mm in telemetry. | `buddy4_imu/terrain.c` | `l` run over the ramp; laps | 11 mm ramp read 13 mm; 10–12 mm on the laps |
| FR6 | Detect an obstacle on the line with the ultrasonic sensor and stop before touching it. | `buddy5_ultrasonic/obstacle.c` | `o` runs; dead sonar (ECHO unplugged) | stops with a 5–7 cm nose gap; a dead sonar ends the run before it moves |
| FR7 | Estimate the obstacle's width and record it in the event log. | `controller/vehicle_ctrl.c` (`probe_obstacle`) | `o`, `a` runs | see NFR4 |
| FR8 | Drive round the obstacle without touching it, rejoin the line behind it and carry on. | `controller/vehicle_ctrl.c` (`avoid_obstacle`, `avoid_return`) | `a`, `f` runs | went round, line met 9 mm from plan, turns within 0.34°; on the laps every avoid rejoined within 22 mm of plan, including boxes one mat after a curve (v69 face search, 3 runs); a box only 20 cm after a curve was clipped by the tail (closer than the course's one mat between scenarios) |
| FR9 | Publish telemetry over MQTT (state, motion, wheel speeds, distance, barcode command, hump height, obstacle distance, faults, heartbeat) and accept START, STOP and run selection from the PC. | `lib/libnet/lwip/lwip_utk_mqtt.c`, `lib/libnet/lwip/telemetry_codec.c`, `buddy1_telemetry/tel_source.c` | Wi-Fi build with the control page; host test `telemetry_codec_test.c` | all buttons work; run buttons return after a run; hotspot loss and car restart recover |
| FR10 | Stop the motors and end the run when picked up; report LIFTED. | `buddy4_imu/terrain.c` (`terrain_lift_update`), `controller/vehicle_ctrl.c` (`lift_check`) | lift the car during `l`, `a` runs; host test `terrain_lift_test.c` | see NFR8 |
| FR11 | Clear the whole course, scenarios in any order, in one run started by one START; keep lapping until STOP, a lift or a stopping fault. | `controller/vehicle_ctrl.c` | `f` run on the circuit | 3 laps, 30.3 m (v67); 2 laps, 22.9 m (v69); every scenario each lap, one START, ended by STOP (21 Nov) |

## Non-functional requirements

| ID | Requirement | Code | Test | Result |
|---|---|---|---|---|
| NFR1 | Motion loop every 2 ms (at least 495 passes/s), PI every 20 ms; each wheel holds 150, 300, 500 mm/s within ±2%. | `buddy2_motion/motion_task.c` | mode 20 part A, mode 5; TIMING line of every course run | 500 passes/s; step test 148/148, 303/299, 506/500 mm/s. A few Wi-Fi course runs read 470/s for one second: the radio's status lines (printed with interrupts masked) held the loop off; from v71 they are held until the run ends, and a lap with the hotspot off and on mid-run read at least 496/s |
| NFR2 | STOP from MQTT or the console reaches the motors within 2 ms in any state (including a turn on the spot and the width measurement) and discards queued moves; at rest within 400 ms from 300 mm/s. | `buddy2_motion/motion_task.c` (STOP checked first every 2 ms pass), `controller/vehicle_ctrl.c` (`seq_abort`) | mode 20 part B; STOP during `a`, `b` runs; START then STOP | at rest in 368 ms; stops mid-turn, mid-measurement and during the face search (v69); a STOP right after START never moves |
| NFR3 | While watching for obstacles: at most 300 mm/s, ping every 65 ms, stop when 2 of the last 3 readings are 300 mm or less, nose gap at least 5 cm. | `buddy5_ultrasonic/obstacle.c` | `o`, `a` runs | nose gap 5–7 cm at every stop |
| NFR4 | Obstacle width within ±3 cm in typical runs. | `controller/vehicle_ctrl.c` (`probe_obstacle`) | `a` runs on measured boxes | 9.5 cm box: 95–96 mm; 15 cm box: 145 mm; 9 cm box after the ramp: 93 mm (v65); one mat after a curve (v69): 15 cm face 142 mm, 9.5 cm face 71 and 96 mm |
| NFR5 | IMU read every 20 ms; hump height within ±3 mm; humps below about 5 mm not reported. | `buddy4_imu/terrain.c` | `l` runs over a measured ramp | 11 mm ramp: 13 mm; on the laps 10–13 mm; a flat-topped 11.5 mm ramp read 11 mm |
| NFR6 | Barcodes decode with the left IR 9 ± 0.5 mm above the floor; junction turns 90° ± 3°, U-turn 180°. | `buddy3_line_barcode/barcode_scan.c`, `controller/vehicle_ctrl.c` | `b` runs; IR height by ruler | IR at 9 mm; turns 89.2–90.2°, U-turns 178.7–179.7° |
| NFR7 | 11-byte (88-bit) telemetry every 500 ms; a lost link raises FAULT_WIFI_LOSS within 3 s, retries every 5 s and never stops or slows the car; the last will marks the car OFFLINE. | `lib/libnet/lwip/lwip_utk_mqtt.c`, `lib/libnet/lwip/telemetry_codec.c` | hotspot off mid-run; car power-off with the control page open | fault raised, car drove on, reconnected; OFFLINE after 15 s; v71 lap with the hotspot off for ~10 s: FAULT_WIFI_LOSS, car drove the whole lap, loop at least 496/s |
| NFR8 | Lift-to-stop: motors stop when all three IRs see no floor for 1 s, or for 0.5 s after an IMU cue (tilt over 25° for 0.1 s, or acceleration more than 300 mg off 1 g); no trigger on a cross or a ramp. | `buddy4_imu/terrain.c` | lifts during `l` and `a` runs; host test `terrain_lift_test.c` | quick lift 0.5 s, slow lift 1.0 s, lift during an obstacle side-step turn 1.2 s; no false stop in any course run, including runs over a black ramp under the right wheel (21 Nov) |
| NFR9 | All tasks under 50% CPU; fixed RAM under half of 264 KB. | `common/cpu_meter.c` | CPU line of every course run; build size | about 14–18.5% busy (v67 laps: each task 0.3–8.8%); 31.7 KB (66.7 KB with Wi-Fi, v58) |
| NFR10 | One lap under 5 minutes. | — | `f` run on the circuit | about 10.1 m in 2.6 min a lap (30.3 m in 470 s, v67), 75–80 s of it at the obstacle |
| NFR11 | C on micro T-Kernel 3.0, Barr-C; only the motion task writes the motor PWM; tasks share data only through message buffers, event flags, a locked status copy, a critical-section fault byte and a few single volatile words; host unit tests pass. | all of `app_program/`; `mtk3smp-rp2040/tests/host/` | build with `-Wall -Wextra` (0 warnings); `run_host_tests.sh` | all 7 test modes and the Wi-Fi build: 0 warnings; host tests pass |

### Demo rubric cross-reference

| Rubric point | Requirements |
|---|---|
| Detect a barcode | FR2 |
| Decode the barcode and obtain the code | FR3 |
| Measure hump height | FR5, NFR5 |
| Follow the line | FR1 |
| Follow the instruction from the barcode | FR4, NFR6 |
| Detect an obstacle | FR6, NFR3 |
| Measure obstacle width | FR7, NFR4 |
| Navigate around the obstacle and rejoin the line | FR8 |
| Send telemetry data via MQTT | FR9, NFR7 |
| Stop motors when the car is picked up | FR10, NFR8 |

## Test cases

### On the car

Each UF2 holds one test mode, picked by `MOTION_TEST_MODE` in
`app_program/test_modes/test_modes.h`. The course build used for the
Tribunal 2 tests is `tests/full_auto/ctrl_course_v62_smp0.uf2` and the final
build `ctrl_course_v71_smp0.uf2` (line following and obstacle squaring
fixes from the full-circuit laps, v63–v69; tidy-up v70; radio prints held
during runs v71); the motion test builds are
`motion_safety_v60_smp0.uf2` (mode 20) and `motion_pi_step_v60_smp0.uf2`
(mode 5) in the same folder (their motion code is the same as v71). Older builds cited by the evidence log are
there and in `tests/ir_test/`. The Wi-Fi build needs the local
`config/wifi_credentials.h` and `mqtt_config.h` (see `README_QUICKSTART.md`).

| Mode | Name | What it tests |
|---|---|---|
| 31 (default) | `CTRL_TEST_COURSE` | The course runs by key (above). Each run prints its event log, status every 500 ms, the line trace, timing, CPU use and the lift-check margins. |
| 32 | `BARCODE_TEST_DRIVE` | Driven barcode scan at 50 mm/s and the decode |
| 20 | `MOTION_TEST_SAFETY` | Car lifted: loop rate and speed error, STOP over queued commands, stall, encoder check, no false trips |
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
| `telemetry_codec_test.c` (in `lib/libnet/lwip/`) | telemetry packet | Round trip at every field's limit, negative speeds, out-of-range values (speed, motion) rejected |
| `cmd_map_test.c` | barcode mapping from MQTT | Packing (A Z B C = `ca b8 30`), lookup, bad and duplicate payloads rejected, the old mapping kept |
| `barcode_decode_test.c` | Code 39 decoder | `A` and `Z` both ways, two scales, one bar short at either end, both ends cut rejected, random patterns |
| `terrain_lift_test.c` | lift check | A bump then a cross, a light ramp, a cue from normal driving then 0.4 s of cross, and a stale cue do not fire; quick, tilted, slow level, "held as a ramp", upside-down and dropped lifts do |
