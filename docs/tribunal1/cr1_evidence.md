# Evidence log: final implementation

INF2004 Group 27 robot car on micro T-Kernel 3.0 (RP2040, Robo Pico).
This log justifies the final implementation of each subsystem with test
evidence. Per the professor's note (29 Oct), it does not compare the
original design with the revised one; earlier steps appear only where a
design choice needs its reason.

- **Code:** `mtk3smp-rp2040/app_program`, branch `thouyong`, which matches
  `main` as of 27 Oct. Commits are quoted for each result.
- **Builds:** the course test (mode 31, `CTRL_TEST_COURSE`) is the
  default. The test UF2s are in `tests/full_auto/` (`ctrl_course_vNN`)
  and `tests/ir_test/` (`barcode_drive_vNN`).
- **Course test keys:**
  - `l`: line and ramp
  - `o`: obstacle stop and measurement
  - `a`: obstacle measure, go round and rejoin
  - `b`: barcode, then the command at the junction
  - `r`: barcode read only
  - `c`: continuous course, everything on with no distance limit
- **Test space:** two 30.5 cm mats (straight and cross), 100 mm/s,
  450 mm per run, unless stated otherwise.

## 1. Summary

| Subsystem | Final design | Key evidence | Status |
|---|---|---|---|
| Motion (Buddy 2) | PI speed control on hardware edge counters; 2 ms loop; right encoder cross-checked against its B channel; stall and encoder faults | PI within 2 % at 150/300/500 mm/s; STOP in 350–386 ms; faults trip and no false trips | Done |
| Line following (Buddy 3) | right-edge following by heading; junction = both IRs black for 6–35 mm, from either side | 3/3 bare-mat runs and 2/2 more on v29; cross found in every run | Done |
| Barcode (Buddy 3) | Code 39, ratio decoding at any 1:3 scale; A–Z command table; gentle following while reading | 5/5 decoded on v29 (A4 'A' ×3, 50% 'A', 50% 'Z'); commands at the cross 8/8 correct | Done (final letter mapping awaited) |
| Ramp (Buddy 4) | height from roll over a 30 mm plateau, h = 114 mm × sin(roll); straight hold on the ramp | small ramp (10 mm) read 11–12 mm; tall ramp (24 mm) read 23–26 mm | Done |
| Obstacle (Buddy 5) | stop at 300 mm, creep to 200 mm, side-step to both edges, width = left − right; go round on the left and rejoin | every stop ≥ 5 cm (about 13 cm after the creep); width ±3 cm typical over 13 runs; 4/4 clean drive-pasts | Done |
| Controller | event-driven task; ramp, barcode and obstacle sequences; continuous course mode; rolling log and scenario reports | each sequence tested on its own; continuous mode built, not yet run | First lab run due |

## 2. Motion (Buddy 2)

### Design

- **Motors and encoders:** TB6612FNG motors on PWM slices 4/5. Encoder
  edges are counted by PWM-slice hardware: left on GP19, right A on GP15,
  right B on GP17 (slice 0). 0.328 mm per edge, 114 mm track.
- **Control loop:** the motion task runs every 2 ms and the PI speed loop
  every 20 ms. The task keeps the odometry, under a lock shared with
  readers.
- **Safety:** STOP is checked every pass and discards queued commands. A
  stall or one-encoder fault raises a bit in the shared fault byte
  (`common/faults.h`) and stops the wheels. The checks are masked while
  pulling away and while on the ramp. A ramp mask held for more than 4 s
  while driving raises `FAULT_MOTOR_STALL`.
- **Right encoder cross-check (27 Oct):** after the car had been off for
  1–2 h, the right A channel counted extra edges. On every pass the right
  count now follows A but never exceeds B + 1 edge, and the PI uses the
  corrected count. The pads are set with a Schmitt trigger and pull-up.

### Evidence

| Test | Result |
|---|---|
| Lifted, `MOTION_TEST_SAFETY` (21 and 27 Oct) | PI at 150/300/500 mm/s within 2 % (500: L 498, R 498); STOP with 3 commands queued: wheels still in 350–386 ms, queue discarded; both wheels held → `MOTOR_STALL`; left held → `ENC_LEFT`; no false trip in 10 s |
| Hand turn, 10 turns, fault active (27 Oct) | A raw 6587, B 6365, corrected right count 6366 (expected about 6340) |
| `MOTION_TEST_CPU` (21 Oct) | 0.33 % CPU while driving (about 6 µs per pass, longest 13 µs); stack high-water 296 of 4096 B |
| Encoder cross-check during obstacle side-steps (28 Oct) | A − B over each probe 0 to ±3 edges |

### The loop period is 2 ms, not 1 ms

The early design quoted a 1 ms loop (995 polls/s). That count actually
spanned a 1 s run plus a 1 s settle. `tk_dly_tsk(1)` also waits one extra
kernel tick (`knl_timer_insert_reltim()` adds `TIMER_PERIOD`), so the loop
runs at 500 passes/s. Two independent measurements confirmed this
(`motion_loop_rate()` and the 1 MHz timer in `MOTION_TEST_CPU`).

The 2 ms loop is kept because:
- the PI runs every 20 ms either way;
- edges are counted in hardware, so no edge is missed;
- STOP latency is one pass (2 ms) against about 380 ms of braking;
- a true 1 ms loop would double the task's CPU for no gain.

The pass mark for the all-tasks test is 495 passes/s or more.

## 3. Line following and junctions (Buddy 3)

### Design

- **IRs:** centre GP4, right GP26 (20 mm right of the centre IR), left
  GP6 (digital) and GP27 (analog, for the barcode).
- **Right-edge following by heading** (v26, 29 Oct). The centre IR rides
  the line's right edge, and each reading picks a target heading beside
  the line's direction (`line_dir`, the encoder heading averaged over
  100 mm):
  - centre black: aim 2° right of the line's direction;
  - centre white: aim 2° left;
  - the aim grows by 0.08° per 20 ms while the car stays on the same side
    (for curves), up to 45°.

  The wheel speed difference is the heading error / 15 (mm/s), capped at
  60 and slew-limited.
- **Right IR alone black:** the car has overshot left, so it turns hard
  right. This lasts only while the right IR's black stretch is arm-sized
  (≤ 35 mm), and only until the car points more than 5° right of the
  line's direction. A black ramp beside the line can therefore no longer
  pull the car onto it.
- **Lost line:** the centre IR white for 100 mm triggers an on-spot search
  within ±60° of the line's direction. A search counts as successful only
  when the centre IR sees black.
- **Junctions:**
  - Right side: the centre and right IR are both black for ≥ 6 mm, and
    the right IR's black stretch ends within 35 mm. A black ramp keeps the
    right IR black for 100 mm or more, so it is not taken as a junction.
  - Left side: the barcode reader sees one 10–35 mm bar with no other read
    in the 30 mm before it. The report is shifted by the left IR's 20 mm
    lead.
  - The first report wins, and a second report within 60 mm is the same
    junction. With no command pending, the junction is crossed straight.

### Why heading steering (the decision)

The earlier steering set the wheel speed difference directly from the IR
reading. It started at 20 mm/s and grew by 2 per period on one side.

- With the lost-line limit at 40 mm, the v23 traces showed a stop and
  search on nearly every swing: the car only turned back after about
  40 mm of white.
- Raising the limit to 120 mm (v25) let each swing grow (−23°, then +13°
  to +20°) until the car left the mat.

With heading steering, the correction reverses the moment the IR changes
side.

| Build | Bare-mat `l` runs | Weave | On-spot searches |
|---|---|---|---|
| v23 (old steering, lost at 40 mm) | 4 | about ±15°, with a stop at every swing | 4–5 per run |
| v25 (old steering, lost at 120 mm) | 3 | grew to ±20°; 0/3 finished (left the mat) | — |
| v26 (heading steering) | 3/3 finished | about ±8° | 1 per run, at a known weak spot on the test mat |
| v29 (smaller aim) | 2/2 finished | about ±5° | 1–2 per run (limit 60 mm, raised to 100 mm in v30) |

The cross was detected and crossed straight in every v26/v29 run
(271–309 mm).

**Sensor setting check:** on a plain straight, the centre and right IR
must never read black together. If they do, the trimmer is too sensitive,
and the car takes the line itself for a junction arm. Check by sliding the
car slowly across the line (29 Oct).

## 4. Barcode (Buddy 3)

### Design

- **Sensor:** the left IR, with its analog output (GP27) slit-taped
  (about 1 mm slit) at about 9 mm above the mat. It now sits 65 mm left of
  the car's centreline (bracket moved on 29 Oct). The bars lie 42–203 mm
  left of the centre IR: 18 mm of line, a 24 mm gap, then bars 161 mm long
  at every scale. Before the move, the IR sat only about 8 mm inside the
  bars.
- **Sampling:** one sample per encoder step (0.33 mm), measured at the
  left IR, which corrects for the car's yaw. The white level is tracked.
  A bar starts 60 mV above white.
- **When a read ends:** after 4× the narrowest bar of white, at least
  22 mm and at most 30 mm. That is more than the widest space of any
  expected scale (A4: 18 mm). The buffer holds about 360 mm, so a
  3-character A4 code (282 mm) fits.
- **Decoding:** Code 39, read in both directions. Wide and narrow are told
  apart by ratio (wide ≥ 1.8× narrowest), so any 1:3 scale decodes.
- **Reading mode:** from the first bar until the read ends, the car keeps
  following the edge with a fixed ±1° aim. Its heading then stays within
  about ±2°, so the left IR crosses the bars on a steady path.
- **Commands:** a 26-letter table, forward and reversed. A = left,
  A-reversed = right, Z = straight, Z-reversed = U-turn; the other letters
  carry no command until the final mapping is given. The command is
  carried out at the next junction, turning on the spot from the line's
  direction.

### Evidence

| Test | Result |
|---|---|
| Driven scans, 50% sample (`barcode_drive`, 24 Oct) | 10/10 A / A-reversed, 6/6 Z / Z-reversed; no wrong decode |
| Command at the cross (v4–v7, 26 Oct) | 8/9 read; every read and command correct; turns 87.4–91.2° (from the line's direction); U-turn 180.0° |
| Command at the cross (v16, 28 Oct) | every `b` run decoded and turned correctly; junctions from both sides |
| A4 'A', blind straight hold (v27) | 3/4; the miss cut the code at its first wide space (fixed: at least 22 mm of white) |
| **v29, reading mode and the moved IR** | **5/5**: A4 'A' ×3 forward, 50% 'A', 50% 'Z'; barcode placed near and far from the start |

**Why reading mode (the decision):** the left IR is about 130 mm ahead of
the axle. A ±8° weave therefore swings it about ±18 mm sideways, and A4
reads broke up, with "spaces" of 19–53 mm. A blind straight hold fixed the
heading but kept whatever sideways offset the car had at the first bar.
Gentle following keeps both the heading and the sideways position.

## 5. Ramp height (Buddy 4)

### Design

- **What is measured:** the LSM303D (I2C1, GP2/3) is sampled by the
  controller every 20 ms. With one wheel on the ramp, the car rolls rather
  than pitches.
- **Detection:** a ramp starts when the smoothed roll stays past 2.5° for
  20 mm, and ends within 1.5° for 15 mm.
- **Height:** h = 114 mm × sin(roll), using the most tilted 30 mm mean.
  The pivot is the left-wheel-to-castor line, and the formula holds for
  the measured geometry. The lowest height it can detect is about 5 mm
  (flat-floor noise 1.8–2.5°).
- **On the ramp:**
  - the line task drives straight along the line's direction (the
    tilted IR reads false black);
  - the motion faults are masked;
  - an on-spot search stops at once;
  - 50 mm after the ramp, following resumes, with a search within ±45° if
    needed.
- **Report:** the highest ramp of a run is reported (`max_ramp_mm`).
- **Impacts:** logged only (user decision, 26 Oct). A jolt over 500 mg
  counts only if the wheels then drop below 60 % within 250 ms, at a
  steady speed and off the ramp. IR-bracket strikes on the ramp (up to
  1659 mg) were all correctly ignored.
- **Compass:** not used for heading. On this floor it reported a 48° turn
  over 50 cm while the encoders showed 1.5°.

### Evidence

| Ramp | Result |
|---|---|
| Small practice ramp, 10 mm (the final demo height) | 11–12 mm in every clean run (v11–v16, 27–28 Oct) |
| Tall ramp, 24 mm | 23–26 mm, 6/6 (21 Oct); 25–26 mm with the skid (26 Oct) |
| Flat floor | no ramp detected |

The professor (27 Oct) set the final ramp at about the small practice
ramp's height, and the tall ramp can be disregarded. The small ramp clears
without the skid.

## 6. Obstacle (Buddy 5)

### Design

1. **Detect and stop:** while following, the ultrasonic sensor (GP7/28,
   about 60 mm behind the nose) pings every 65 ms. The car stops when 2 of
   the last 3 pings read ≤ 300 mm. Speed is capped at 300 mm/s while
   watching.
2. **Square up and creep in:** the car turns to the line's direction, then
   creeps at 40 mm/s until the sensor reads 200 mm (at most 150 mm of
   creep, so the nose gap stays ≥ 8 cm even if every ping is lost).
3. **Measure:** the car side-steps in 25 mm steps (turn 90°, drive, turn
   back, median of 3 pings).
   - Right side first, until 2 readings in a row are off the face.
   - A reading counts as the face if it is no more than 25 mm beyond the
     median of the face readings so far; a nearer reading always counts.
   - Then back to the start in one move, and the same to the left.
   - Each edge is the midpoint between the last on-face step and the first
     off-face step, less 17 mm (calibrated).
   - Width = left edge − right edge, measured on the wheels. The range
     stays about 190 mm, clear of the floor echo at about 290 mm.
4. **Go round** (on the left):
   - side-step until the car's right side clears the left edge by 40 mm
     (planning for at least 100 mm of width);
   - drive past the face, an assumed 200 mm depth, and the car's 90 mm
     left half-width plus the margin;
   - side-step back to the line's position and carry on following.

### Why side-stepping (the decision)

- **Turning scan rejected.** A flat face echoes only when the sonar points
  at it squarely, and the sonar barely moves sideways as the car turns on
  the spot. So the angle over which a face was seen did not grow with
  width:

  | Face | Angle seen |
  |---|---|
  | 9 cm | 47–52° |
  | 15 cm | 50–56° |
  | 23 cm | 49–57° |

  (v17, 28 Oct)
- **Side-stepping measures the edges geometrically.** The echo disappears
  when the sonar passes the edge.
- **First calibration (v18):** the raw edges read about 17 mm wide per
  edge, because the beam still clips the corner. Over 9 runs: 9 cm 128 ×3,
  15 cm 178/205/153, 23 cm 254/257/281 mm.

### Evidence (v20, 13 runs, 28 Oct)

| Face | Width read (mm) | Error |
|---|---|---|
| 9 cm | 71, 93, 95, 122, 95 | −19 to +32 mm |
| 15 cm | 120, 120, 147 | −30 to −3 mm |
| 23 cm (rounded corners) | 226, 249, 225, 272, 301 | −5 to +71 mm |

- **Accuracy:** mean error +7 mm, typically ±3 cm. The worst under-read
  is −3 cm, which the 40 mm margin covers. Over-reads (worst +7 cm, where
  the corner echo ran on along the side) only cost clearance.
- **Stops:** every stop met the ≥ 5 cm nose gap. The trigger read
  279–300 mm, and the creep ended at 191–198 mm (nose gap about 13 cm).
- **Drive past (`a` runs, v20):** 4/4 cleared the box with no contact.
  The car ended 6–7 mm from the line's position.
- **Time:** measuring took about 43 s (9 cm), 51 s (15 cm) and 71 s
  (23 cm); the drive past took 17–18 s.
- **Stop at full watching speed** (`OBST_TEST_STOP`, 300 mm/s, 22 Oct):
  the car braked in 62–67 mm and stopped 12.7–15.0 cm from a flat box and
  9.8–10.6 cm from a round bottle. 7/7 runs were ≥ 5 cm.

## 7. Vehicle controller and continuous course

### Design

- **Task and events:** the controller (priority 6, below motion) is the
  only task that decides what the car does. The buddy tasks post 4-byte
  events to `ctrl_evt_mbf`, which never blocks: obstacle, line end,
  barcode, junction, and every new fault bit.
- **Each 20 ms the controller:**
  - samples the IMU and runs the ramp detector;
  - owns the ramp hold and the hump mask;
  - keeps the pending barcode command for the next junction;
  - runs the obstacle sequence.
- **Continuous course mode** (`c`, v22): no distance limit. After a
  junction turn, or once round an obstacle, line following restarts. A
  junction with no command is crossed straight. The run ends only on a
  stop request (later Buddy 1), a stopping fault, or a failed line search.
- **Status:** `robot_status` (18 B) is copied under a lock for telemetry
  every 500 ms: state, faults, last event, flags, distances, heading,
  roll, sonar, width, highest ramp and total distance in cm.
- **Rolling log:** the event log keeps the latest 64 events, with 32-bit
  time and run distance.
- **One-off scenario reports:**
  - ramp: `RAMP_OFF` (height);
  - barcode: `BARCODE` then `COMMAND` (letter, direction, turn);
  - obstacle: `OBSTACLE`, `OBST_CREEP`, `OBST_EDGE` ×2, `OBST_WIDTH`, then
    `OBST_DONE` (result, time).

### Evidence

- Each sequence on its own: see sections 3–6. No events were dropped in
  any logged run.
- **Continuous course:** built and pushed (v22, then v30). It needs the
  lab circuit and has not been run yet.
- **Cycle time estimate** for one ramp, one barcode and a ~20 cm obstacle
  on 9 mats:
  - normally about 2 min: 31 s of following, 5–6.5 s of turn, about 85 s
    for the obstacle;
  - worst case about 3 min.

  The demo allows 10 min.

## 8. Code structure and Barr-C (28 Oct, commits ab86df2, b510d3b)

- **Structure:**
  - `app_main.c` is start-up only (63 lines, from 5,273): the health LED,
    the Wi-Fi status task on a Wi-Fi build, the motion task, then the
    selected test mode.
  - The 9 test modes in use are in `test_modes/`, with their numbers
    unchanged.
  - The port's demo pipeline and 24 retired modes were removed; they
    remain in git history.
- **Barr-C fixes:**
  - every line ≤ 80 columns;
  - Boolean variables `b_` (globals `g_b_`), file-scope statics `g_`;
  - `/* */` comments and `u` suffixes in `pin_defs.h`.
- **Remaining kernel types:** `INT`/`UINT`/`UB` stay only where
  µT-Kernel's own signatures need them.
- **Verification:** the course build compiled to a byte-identical binary
  before and after the Barr-C pass. All test modes build with
  `-Wall -Wextra` and no warnings.

## 9. Change Requirement 1: minimum-bit data (to do, Week 13)

Required: every shared or sent field uses only the bits its range needs.
This covers:
- faults as one bitmask (already 7 bits in one byte);
- `robot_status`, the events, the task/ISR data and the MQTT payload
  (Buddy 1);
- safe read-modify-write on packed shared data (`common/critical.h`).

Loop-internal arithmetic stays at full width.

Reference figures, measured before any packing, are kept so the packing
can be shown. The professor's 29 Oct note drops only the original-vs-
revised design comparison.

| Item | Bytes now |
|---|---|
| `robot_status_t` | 18 |
| `ctrl_evt_t` (event message) | 4 |
| `ctrl_log_t` (log entry, test) | 12 |
| `motor_cmd_t` (motion command) | 6 |
| Firmware, course build v29 (text / data / bss) | 74702 / 44 / 31128 |

Big bss items that are test-only and will be compiled out of the final
build:
- the PI trace (4200 B);
- the line trace (5.6 kB);
- the status snapshots in the test mode.

## 10. Known limits and open items

- **Obstacle width:**
  - About ±3 cm on flat faces. Rounded corners over-read, which is safe.
  - The depth is assumed to be 200 mm.
  - Only the left side is used to go round.
- **Line following:** tested on straights and the cross; curves on the
  lab circuit are still to be tested.
- **Barcode:**
  - the final letter mapping is awaited (the table is ready);
  - the IR height needs about 9 mm ± 0.5 mm, because the bars blur at
    10 mm.
- **Continuous course:** first lab run pending.
- **CR1 packing and Buddy 1 telemetry:** to do.
