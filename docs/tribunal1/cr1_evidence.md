# Evidence log: final implementation

INF2004 Group 27 robot car on micro T-Kernel 3.0 (RP2040, Robo Pico).
This log justifies the final implementation of each subsystem with test
evidence. Per the professor's note (29 Oct), it does not compare the
original design with the revised one; earlier steps appear only where a
design choice needs its reason.

- **Code:** `mtk3smp-rp2040/app_program`, branch `thouyong`; the final build
  is v82 (commit d11dd12, 24 Nov); sections 13–15 cover v57–v82. Commits
  are quoted for each result.
- **Builds:** the course test (mode 31, `CTRL_TEST_COURSE`) is the
  default. The test UF2s are in `tests/full_auto/` (`ctrl_course_vNN`)
  and `tests/ir_test/` (`barcode_drive_vNN`).
- **Course test keys:**
  - `l`: line and ramp
  - `o`: obstacle stop and measurement
  - `a`: obstacle measure, go round and rejoin
  - `b`: barcode, then the command at the junction
  - `r`: barcode read only
  - `c`: continuous course, everything on with no distance limit,
    barcode commands off (lapping test)
  - `f`: full course, as `c` with barcode commands carried out (added
    4 Nov; MQTT START now starts `f`)
  - `m`: type the barcode mapping (forward, U-turn, left, right) by
    hand; from v54 (5 Nov) the mapping is not in the firmware and the PC
    normally sends it over MQTT
- **Test space:** two 30.5 cm mats (straight and cross), 100 mm/s,
  450 mm per run, unless stated otherwise. From v33 (30 Oct) the user's
  lab circuit: a loop of joined mats with a 270° curve (radius about
  14 cm), an S-bend, crosses, a ramp, boxes and barcodes, run with `c`
  laps in both directions.

## 1. Summary

| Subsystem | Final design | Key evidence | Status |
|---|---|---|---|
| Motion (Buddy 2) | PI speed control on hardware edge counters; 2 ms loop; right encoder cross-checked against its B channel; stall and encoder faults | PI within 2 % at 150/300/500 mm/s; STOP in 350–386 ms; faults trip and no false trips | Done |
| Line following (Buddy 3) | right-edge following by heading; curve mode for bends; junction = both IRs black for 6–35 mm, from either side | 3/3 bare-mat runs and 2/2 more on v29; lab circuit lapped 13 m each way (v45), clockwise laps consistent (v51) | Done |
| Barcode (Buddy 3) | Code 39, ratio decoding at any 1:3 scale, one bar short accepted at either end; mapping sent from the PC over MQTT; gentle following while reading | 5/5 decoded on v29; commands at the cross 8/8 correct; on the circuit the A4 code read every lap, the 50% `A` after a curve 2/3 laps (v49) | Done |
| Ramp (Buddy 4) | height from roll over a 30 mm plateau, h = 114 mm × sin(roll); straight hold on the ramp | small ramp (10 mm) read 11–12 mm; tall ramp (24 mm) read 23–26 mm | Done |
| Lift (Buddy 4) | all three IRs see no floor for 1 s, or 0.5 s after an IMU tilt or acceleration cue; motors stop, run ends LIFTED | v58: quick lift stopped in ~0.3 s (acceleration cue), slow lift in 1.0 s (no floor); light ramp never read as no floor; host tests pass | Done (black ramp untested) |
| Obstacle (Buddy 5) | stop at 300 mm, creep to 200 mm, side-step to both edges, width = left − right; go round on the left with depth = width, return until the line is seen | every stop ≥ 5 cm (about 13 cm after the creep); width ±3 cm typical over 13 runs; rounded 9.5 cm box 95/96 mm, line met within 5–7 mm (v49) | Done |
| Controller | event-driven task; ramp, barcode and obstacle sequences; continuous course (`c`, `f`); rolling log and scenario reports; motion state | each sequence on its own; continuous laps on the lab circuit (v45–v52); full course with commands, 13.3 m (v57) | Done |
| Telemetry (Buddy 1) | MQTT over Wi-Fi: 11-byte bit-packed status every 500 ms, events, last will, run commands, barcode mapping from the PC | codec tests pass; on the car 29 Oct (status, events, reconnect, last will); mapping round trip with a simulated car (v54) | Done (v54 mapping to confirm on the car) |

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

  Each wheel is corrected by the heading error / 15 (mm/s, so the wheels
  differ by twice that), capped at 60 and slew-limited.
- **Right IR alone black:** the car has overshot left, so it turns hard
  right. This lasts only while the right IR's black stretch is arm-sized
  (≤ 35 mm), so a black ramp beside the line can no longer pull the car
  onto it. The 5° cap (`LINE_R_DEV_MAX_CDEG`) only decides whether a
  right-curve turn starts.
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

### Curves (v33–v51, lab circuit)

- **Curve mode** (v33): while the line stays on one side, a turn
  feed-forward is added and the reference direction follows the car, so
  a bend is followed instead of being taken for a weave.
- **Left curve:** starts after 60 mm of centre white. Its extra turn is
  capped at 26, about a 145 mm radius at 100 mm/s (v45: the 40 cap, about
  108 mm, ended the 270° curve 15–25° short and the recovery swerve
  spoiled a barcode just after it).
- **Right curve:** the line leaves to the right, so it is turned blind at
  once from the right IR, at a corner-sized radius, for up to 90°. If the
  90° runs out, the search looks right first (v51), the curve's own
  direction.
- **Curve exit:** the curve turn fades over about 10 mm (v36), and for
  100 mm after a curve the steering is capped at ±25 (v45), so the car
  does not swing into the opposite curve.
- **Cross or curve:** a junction needs the centre black for 10 mm past
  the arm (v35); the left IR tells a cross or seam from a right curve
  (v42).

| Build | Runs | Result |
|---|---|---|
| v34 | right curve | taken as a cross (both IRs black on entry): fixed in v35 |
| v36 | circuit, both directions | the exit swing into the opposite curve gone |
| v45 | `c`, 2 runs | lapped 13 m each, in opposite directions, no fault; the gentler left turn did not run wide |
| v51 | `c` clockwise | laps consistent; one stop at a mat join where the line steps about 20 mm sideways (environmental: to be fixed on the mat) |

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
- **One bar short** (v46, v47): a read missing the first or the last
  narrow bar still decodes if the rest of that `*` is exact and the other
  `*` is exact. A read cut at both ends does not. In 500,000 random bar
  patterns this added 4 false reads, against 332 the decoder already
  allowed by chance.
- **Reading mode:** from the first bar until the read ends, the car keeps
  following the edge with a fixed ±1° aim. Its heading then stays within
  about ±2°, so the left IR crosses the bars on a steady path.
- **Commands** (v54, the professor's requirement): the mapping is not in
  the firmware. The PC sends four characters, for forward, U-turn, left
  and right, over MQTT (section 8), and a code means the same movement
  whichever way it is read. Until a mapping arrives, codes are read and
  logged but carry no command. The command is carried out at the next
  junction, turning on the spot from the line's direction.

### Evidence

| Test | Result |
|---|---|
| Driven scans, 50% sample (`barcode_drive`, 24 Oct) | 10/10 A / A-reversed, 6/6 Z / Z-reversed; no wrong decode |
| Command at the cross (v4–v7, 26 Oct) | 8/9 read; every read and command correct; turns 87.4–91.2° (from the line's direction); U-turn 180.0° |
| Command at the cross (v16, 28 Oct) | every `b` run decoded and turned correctly; junctions from both sides |
| A4 'A', blind straight hold (v27) | 3/4; the miss cut the code at its first wide space (fixed: at least 22 mm of white) |
| **v29, reading mode and the moved IR** | **5/5**: A4 'A' ×3 forward, 50% 'A', 50% 'Z'; barcode placed near and far from the start |
| Lab circuit, `c` laps (v45) | A4 `Z` read on every lap; the 50% code just after the 270° curve failed (narrow bars only 215–280 mV, one or two lost per read) |
| `r` runs after the curve, flat sheet (v45) | 2/3 decoded; the third missed only the last narrow bar (decoded by v46) |
| `c` laps, 50% `A` after the curve (v47) | 1–2 of 3 laps; the failures dropped a narrow bar |
| **Lower IR, narrower slit (user, v49)** | narrow bars now 640–950 mV (from 215–260 mV); `A` decoded on 2 of 3 laps, the miss on a lap where the car was still swinging out of the curve |

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
- **Lift (end of the demo; v55, 16 Nov):** the professor replaced
  collision detection with one rule: picking the car up stops the motors.
  Buddy 4's lift check (`terrain_lift_update`) runs every 20 ms of a run,
  in the obstacle and junction sequences too:
  - all three IRs black (no floor) for 1 s is a lift (v58);
  - an IMU cue makes it quicker: tilted more than 25° from rest for
    0.1 s, or the acceleration's size more than 300 mg off 1 g (a pull
    up or a drop; ignored on the ramp and for 1 s after it) arms a 2 s
    window in which 0.5 s of no floor is a lift (0.3 s until v60:
    normal driving reached 32° and 328 mg, both cues, so the margin to a
    0.18 s cross was too small);
  - a cross blackens all three IRs for only ~18 mm (0.18 s).
  - Why the IRs lead (v57 runs, 18 Nov): two `l` runs lifted by hand
    stopped only after 3 s (the v56 fallback). In both the IRs lost the
    floor at once, while the IMU saw 24.5° and 449 mg on a quick lift
    and only 17° and 251 mg on a slow one, and the hump detector called
    the lifted car "on the ramp" within ~26 mm. Normal driving reached
    27.7° of tilt in turns (full-course run 4) without a stop, and a
    light 13 mm ramp under a wheel never put all three IRs on black
    (run 6). A black ramp has not been tried.
  - On a lift the controller raises `FAULT_LIFTED` (bit 6, in the motion
    mask): the motors stop at once and stay off until the next run, and
    the run ends in state `LIFTED` (telemetry state 6) with a `LIFT`
    event. Each run's printout gives the largest tilt and acceleration
    seen off the ramp, the margin a normal run keeps below the cues.
  - Host test (`tests/host`): a bump then a cross, a light ramp, 0.5 s
    of black on the ramp after a jolt, the landing and a stale cue do
    not fire; quick, tilted, slow level, "held as a ramp", upside-down
    and dropped lifts do.
  - **Lift runs on v58 (18 Nov), car following a taped line:**

    | Lift | All three IRs black from | Stopped | Cue |
    |---|---|---|---|
    | Quick | 518 mm | 546 mm, ~0.3 s later | acceleration, 375 mg |
    | Slow | 610 mm | 710 mm, 1.0 s later | no floor, 1015 ms |

    Both runs ended LIFTED with `FAULT_LIFTED` and the motors off.
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
   - A rounded corner counts as box (v48): a reading up to 60 mm beyond
     the face (`PROBE_SHOULDER_MM`) still counts, so a rounded box is not
     cut short.
4. **Go round** (on the left):
   - square up to the line's direction, measured only on plain following
     (no curve or junction, at least 30 mm; v47);
   - side-step until the car's right side clears the left edge by 40 mm
     (planning for at least 100 mm of width);
   - drive past the face, a depth taken as the planned width (100–200 mm,
     v49), and the car's 90 mm left half-width plus the margin;
   - side-step back until the centre or right IR sees the line, then on
     157 mm to sit on it (up to 60 mm beyond the planned distance; v49),
     and carry on following.

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
- **Lab circuit (v47–v49, 31 Oct):**

  | Build | Obstacle | Width | Avoid |
  |---|---|---|---|
  | v47 | 10.5 cm sharp box | 95, 96 mm | rejoined, 68 s each |
  | v47–v48 | 9.5 cm rounded box | 42–68 mm (corner echo lost) | rejoined; the 100 mm minimum covered it |
  | v49 | 9.5 cm rounded box | **95, 96 mm** | line met 5 and 7 mm from plan; turns within 0.6° |
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
  stop request (`s`, or STOP over MQTT), a stopping fault, or a failed
  line search. `c` reads codes without acting on them (a lapping test);
  `f` (v53) is the full course with the commands carried out, and MQTT
  START starts it.
- **Motion state** (v54): stop, stationary, straight, turning left or
  turning right, from the line task's steering while following and from
  the wheel speeds otherwise. It is sent in the telemetry.
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
- **Continuous course:** run on the lab circuit from v33. Laps of 13 m
  in each direction with no fault (v45), every scenario chained: curves,
  crosses, the ramp (26–27 mm black ramp), barcodes and obstacles. The
  v52 CPU figures come from a 16.1 m `c` run (section 11).
- **`f`, full course with barcode commands (v57, 18 Nov):** on a new,
  larger circuit, with the mapping sent from the dashboard over MQTT
  (forward C, U-turn A, left Q, right S):

  | Run | Distance | Result |
  |---|---|---|
  | 3 | 4.9 m | `S` read reversed twice; both right turns done (90.2°, 89.2°) |
  | 4 | 13.3 m | obstacle measured 145 mm and gone round in 78.7 s (line met 9 mm from plan, every turn within 0.34°); ramp 13 mm (ramp under the left wheel); `F` (not mapped) crossed straight; `Q` read reversed, left turn 89.4° |

  Both runs ended on STOP with no fault and no event dropped. Junction
  arms under the left IR log as one-bar failed reads, as expected.
  `LIFT CHECK` in run 4: largest tilt 27.7° and acceleration 138 mg off
  the ramp in normal driving. The tilt passed the 25° cue without a
  false stop (the IRs still saw the floor), so the IR "no floor" check is
  the lift's real guard.
  MQTT: telemetry stalled for stretches (send buffer full) while the
  phone and laptop were far from the car; the link recovers when it is
  closer, so keep the phone near the middle of the course.
- **Cycle time estimate** for one ramp, one barcode and a ~20 cm obstacle
  on 9 mats:
  - normally about 2 min: 31 s of following, 5–6.5 s of turn, about 85 s
    for the obstacle;
  - worst case about 3 min.

  The demo allows 10 min.

## 8. Telemetry and remote control (Buddy 1)

### Design

- **Client:** Buddy 1's MQTT client (`lib/libnet/lwip/lwip_utk_mqtt.c`,
  merged 31 Oct, commit 2f193fb) runs in the radio task on a Wi-Fi
  build. It fills its packets from the car through the hooks in
  `app_program/buddy1_telemetry/tel_source.c`.
- **Status** (`inf2004/group27/telemetry`, every 500 ms): 11 bytes,
  bit-packed to each field's range (88 bits):

  | Field | Bits |
  |---|---|
  | version | 2 |
  | state | 4 |
  | left, right speed (±550 mm/s) | 11 + 11 |
  | distance (mm, wraps at 65.5 m) | 16 |
  | barcode command, valid | 3 + 1 |
  | highest ramp (mm) | 8 |
  | obstacle seen, sonar (mm) | 1 + 12 |
  | fault bits | 8 |
  | heartbeat | 8 |
  | motion (v54) | 3 |
- **Events** (`event`, 14 bytes each), timing (`sys`, once a second) and
  a last will on `online`.
- **Commands** (`command`, 1 byte): 00 START (`f`), 01 STOP, 0x10–0x16
  the `l o b a r p c` runs.
- **Barcode mapping** (`mapping`, v54): 3 bytes, retained, four 6-bit
  indexes into the 43 Code 39 characters (forward, U-turn, left, right).
  The car checks them (Code 39, all different), keeps them in
  `buddy1_telemetry/cmd_map.c` and answers on `mapping/ack`. A rejected
  payload keeps the old mapping.
- **Wi-Fi loss:** `FAULT_WIFI_LOSS` after 3 s without the broker
  (logged, the car drives on); the radio rejoins every 5 s.
- **Laptop:** `tools/mqtt_viewer.py` is the broker and viewer in one,
  with a control page: car status, obstacle, ramp and barcode panels,
  run buttons, the mapping box, events and run timing.

### Evidence

- **On the car (29 Oct, first client):** status and events arrived live
  and matched the serial log; the car reconnected on its own after a
  viewer restart; the last will showed OFFLINE 15 s after unplugging;
  line traces were unchanged with Wi-Fi on.
- **Codec tests** (`telemetry_codec_test.c`): round trip at every
  field's maximum, negative speeds, and rejection of out-of-range values
  (speed, motion) pass.
- **Mapping (v54):** from the control page to a simulated car, A/Z/B/C
  was sent as `ca b8 30`, and the page showed the car's confirmation.
  Bad and duplicate payloads were rejected with the mapping unchanged
  (PC test of `cmd_map.c`).
- **Style:** the client and codec were reformatted to Barr-C (4 Nov);
  the Wi-Fi firmware stayed byte-identical.

## 9. Code structure and Barr-C (28 Oct, commits ab86df2, b510d3b)

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

## 10. Change Requirement 1: minimum-bit data (Week 13)

**Done for Buddy 1:** the MQTT status is bit-packed to each field's
range, 88 bits in 11 bytes (section 8), the mapping is 4 × 6 bits in
3 bytes, and the faults are one 8-bit mask. The car-side structures below
are still at their unpacked sizes.

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

## 11. Memory and CPU (2 Nov, course build v52)

### Memory

Measured from the build with `arm-none-eabi-size`, per object file. The
RP2040 has 264 KB of RAM and 2 MB of flash.

| Build | Code (flash) | Fixed RAM (data + bss) | Task stacks |
|---|---|---|---|
| Course build, no Wi-Fi | 79.0 KB | 31.7 KB | about 14 KB |
| Course build with Wi-Fi and MQTT | 367.0 KB | 66.7 KB | about 14 KB, plus the radio's |

| Part | Code | Fixed RAM | Stack | Largest RAM item |
|---|---|---|---|---|
| Buddy 2, motion | 5.1 KB | 4.4 KB | 4 KB | 4.2 KB speed-step trace (test mode 5) |
| Buddy 3, line and barcode | 11.4 KB | 9.4 KB | 1 KB + 1 KB | 4.8 KB line trace, 2.2 KB scan buffer, 1.5 KB failed reads |
| Buddy 4, IMU | 4.1 KB | 0.5 KB | runs in the controller's | pitch and roll filter buffers |
| Buddy 5, ultrasonic | 1.0 KB | ~0 | 2 KB | none |
| Controller | 7.1 KB | 1.1 KB | 2 KB | 0.8 KB event log |
| Common (incl. CPU meter, LED) | 0.9 KB | 0.2 KB | 4 KB (LED task) | 0.2 KB CPU meter |
| Test log (`c` run and `p`) | 6.1 KB | 6.5 KB | runs in the start-up task | 4.8 KB copy of the line trace |
| Kernel, drivers, libraries | 37 KB | 9.2 KB | kernel's own | task tables |

About 18 KB of the fixed RAM (7% of the RAM) is diagnostic only: the
line trace and its copy, the speed-step trace, the barcode bar lists and
the status snapshots. They are kept in the final build on purpose: no
resource is near its limit, and the logs are what the course is tuned
from.

### CPU

Each task's time from waking to sleeping again is read from the RP2040's
1 µs timer (`common/cpu_meter.c`). Time a task spends waiting on a
motion command it asked for is charged to that task. Preemption by a
higher-priority task counts in both, so each figure is an upper bound.
The meter itself costs about 0.1–0.2% of the CPU (estimated), 164 B of
RAM and 0.9 KB of code.

`c` run of 16.1 m, v52 (2 Nov):

| Task (Buddy) | Priority | Share of the run | Longest pass | What the time is |
|---|---|---|---|---|
| motion (2) | 5 | 1.3% | 161 µs | 2 ms control loop |
| line (3) | 7 | 0.5% | 2.6 ms | 20 ms following loop; the longest pass is a search step |
| barcode (3) | 8 | 0.3% | 3.6 ms | 2 ms sampling; the longest pass is decoding a read |
| obstacle (5) | 6 | 7.7% | 9.2 ms | waiting in a loop for each sonar echo (up to ~6 ms per 65 ms ping) |
| controller (all, incl. Buddy 4) | 6 | 8.7% | 4.2 ms | 20 ms loop, IMU reads over I2C (polled) |
| **Total** | | **about 18.5%** | | about 81% idle |

The two largest shares are polled waits on hardware (the echo pulse and
the I2C bus), not computation. The motion loop's longest pass (161 µs)
is 8% of its 2 ms period.

## 12. Known limits and open items

- **Obstacle width:**
  - About ±3 cm on flat faces. Rounded corners over-read, which is safe.
  - The depth is taken as the width (100–200 mm), so an obstacle much
    deeper than it is wide could be clipped at its far corner.
  - Only the left side is used to go round.
- **Lift:**
  - needs the IMU: with FAULT_IMU there is no cue, so no lift;
  - a lift during a turn on the spot acts when the turn ends (a few s at
    most);
  - a very slow, level lift (under 300 mg and 25°) does not arm it; the
    lift runs will show the real margins.
- **Line following:** a mat join where the line steps sideways can be
  taken for a right curve (v51 run); the mats must be joined in line.
- **Barcode:**
  - the mapping must be sent from the PC after every restart;
  - a 50% code right at a curve's exit can still fail while the car
    swings out of the curve;
  - the IR height needs about 9 mm ± 0.5 mm, because the bars blur at
    10 mm.
- **MQTT range:** telemetry stalls when the car is far from the phone
  hotspot; place the phone near the middle of the course.
- **CR1 packing:** the MQTT data is packed; `robot_status` and the event
  messages are not.

## 13. Tribunal 2 verification (18–19 Nov, v57–v62)

Requirement IDs are the Tribunal 2 submission's (A1). Runs on the taped
line and the straight + cross mats unless stated; course firmware v59–v62,
Wi-Fi build with the dashboard.

| ID | Test | Result |
|---|---|---|
| FR1, FR11, NFR10 | Full course with commands on the new circuit (v57, 18 Nov) | 13.3 m, every scenario, no fault, about 3.5 min (79 s at the obstacle) |
| FR2, FR3 | Codes read both ways on the circuit and the mats | `A`, `Z`, `S`, `F`, `Q` read, forward and reversed |
| FR4, NFR6 | `b` runs, mapping from the dashboard | U-turn 178.7–179.7° (5 runs); forward: JUNCTION then STRAIGHT (3 runs); left 89.4°, right 89.2–90.2° (circuit). The mapping survives a car restart (the broker keeps it) |
| FR5, NFR5 | 11 mm ramp | read 13 mm |
| FR6, NFR3 | Dead sonar (ECHO unplugged), `o` runs (v62) | run ends FAULT (0x10) at 0 mm, no movement. Nose gap at every obstacle stop 5–7 cm (user) |
| FR7, NFR4 | 9.5 cm and 15 cm boxes | 95–96 mm (v49) and 145 mm (v57) |
| FR8 | `a` run, circuit (v57) | went round, line met 9 mm from plan, turns within 0.34° |
| FR9, NFR7 | Dashboard and MQTT | START/STOP and run buttons work; buttons return after an `o` run; a run key starts the next run after `c`; hotspot off mid-run: WIFI fault, car drives on, reconnects; car off: OFFLINE after 15 s; on again: ONLINE in about 15 s. v59: no publish failures (16 in-flight, Nagle off) |
| FR10, NFR8 | Lifts (v61–v62) | quick lift on the line 0.5 s (acceleration cue); slow lift 1.0 s (no floor); lift during an obstacle side-step turn 1.2 s (v60 turn check; 2.3 s before) |
| NFR2 | STOP (v61) | mid-turn: stopped 32° into a U-turn; in the obstacle measure: stopped, no width published; START then STOP within 1 s: never moved (0 mm). Mode 20: at rest 368 ms after STOP from 300 mm/s, queued moves discarded |
| NFR1 | Mode 20 / mode 5 (v60 motion code), course runs (v61) | loop 500/s; speed error 150/300/500 mm/s: -1/0/0 % (mode 20); step test 300: L 303 R 299, 500: L 506 R 500, 150: 148/148, -300: -297/-298 (all within ±2%); course runs 500/s after v61 (469–472/s on v59–v60, fixed by putting the Wi-Fi task below the motion task) |
| Faults | Mode 20 parts C–E | stall trips (MOTOR_STALL), left encoder trips (ENC_LEFT), 10 s free run: no trip |
| NFR9 | v58 build | RAM 31.7 KB (66.7 KB with Wi-Fi), CPU about 14% busy in course runs |

Fixes made during these tests: v59 (MQTT in-flight limit and Nagle; no
width from an aborted probe; STOP logged when it ends a sequence;
distance reset at run start), v60 (lift and STOP checked inside a turn on
the spot), v61 (Wi-Fi task below motion; lift cue path 0.5 s), v62 (dead
sonar stops the run before it moves; line trace cleared at run start).

## 14. Full-circuit laps and fixes (21 Nov, v63–v71)

Runs on the full circuit, `f` (full course, commands on), Wi-Fi build with
the dashboard. Each fix came from a trace in the run before it.

| Build | Problem in the run before | Fix |
|---|---|---|
| v63 | The left IR crossed the line on curves and reported false junctions | Left-IR junctions within 100 mm of a curve turn are logged (JUNCTION arg 3) and ignored |
| v64 | After a U-turn the line sat under R and the car spiralled; a left-IR junction on the ramp | After a junction turn, no curve turn until C is on the line's right edge (at most 250 mm); the ramp counts as a curve for the junction filter |
| v65 | S-bend: a blind right turn swept the car across the line (R, C, then L) and kept turning; an obstacle just after a curve was squared to a direction 13° off and the probe found no face | L seeing black in a blind right turn means the line is left; with no clean straight to square to, the car finds the face by turning (5° steps each way until the echo is lost) and faces the middle of that span |
| v66 | After a right junction turn the acquire window ended at once (before C had seen the line); a barcode read held a fixed aim with the line under R alone and the car left the line | The window ends only once C has seen the line; the line under R alone steers back as usual during a read |
| v67 | Leaving the S-bend, C saw the line for only 8 mm while turning right, so the car kept turning past it and reached the next barcode at an angle | C alone then white while turning right means the line is left, however short C's black |
| v68 | During a read, a bar under L was taken for the line (the v65 check) and the car turned left off it | The L check is off while a barcode read is in progress |
| v69 | An obstacle on a curve: after the creep the first ping had no echo (face ~7° off square at 197 mm) and the probe gave up | The face search runs again from the scan range after the creep, and searches outward when the heading it is on has no echo |

| ID | Test | Result |
|---|---|---|
| FR1, FR11, NFR10 | v67, one START, ended by STOP | **3 laps, 30.3 m** in 470 s, no fault (about 10.1 m and 2.6 min a lap, 75–80 s of it at the obstacle). v66: 2 laps, 20.9 m |
| FR4, NFR6 | Commands on the laps (v66, v67) | left 87.9° and 88.5°, U-turn 179.2°, forward crosses straight; the car picked the line up after every turn |
| FR2, FR3 | Codes on the laps (v67) | `Z` (forward and reversed), `A`, `Q` read; one `A` failed on the third lap: narrow bars peaked at 234–263 mV against 650–1700 mV in good reads and one bar was lost, the faint-print or lifted-paper fault seen before |
| FR6, FR7, FR8, NFR3, NFR4 | Obstacles on the laps (v65–v67) | 9 cm box just after the ramp (v65, face found by turning): 93 mm, rejoined 8 mm from plan. On the straight: 71 mm (v65), 146 and 123 mm (v66), 147 mm (v67); every avoid rejoined the line, within 22 mm of plan |
| FR5, NFR5 | Ramp on the laps | 10–12 mm (11 mm ramp) |
| NFR1, NFR9 | TIMING and CPU lines (v67) | motion loop at least 496/s, longest pass 116 µs; tasks 0.3–8.8% each |
| NFR11 | v63–v67 builds | all 7 test modes and the Wi-Fi build: 0 warnings; host tests pass |

**Final build v69** (`tests/full_auto/ctrl_course_v69_smp0.uf2`, commit
9069d3d), 21 Nov:

| ID | Test | Result |
|---|---|---|
| FR1, FR11, NFR10 | `f`, one START, ended by STOP | **2 laps, 22.9 m** in 387 s, no fault (about 3.2 min a lap, 80 s of it at the obstacle) |
| FR4, NFR6 | Commands on the laps | left 88.4°, 90.2°, 90.3°, 89.5°; U-turn 178.5°; forward cross straight |
| FR2, FR3 | Codes on the laps | `Z` (forward and reversed), `A`, `Q` read |
| FR7, FR8, NFR4 | Box one mat (33 cm) after a curve's exit, face found by turning (3 runs) | 15 cm face: 142 mm; 9.5 cm face: 71 mm and 96 mm; went round and rejoined each time. On the straight (plain following): 148 and 150 mm (15 cm face) |
| FR8 (limit) | Box 20 cm after a curve's exit | measured 96 mm, but the car passed it at an angle and its tail clipped the box. Closer than the one mat of plain line assumed between scenarios |
| NFR2 | STOP from the console during the turning search | stopped at once; no width published (run ended DONE, STOP_REQ) |
| FR5, NFR5, NFR8 | Two ramps on the laps, under the right wheel as intended, one of them black | pointed 11 mm ramp: 10 and 13 mm; flat-topped ramp (about 11.5 mm): 11 mm. The black ramp read like the light one and never stopped the car: under the right wheel it is outside the IRs' path, so the lift check's "all three IRs black" cannot happen. (One 21 mm reading is not a test: the ramp had been placed on the car's left, the left IR bracket pushed it aside and the car jolted part-way up and dropped back.) |
| NFR1, NFR9 | TIMING and CPU lines | motion loop at least 496/s, longest pass 135 µs; each task 0.3–8.8% |

Earlier v67 firmware: `tests/full_auto/ctrl_course_v67_smp0.uf2` (commit 48f68ec).

**v70** (3aab01f): bug-hunt group C tidy-up, no change to driving: the
telemetry event read skips the slot being overwritten; distance never
negative (it showed 655 m after a turn); obstacle width never negative and a
step limit on the side-step probe; the I2C disable wait bounded; the unused
drive-a-distance function and dead variables removed.

**v71** (3f2ff11): NFR1 loop dips. A few Wi-Fi course runs read 470 motion
passes in their worst second. `tm_printf` masks every interrupt for the
whole line (about 87 µs a character at 115200), so the radio's status
lines (`[wifi] link`, `[mqtt]` state, received command, mapping) held the
1 ms tick, and the 2 ms loop, off for about 60 ms when the link changed
mid-run. They are now held while a run is going and printed after it
(`app_console_quiet`, Buddy 1); telemetry, commands and reconnection are
unchanged.

| ID | Test (v71, 21 Nov) | Result |
|---|---|---|
| NFR1, NFR7, FR9 | `f` lap, hotspot off at the start for ~10 s then on, ended by STOP | FAULT_WIFI_LOSS raised, the car drove on and finished the lap (12.0 m, 200 s); motion loop **at least 496/s**, longest pass 114 µs |
| FR1–FR8, FR11 | the same lap | Z forward then left 89.5°; A then U-turn 179.0°; Q reversed then straight at the cross; 15 cm box 148 mm, rejoined 12 mm from plan; ramp 11 mm |

Final firmware: `tests/full_auto/ctrl_course_v71_smp0.uf2`.

## 15. Long-box obstacle avoid (24 Nov, v72–v82)

The demo obstacle is likely a rectangular box much deeper than its face is
wide. Until v71 the avoid took the depth as the measured width (capped at
200 mm), so a long box would be hit when the car turned back to the line.
Runs in `c` mode on the mats, Wi-Fi build.

| Build | Problem | Change |
|---|---|---|
| v72 | Depth assumed square | Side step to the box's left edge + 155 mm (the IR tip's swing when turning on the spot, from 140 mm ahead and 65 mm left of the axle) + 40 mm. Before driving back, facing the line, the sonar checks for the box's side; while it is there the car steps 60 mm further and looks again (cap 600 mm extra, then stop with a fault). Probe turns 80 → 120 mm/s |
| v73–v74 | The rear clipped the box on the final turn onto the line | The echo can drop out one step before the back corner: once seen, 2 clear readings in a row are needed |
| v75–v76 | A width over-read by 50 mm put the box past the "box seen" limit and the car turned in; the turn toward the line touched a box read narrow | Box seen = any echo nearer than the line (max 260 mm, below the ~290 mm floor echo); side margin 40 → 80 mm; clearance after the second clear reading 50 → 70 mm |
| v77 | A box up to ~130 mm deeper than its width was already passed at the first look | The past leg ends at the assumed back face; the check always needs 2 clear readings and always adds the 70 mm |
| v78–v79 | Heading drift over ~30 turns: line met 57 mm beyond plan; 120 mm reached another piece of track | Way-back search 60 → 80 mm |
| v80 | Squared up 14° crooked after a curve (width 47 mm for a ~70 mm face) | Face skew fitted from the width probe's range-against-offset readings; when the reference is not from plain following, the avoid turns by it (capped 20°, ignored under 2°) and restates the car and the edge; width / cos(skew) |
| v81–v82 | Turning on the spot over a cross arm, all three IRs read black for 1 s: a false lift | Inside the obstacle sequence, no floor alone is a lift after 3 s (an arm fits under the IR row for at most ~2.3 s of the face search); an IMU cue still makes it 0.5 s; 1 s everywhere else (v81 switched the rule fully off and a gentle lift was not stopped) |

| ID | Test (24 Nov) | Result |
|---|---|---|
| FR8 | Long box (18 cm face, longer than wide) `a`/`c` runs, v77–v82 | seen beside the car each time, went 255–344 mm further than the square guess, no contact, line rejoined 5–57 mm from plan |
| FR8 | Box one mat after a curve, v81 | skew −3.3° applied, width 69 mm, went 344 mm further, rejoined 45 mm from plan (v80 without the correction: rejoined another piece of track) |
| FR7, NFR4 | Widths with the 120 mm/s probe turns | 18 cm face 171–174 and 200 mm; 17 cm face 147 mm; ~7 cm face 69–71 mm (all within ±3 cm) |
| NFR2 | STOP from the console during the depth check, v78 | stopped at once, no width used, OBST_DONE failed + STOP_REQ |
| FR10, NFR8 | Gentle lift during the face search, v82 | stopped at 3.0 s (cue no floor 3016 ms) |
| CR2 | Depth cap | still blocked 600 mm past the plan: the avoid stops (EVT_OBST_AVOID arg 4), run ends FAULT; by design, not driven |

**Pending on v82:** one `f` lap with commands on and the long box, ended
by STOP (FR11, NFR10 lap time with the 100–120 s obstacle sequence, FR4,
NFR1); a firm lift during the obstacle sequence (expected 0.5 s, IMU cue).
Not tested: a square box on v77+ (the course is not expected to use one; the
same depth-check loop runs for every box).

Build: `tests/full_auto/ctrl_course_v82_smp0.uf2` (commit d11dd12).
