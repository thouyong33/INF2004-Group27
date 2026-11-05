# Evidence log: final implementation

INF2004 Group 27 robot car on micro T-Kernel 3.0 (RP2040, Robo Pico).
This log justifies the final implementation of each subsystem with test
evidence. Per the professor's note (29 Oct), it does not compare the
original design with the revised one; earlier steps appear only where a
design choice needs its reason.

- **Code:** `mtk3smp-rp2040/app_program`, branch `thouyong`, up to v54
  (commit 0f732f8, 5 Nov). `main` matches it up to v52 (2 Nov). Commits
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
| Obstacle (Buddy 5) | stop at 300 mm, creep to 200 mm, side-step to both edges, width = left − right; go round on the left with depth = width, return until the line is seen | every stop ≥ 5 cm (about 13 cm after the creep); width ±3 cm typical over 13 runs; rounded 9.5 cm box 95/96 mm, line met within 5–7 mm (v49) | Done |
| Controller | event-driven task; ramp, barcode and obstacle sequences; continuous course (`c`, `f`); rolling log and scenario reports; motion state | each sequence on its own; continuous laps on the lab circuit (v45–v52) | Done (`f` with commands to run on the course) |
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
- **Curve exit:** the curve turn fades over about 13 mm (v36), and for
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
   - A rounded corner counts as box (v48): a reading up to about 35 mm
     beyond the face still counts, so a rounded box is not cut short.
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
- **`f`, with commands:** built (v53, v54); to be run on a layout where
  a cross follows a code.
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
- **Line following:** a mat join where the line steps sideways can be
  taken for a right curve (v51 run); the mats must be joined in line.
- **Barcode:**
  - the mapping must be sent from the PC after every restart;
  - a 50% code right at a curve's exit can still fail while the car
    swings out of the curve;
  - the IR height needs about 9 mm ± 0.5 mm, because the bars blur at
    10 mm.
- **Full course with commands (`f`):** to be run where a cross follows a
  code, with the mapping sent over MQTT.
- **CR1 packing:** the MQTT data is packed; `robot_status` and the event
  messages are not.
