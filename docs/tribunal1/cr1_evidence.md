# Change Requirement 1: minimum-bit data — evidence log

Measurements for the Week 13 submission: RAM before/after, message sizes
before/after, and correctness/timing. Sizes come from `arm-none-eabi-size`
on the uniprocessor build (`make`, `mtk3pico_smp0_uart.elf`) with
`MOTION_TEST_MODE = DEMO_ROAM` (the integrated Buddy 2 + 4 + 5 program), so
every row compares the same application.

## Baseline (before any change), commit e8358aa

| Image | text | data | bss |
|---|---|---|---|
| Whole firmware (DEMO_ROAM) | 57782 | 52 | 14744 |
| motion_task.o | 3860 | 2 | 4376 |

Message and shared-data sizes (`sizeof`, bytes):

| Type | Used for | Bytes |
|---|---|---|
| `motor_cmd_t` | motion command queue message | 6 |
| `odometry_t` | odometry snapshot to other tasks | 24 |
| `enc_diag_t` | encoder snapshot | 28 |
| `motion_trace_t` | PI trace entry (test only) | 14 |
| `imu_raw_t` | IMU sample | 12 |
| `imu_attitude_t` | pitch/roll/heading | 6 |
| `hump_event_t` | hump report | 16 |
| `collision_event_t` | impact report | 8 |
| `obstacle_profile_t` | obstacle scan result | 18 |

Timing baseline: motion loop 995–996 passes/s (7 Oct, Appendix L13); PI
steps within 1 % of target (7 Oct, L1).

## Phase 1: Buddy 2 cleanup and safety (Tribunal 1 decisions)

Not the bit-packing yet; it removes dead data first so the packing only
touches the final structures.

- Polled quadrature decoder removed (hardware edge counters replaced it):
  `enc_diag_t` 28 → 8 bytes.
- No printing in the motion loop (`[ODOM]`, `[MOTOR]` gone).
- Shared fault bitmask: 7 faults in one byte (`common/faults.h`), set and
  cleared inside a critical section (`common/critical.h`: DI/EI, plus a
  spinlock on an SMP build).
- STOP is a one-byte flag checked every pass; queued commands are discarded.
- Odometry and encoder snapshots read under the same lock the motion task
  writes them with.

| Image | text | data | bss |
|---|---|---|---|
| Whole firmware (DEMO_ROAM) | 57010 | 52 | 14732 |
| motion_task.o | 3576 | 2 | 4360 |

Most of motion_task.o's bss is the 300-entry PI trace (4200 bytes, test
only); it will be compiled out of the final build in the packing phase.

Timing/correctness after phase 1 (`MOTION_TEST_SAFETY`, car lifted,
21 Oct):

| Check | Result |
|---|---|
| PI at 150 / 300 / 500 mm/s | L 148 / 298 / 498, R 147 / 300 / 498 mm/s (within 2 %) |
| Motion loop rate | 500 passes/s (see note) |
| STOP at 300 mm/s, 3 commands queued | wheels still after 386 ms; queued commands discarded |
| Both wheels held | MOTOR_STALL raised, wheels stopped |
| Left wheel held | ENC_LEFT raised, wheels stopped |
| 10 s free running at 300 mm/s | no fault |

See "Finding: the motion loop runs every 2 ms, not 1 ms" below for the
loop rate.

Motion task CPU and RAM (`MOTION_TEST_CPU`, lifted, 21 Oct): 0.20 % CPU
stopped, 0.33 % driving at 150/300/500 mm/s (about 6 us per pass, longest
13 us), not counting kernel task switches; stack high-water 296 of 4096
bytes.

## Finding: the motion loop runs every 2 ms, not 1 ms

**Decision (21 Oct): keep the 2 ms loop. No code change; the documents
are corrected.** Change type for the Tribunal 1 change log: *Clarified /
no change*.

### What was claimed

The Week 6 design and Tribunal 1 (change log row "Hardware PWM-slice edge
counters with a 1 ms Motion loop", risk R1, and the "All tasks" subsystem
test) describe a **1 ms motion loop** and quote **995–996 polls per
second** (Appendix L1a and L13, 7 Oct) as evidence, with a pass mark of
"990 per second or more".

### How the finding came about

1. Phase 1 (commit 7eff2d7) added a direct timing measurement to the
   motion task: it counts loop passes and divides by the elapsed kernel
   time once per second (`motion_loop_rate()`).
2. The `MOTION_TEST_SAFETY` run (21 Oct, car lifted) reported
   **500 passes/s** at 150, 300 and 500 mm/s, half of the quoted figure.
3. A second, independent measurement (`MOTION_TEST_CPU`, commit 2c58b80)
   timed each pass with the 1 MHz hardware timer: again **500 passes/s**,
   about 6 µs of work per pass, so the loop is not running late; it
   sleeps for 2 ms each time.
4. The loop code was compared with the 7 Oct version (commit c45861c):
   unchanged (`TASK_PERIOD_MS 1`, `tk_dly_tsk(TASK_PERIOD_MS)` at the end
   of each pass, 1 ms kernel tick `CNF_TIMER_PERIOD 1`). So the rate had
   not changed since 7 Oct; the old figure had to be re-read.

### Why the loop period is 2 ms

A pass is one trip round the motion task's `while (1)` loop: read the
encoder counters, check STOP, take at most one command, run the PI every
20 ms, update the PWM, then `tk_dly_tsk(1)`.

`tk_dly_tsk()` guarantees *at least* the requested delay. The kernel adds
one tick to every relative timeout so a wait can never be cut short by a
tick that is already part-way through
(`kernel/tkernel/timer.c`, `knl_timer_insert_reltim()`:
`event->time = current_time + tmout + TIMER_PERIOD`). With a 1 ms tick,
`tk_dly_tsk(1)` therefore wakes on the second tick: one pass every 2 ms,
500 passes per second.

### Why the 1 ms figure was wrong

The 7 Oct fixed-time test (`motion_fixed_time_runs()` in commit c45861c)
did, for each run:

```
motion_reset_odometry();      // polls counter = 0
motion_set_speed(...);
tk_dly_tsk(FIXED_RUN_MS);     // 1000 ms run
motion_set_speed(0, 0);
tk_dly_tsk(FIXED_SETTLE_MS);  // 1000 ms settle
motion_print_encoder_diag(label);   // prints polls=995
```

The polls counter therefore covered the **1 s run plus the 1 s settle,
about 2 s**, so `polls=995` meant ~995 passes in 2 s, i.e. ~500 per
second. It was read as "per second" because the test was described as
"1 s runs" and the loop constant is named `TASK_PERIOD_MS 1`. That
reading went into the Week 6 design figures and was repeated at
Tribunal 1 without a direct per-second measurement. The figure was
consistent with the code all along; only its interpretation was wrong.

### Why it does not matter for control, and why we keep 2 ms

- The PI speed loop runs every 20 ms, ten passes apart either way.
- The encoders are counted by PWM-slice hardware, so the loop rate does
  not affect how many edges are seen.
- STOP latency is at most one pass (2 ms), small next to the ~380 ms
  braking ramp from 300 mm/s (`MOTION_TEST_SAFETY` part B: 386 ms).
- A true 1 ms loop (a 1 ms cyclic handler waking the task) would roughly
  double the motion task's CPU (0.33 % → ~0.7 %, plus ~1000 task switches
  per second) for no control benefit.

### Corrections to the Tribunal 1 documents

| Item | Was | Now |
|---|---|---|
| Change log, "edge counters with a 1 ms Motion loop" | 1 ms loop; polls 995–996/s (L1a, L13) | 2 ms loop, 500 passes/s; L1a/L13 counts span 1 s run + 1 s settle |
| R1 mitigation / all-tasks test pass mark | polls counter 990/s or more | loop rate 495 passes/s or more (`motion_loop_rate()`) |
| Evidence | L1a, L13 | `MOTION_TEST_SAFETY` and `MOTION_TEST_CPU` logs (21 Oct), commit c45861c test code, kernel `timer.c` |

## Buddy 4: one-sided hump (course test), 21 Oct

Course test (user, 21 Oct): the 1.0 cm or 2.4 cm ramp under the **right
wheel only**; success = the car rides it, stays straight and still
follows the line afterwards. No full-width hump on the course.

**Method change.** With one wheel raised, the car rolls rather than
pitches, so the hump is now measured from roll: height =
114 mm (track) × sin(roll). Roll is smoothed (7-sample mean, ~0.14 s);
a side hump starts when |roll| stays past 2.5° for 20 mm and ends when it
is back within 1.5° for 15 mm; the height uses the most-tilted 30 mm
mean (`terrain.c`, commits 54af32f, 020259b). The pitch detector's report
of the same ramp is dropped when the car rolled during it. While on the
hump, `motion_set_hump_mask()` masks the stall/encoder checks.

**Why not the compass for heading hold.** One-sided ramp log
(`IMU_TEST_RAMP_ROLL`, e0abadd): on flat floor the compass reported a
48° turn over 50 cm while the encoders showed 1.5°; the field changes
with position on this floor, so a compass heading hold would steer the
car off course.

**Results** (`IMU_TEST_SIDE_HUMP`, 50 cm at 150 mm/s):

| Run | Ramp | Detected | Height | Sideways offset |
|---|---|---|---|---|
| A | flat | none | – | – |
| B | 1.0 cm | right | 12 mm | – |
| C | 2.4 cm | right | 27 mm | – |
| 1–6 | 2.4 cm | right (6/6) | 23, 23, 25, 26, 26, 26 mm | 19, 8, 9, 8, 0 mm (5 measured) |

Flat-floor baseline (`MOTION_TEST_STRAIGHT`, 5 × 50 cm): 14, 7, 0, 7,
2 mm right; tyre difference about 0.5 %, within setup error, no trim
applied. Hand-turn check: L 6320, R 6327 edges / 10 turns.

**Impact confirmation on the ramp.** Ramp and IR-bracket jolts reached
1659 mg, above the 500 mg jolt threshold, but the wheels never dropped
below 81 % of the command, so every one was correctly ignored by the
wheel-slowdown rule (IMPACT = jolt plus wheels below 60 % within
250 ms). The only "would count" cases were at the commanded stop and the
pull-away, where the wheels slow because they are told to; the
controller therefore confirms impacts only while `motion_is_steady()`.

**Open (hardware).** The IR bracket strikes the 2.4 cm ramp and the left
IR arm drags on the descent, steering the car by a varying amount
(offsets 0–19 mm). Software cannot see or correct this; a smooth skid on
the IR mount is the proposed fix. Line following (Buddy 3) corrects the
remaining heading after the ramp.

## Buddy 5: obstacle stop distance (watch task), 21–22 Oct

Decision implemented (Tribunal 1): stop when 2 of the last 3 pings read
≤ 300 mm from the sensor face; one missed echo no longer clears the
count; 300 mm/s while watching (`obstacle.c`, commits 5d99d66, b5677e6).
Course requirement: stop with ≥ 5 cm between the nose (IR tip) and the
block.

`OBST_TEST_STOP`, 300 mm/s, per-ping trace (22 Oct):

| Run | Target | Reading at trigger | Braking | Reading at rest | Nose gap |
|---|---|---|---|---|---|
| 1 | box (flat face) | 247 mm | 65 mm | 185 mm | 12.7 cm |
| 2 | box | 271 mm | 64 mm | 221 mm | 15.0 cm |
| 3 | box | 265 mm | 62 mm | 221 mm | 15.0 cm |
| 4 | box | 283 mm | 65 mm | 208 mm | 14.6 cm |
| 5 | box | 277 mm | 66 mm | 201 mm | 14.3 cm |
| 6 | round bottle | 259 mm | 67 mm | (lost) | 10.6 cm |
| 7 | round bottle | 230 mm | 63 mm | 172 mm | 9.8 cm |

- With a flat face the readings fall smoothly, ~20 mm per ping; the
  first near ping is the first reading ≤ 300 mm and the stop triggers on
  the next one. Braking from 300 mm/s is 62–67 mm.
- The sensor face sits about 6 cm behind the nose (reading at rest minus
  nose gap: 58–71 mm).
- A round bottle reflects only a thin strip: near the end the trace
  shows readings jumping to 372–450 mm (the beam passing it and finding
  something further away), and readings just above 300 mm delay the
  trigger. An earlier bottle run (21 Oct) triggered at 191 mm and stopped
  5.9 cm from the nose; that was the bottle, not a code fault.
- Result: 7/7 runs meet the ≥ 5 cm requirement (minimum 9.8 cm, bottle);
  flat-faced box 12.7–15.0 cm. The 300 mm threshold is kept.
