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

Note on the loop rate: `tk_dly_tsk(1)` with a 1 ms tick sleeps until the
second tick boundary, so the motion loop runs every 2 ms (500 passes/s),
not every 1 ms. This is not a change: the loop has always run at this
rate. The 7 Oct "polls=995/996" figure (Appendix L1a/L13) was counted
from `motion_reset_odometry()` to the print, which spans the 1000 ms run
plus the 1000 ms settle (`FIXED_RUN_MS` + `FIXED_SETTLE_MS`), i.e. about
995 passes in 2 s = ~500 per second. Quoting it as "995 per second" was a
misreading. The PI runs every 20 ms and the encoders are hardware
counters, so control is unaffected.

Motion task CPU and RAM (`MOTION_TEST_CPU`, lifted, 21 Oct): 0.20 % CPU
stopped, 0.33 % driving at 150/300/500 mm/s (about 6 us per pass, longest
13 us), not counting kernel task switches; stack high-water 296 of 4096
bytes.
