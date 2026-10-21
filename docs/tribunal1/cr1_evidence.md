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

Timing/correctness after phase 1: see the `MOTION_TEST_SAFETY` log
(loop passes/s and PI error at 150/300/500 mm/s, STOP, stall and encoder
trips) — to be filled in from the run.
