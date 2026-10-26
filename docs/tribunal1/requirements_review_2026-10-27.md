# Requirements testability review (27 Oct 2026)

This review covers every requirement and test target from:
- the Week 6 design review (`Week_6_Design_Review_Group27.docx`, the same requirements as the submitted `INF2004_Group_27.docx`);
- the Tribunal 1 versions;
- Change Requirement 1;
- the course rules relayed from the professor (23 and 26 Oct).

**Limit:** I couldn't open the Google Doc tabs (Versions A–C) from this session. The Tribunal items below come from the two Version B reviews and the Version C update notes in the project folder, and from the Change Requirement 1 notes. The exact wording in the Google Doc should be checked by the design-review thread.

Each requirement is checked against the test runs logged in `docs/tribunal1/cr1_evidence.md`. Status:
- **✔ met:** tested and passing.
- **◐ partly:** tested in part.
- **✘ fails as written:** a test shows the wording can't be met.
- **? not testable:** no pass criterion, or no way to run it on the course.
- **○ not tested yet.**

## 1. Week 6 functional requirements

| ID | Requirement (short) | Testable? | Status | Notes |
|---|---|---|---|---|
| FR1 | Follow the line with the three IR sensors, recovering after a short loss | Yes, but "short loss" is undefined | ◐ | Straight + cross: 6/6 runs, with recoveries from off-line starts. **Curves never tested** (Wednesday). Define "short loss", e.g. "up to 40 mm of white", which is what the code does. |
| FR2 | Decode barcodes and execute Left / Right / Straight / U-turn | Yes | ◐ | A and Z: 26/26 driven reads, 12/14 line-following reads, every command correct. **Mapping not final** (the professor's A = right vs our A = left). |
| FR3 | Encoder moves: distance, turn, velocity, stop, PI per wheel | Yes | ✔ | PI within 2 % at 150/300/500 mm/s. Turns land within ~0.3° on the encoders. STOP works. |
| FR4 | Detect humps and report the **highest** peak height | Yes | ◐ | Detection works (tall ramp 22–27 mm against 24; small 12 against 10). **The highest peak isn't tracked or published yet**, only each ramp's height in the log. |
| FR5 | Detect obstacles, stop, profile location, **width**, closest point, clearance | Partly: there is no accuracy limit for width | ◐ | Stop works (7/7, ≥5 cm). **Width is unvalidated:** a 15 cm box read 300 mm on 9 Oct, and calibration has just started. "On **or near** the line" is undefined (see T6). |
| FR6 | Bypass without collision, reacquire the line, resume | **Not as written on the course** | ? | On 30.5 cm raised mats a ~15 cm car can't get round a block on the line without leaving the mat. The bypass can only be tested once the block placement is confirmed (Wednesday). |
| FR7 | MQTT telemetry, start/stop, heartbeat, reconnect | Yes | ○ | Buddy 1, not built yet. |
| FR8 | Detect a collision, **stop the motors** and report it | **No** | ? | The team decided on 26 Oct that impacts are **logged only**: the IR bracket hits the ramp, and the course has no impact scenario. The "stop the motors" half can't be tested and contradicts that decision. Reword to "detect and report", with stopping deferred. Detection was only checked against false positives (jolts up to 1659 mg correctly ignored). **No real impact has been logged.** |

## 2. Week 6 non-functional requirements

| ID | Requirement (short) | Testable? | Status | Notes |
|---|---|---|---|---|
| NFR1 | Encoders read **every 1 ms**, PI every 20 ms, ±2 % speed | Yes | **✘** (the 1 ms part) | The loop runs every **2 ms** (500 passes/s). The kernel adds one tick to `tk_dly_tsk(1)`, and the old "995/s" figure was counted over 2 s. You decided to keep 2 ms with a pass mark of ≥495/s. The ±2 % part passes. **Reword to 2 ms.** |
| NFR2 | A 30–50 cm move at 300–500 mm/s stops within ±3 % | Yes | ○ | Measured +1.0 to +3.2 % before the 0.328 mm/edge calibration. **Not re-run since the calibration**, and A2 says to repeat it on the track surface. |
| NFR3 | IR sampled **every 2 ms**, so ≤1 mm per sample at 500 mm/s | Yes | **✘** as written | The **line task samples every 20 ms**, which is 2 mm per sample at the course speed of 100 mm/s. The **barcode task** samples every 2 ms and keeps one reading per 0.33 mm. The car never runs at 500 mm/s on the course. Reword per task and at the real speed. |
| NFR4 | Ping every 60 ms. Stop when an obstacle is first within **20 cm of the car front** | Yes | **Superseded** | Tribunal changed this to 300 mm from the sensor, with the nose ≥5 cm away. Pings run every 65 ms. Tested 7/7. **Update the Week 6 wording, or mark it superseded.** |
| NFR5 | IMU every 20 ms. Hump height in mm, with accuracy measured, not promised | Yes | ✔ | Small ramp 12 mm (10 actual). Tall ramp 22–27 mm (24 actual), with one outlier of 37 mm while the bracket was digging (before the skid). The heights are now measured by **roll**, not pitch. |
| NFR6 | Telemetry ≥ every 500 ms. WiFi loss never delays control | Yes | ○ | Buddy 1. The controller-side snapshot every 500 ms is simulated in the test and works. |
| NFR7 | No brownout on start (soft start, 550 mm/s cap) | Yes | ◐ | No resets in any run since soft start, but there's **no explicit test record** (e.g. 20 cold starts in a row). |
| NFR8 | C on µT-Kernel, team template, **Barr-C** | Only by review | ○ | **Not checked.** It needs a Barr-C checklist pass or a static-analysis report on the final code. The test code in `app_main.c` doesn't follow Barr-C (e.g. `INT`/`LOCAL` style). Say whether test code is excluded. |

## 3. Week 6 statechart and assumption details with a pass/fail meaning

| Item | Status | Notes |
|---|---|---|
| LINE_SEARCH: "no line within **3 s** is a fault" | **✘** | The code searches on the spot within ±60° (up to 8 s per turn). On 26 Oct, run 6 searched for about 5 s and then succeeded. Reword the limit, or change the code. |
| START button on GP20; FINISHED on an end barcode | ○ | Not built. Runs are started from the console. |
| EXECUTE_BARCODE "returns once the line is seen" | ◐ | It turns at the junction. Following on after the turn exists but is switched off on the test mats (nothing past the cross). |
| A6: Motion loop ≥990 polls/s with all tasks running | **✘** | It's wrong for the same reason as NFR1. The correct pass mark is ≥495 passes/s. **Re-measure with every task running** (controller + line + obstacle + barcode). |
| A3: IR digital outputs work for barcodes | **✘** (resolved) | The digital outputs missed the 3 mm bars. Barcodes now use the left IR's analog output with a 1 mm slit. |
| A4: compass heading usable while driving | **✘** (resolved) | It drifted 48° on the floor. Heading now comes from the encoders. |
| Gap check: atomic odometry, "read at a high rate while driving to check" | ◐ | The critical section is built. The high-rate check was never run on its own. |

## 4. Tribunal 1 (Versions B/C) test targets

| # | Target | Status | Notes |
|---|---|---|---|
| T1 | Stop at 300 mm from the sensor, nose ≥50 mm away, watch speed ≤300 mm/s | ✔ | 7/7 (box 12.7–15.0 cm, bottle 9.8–10.6 cm). On the line through the controller: 22.5 cm. |
| T2 | STOP during a scan: **wheels stopped within 50 ms** | **✘** as written | Measured at rest in 40, 52 and 64 ms. The proposal to restate it as "STOP reaches the motors ≤2 ms, at rest ≤70 ms at sweep speed" is **still undecided** (with the design review). The reviewer also asked for 10 runs, or a bound worked out from the ramp. |
| T3 | Scan timeout 30 s raises a fault and stops | ✔ | Tested (part C of the scan test). Scans now take 15.7–16.3 s. |
| T4 | Stall (<30 % for 200 ms) and encoder (≥40 % duty, 200 ms) faults | ✔ | Tripped correctly lifted, no false trips. A 4 s cap on the ramp mask was added later. It isn't directly tested: the stuck run happened before the cap existed. |
| T5 | Motion loop rate pass mark | ✘ → restated | Was ≥990/s; now ≥495/s (see NFR1). |
| T6 | **A box beside the line must not trigger a stop** | **? / ○** | **Never tested, and "beside" isn't defined.** At 30 cm the beam is about 17 cm across, so a box within roughly 8 cm of the line's centre will stop the car. Define a lateral distance and test it. |
| T7 | Hump height within ±5 mm | ◐ | The tall ramp is within ±3 mm in most runs, with one outlier of +13 mm before the skid. The full-width hump test was dropped (the course only has one-sided ramps). |
| T8 | Impact = jolt **and** wheels below 60 % within 250 ms, confirmed only at steady speed | ◐ | Built. Only false positives were tested. Impacts are log-only (FR8). |
| T9 | Telemetry 2 Hz, heartbeat within 5 s | ○ | Buddy 1. |
| T10 | Fault bitmask (8 bits), set and cleared safely | ✔ | Built: `common/faults.h` with a critical section. |

## 5. Change Requirement 1 (minimum-bit data), Week 13 evidence

| Item | Testable? | Status |
|---|---|---|
| Every application field uses only the bits its range needs | Yes, by the data representation table | ○ The baseline is recorded. Packing hasn't started. |
| RAM use before and after | Yes (map file) | ◐ Baseline only (57782/52/14744). |
| Message sizes before and after | Yes | ◐ Baseline only. MQTT is Buddy 1. |
| Faults as bits in one shared mask | Yes | ✔ |
| Safe access to packed shared data | Partly, by review and stress run | ◐ Faults and odometry are in critical sections. Packed structs aren't built yet. |
| No timing degradation | Yes (loop rate, CPU, PI error before and after) | ◐ "Before" is logged (0.33 % CPU, 6 µs per pass). "After" is still to come. |
| MQTT sends compact binary, not text | Yes | ○ Buddy 1. |

## 6. Course rules from the professor (23 and 26 Oct)

| Rule | Testable? | Status | Notes |
|---|---|---|---|
| Ramp (one-sided, at most 2.4 cm): ride straight, then find and follow the line | Yes | ✔ | Tall ramp (the upper limit) 4/4 with the skid, both placements. |
| Barcode: decode **any A–Z** in either direction, execute at the next junction | Partly | ◐ | The decoder has the full Code 39 table, but only A and Z have been tested. "Any A–Z" can only be shown on a sample set: F, plus ideally a few letters with different wide-space positions. |
| Barcode command mapping | **No, not yet defined** | ? | Waiting on the professor. |
| Obstacle 10–20 cm: detect, **measure or estimate its width, report it** | **Not yet**: no accuracy limit, no defined report channel | ? / ◐ | Calibration is in progress. Agree a tolerance (e.g. ±2 cm or ±20 %) and where the width is reported (telemetry, log or both) before it can be passed or failed. |
| Obstacle: avoid, find the line, continue | Depends on block placement | ? | See FR6. |
| Stay on the mats | Yes | ✔ so far | Every test run stayed on the mats. Any avoid route must too. |

## 7. Summary: what can't be tested as currently written

1. **FR8:** "stop the motors on a collision". There's no course scenario for it, and it contradicts the log-only decision.
2. **FR6 and the obstacle avoid rule:** a bypass of a block on the line isn't possible on 30.5 cm mats. It's untestable until the placement is known.
3. **FR5 and the course width rule:** there's no accuracy tolerance and no defined report channel.
4. **T6:** "box beside the line" has no distance defined, and it has never been tested.
5. **Barcode mapping:** the commands aren't defined yet.

**Testable, but our results show the wording fails:**

6. **NFR1, A6 and T5:** "1 ms" and "≥990/s". The real figures are 2 ms and ≥495/s.
7. **NFR3:** "IR every 2 ms at 500 mm/s". The line task runs every 20 ms, and the car runs at 100 mm/s.
8. **NFR4:** "20 cm from the car front". It's superseded by T1.
9. **T2:** "wheels stopped within 50 ms". Measured 40–64 ms; the restatement is pending.
10. **LINE_SEARCH:** "3 s". A real search took about 5 s.
11. **A3 and A4:** both were shown wrong, and the design has moved on (analog barcode read, encoder heading).

**Testable, but not tested yet:**

- FR7, NFR6 and T9 (Buddy 1).
- NFR2 re-run.
- NFR7 cold-start record.
- NFR8 Barr-C review.
- A6 with every task running.
- Curves (FR1).
- More barcode letters.
- FR4 highest-hump report.
- Obstacle width.
- The CR1 "after" measurements.
