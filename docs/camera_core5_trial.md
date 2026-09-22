# Camera CPU placement trials

## Current placement: September 22 task grouping

After the rollback and three parked grouping comparisons below, the user
approved moving card to core5 and planner to core4. Camera userspace and its
IRQ targets stay on isolated core6 with normal scheduling. This supersedes
the card/planner placements in the historical sections of this document.

| Core | Main application work | Scheduling |
| --- | --- | --- |
| 4 | controlsd, selfdrived; radarcan, planner | FIFO53; FIFO51 |
| 5 | card; radard | FIFO53; FIFO51 |
| 6 | camerad and camera IRQ targets | Camera SCHED_OTHER/nice0 |
| 7 | modeld/eGPU worker; DM model when enabled; GPU IRQ | FIFO54; FIFO5 |

UI remains on cores0..3/SCHED_OTHER. Core6 is reserved by application placement;
this is not a guarantee that no other kernel work or interrupt runs there.
The shared runtime placement applies to C3/C4, but only parked C4 measurements
are available. No inference, radar detection, lead-selection, CAN joining,
control algorithm, priority, validity threshold or user setting is changed.

Three separate 30/60/30-second before/trial/restored comparisons held the
camera/IRQ on core6. Each transition had a five-second settle, P/zero speed/
controls-disabled guards, and no ftrace. Card moved to core5 in all trials:

| Candidate | Planner / radarcan / radard cores | Road / wide max age (ms) | Planner max work (ms) | Radar input max age (ms) |
| --- | --- | --- | --- | --- |
| All card/planner/radard on core5 | 5 / 4 / 5 | 37.050 / 37.434 | 32.080 | 15.513 |
| **Selected: planner on core4** | **4 / 4 / 5** | **36.889 / 37.335** | **19.132** | **24.953** |
| CAN work on core5, planning/radard on core4 | 4 / 5 / 4 | 36.738 / 37.215 | 20.273 | 19.075 |

Surrounding baseline maxima were 51.451-58.083ms for road/wide cameras. Camera
runnable wait fell from 59.21-97.60ms/s to 5.69-5.88ms/s. Camera age means
message construction minus estimated SOF+11ms EOF, not measured hardware EOF
or IPC arrival. Card still consumed about 57% CPU and 5.5ms per 100Hz cycle.
Moving it away repeatedly improved camera timing; this is evidence of a CPU
sharing cost in these conditions, not an explanation of every past SOF/IFE fault.

The selected trial's planner max work increased from about 8ms to 19ms and
radar input max age from about 15ms to 25ms. Its radard model age averaged
5.321ms (max 15.428), compared with trial1's 24.234ms (max 36.488) and trial3's
18.037ms (max 28.140). Named core4 tasks used about 76% CPU and core5 about 61%,
excluding other threads, background tasks and IRQs. Extra load remains a risk.
All measured phases had no model gaps, invalid odometry/pose inputs, CAN-invalid
carState, invalid radar/plan/control messages; IMU ages stayed below 34ms.
DM was disabled (DisableDM=2). Driving with more radar objects, DM-enabled
operation, C3, thermal behavior and failure-rate improvements are unvalidated.

After approval, the selected placement was applied live and retained after a
further 90-second check: road/wide maximum ages 36.782/37.258ms, camera runnable
wait 5.80ms/s, zero model gaps or invalid odometry/pose/CAN/radar/plan messages.
Planner maximum work was 20.813ms and radar input maximum age 26.824ms; these
remain trade-offs rather than universally improved timings. All card threads
read back core5 and planner threads core4. The vehicle checkout was still
eda4f745: this live application does not install the committed source update.
The new commit must be installed for the grouping to persist after process
restarts. Local regression checks passed 30 tests with 5 Linux-only skips.
Card's 22 existing Ruff findings were unchanged; other edited Python files
passed Ruff. No native camerad build was needed for its comment-only change.

The following sections retain the earlier trial and rollback evidence.

## Historical September 22 follow-up: camera placement rolled back

The camera/IRQ part of this trial is reverted to core6. The UI remains on
cores0..3 with its verified SCHED_OTHER policy. All control, radar, model and
cluster placements are unchanged. Camera policy/nice remain SCHED_OTHER/0.
The table and rationale below describe the historical September 21 trial.

Parked Ioniq 5 C4 testing on `eda4f745` verified the live kernel command line
`isolcpus=6,7`: core5 admits ordinary background work. A complete scheduler
trace showed camerad runnable but waiting 24.266ms while proclogd consumed
13.155ms and planner/radard about 10.6ms on core5. The preceding sensor
exposure ioctl had completed its CCI wait, and the next road frame's ISP
completion was already recorded. A later 93.856ms wide-camera delay included
43.393ms of kswapd execution across two camerad runnable waits. That latter
sample had substantial temporary tracing storage on tmpfs, so it demonstrates
the interference path without proving an unperturbed incident frequency.
The tracing footprint and locally backed-up temporary files were then reduced.

With ftrace off, P/zero speed/selfdrive disabled throughout, the original
core5/SCHED_OTHER/nice0 configuration reproduced a 91.961ms road-camera age,
model frame 47481 -> 47483, invalid cameraOdometry, and livePose.inputsOK=false.
IMU ages remained below 34ms and sensorsOK stayed true. Camera ages here use
message construction time minus the estimated SOF+11ms EOF, not an independently
measured hardware EOF. A nice=-10 trial did not materially improve mean/p99
camera age, so no priority change is retained.

A separate camera+IRQ-only core5/core6/core5 comparison retained normal
scheduling and all other placements. Each transition had a five-second settle
interval; the measured phases were 45/90/45 seconds:

| Measurement | core5 before | core6 | core5 restored |
| --- | ---: | ---: | ---: |
| Road mean age (ms) | 37.657 | 40.041 | 36.749 |
| Road max age (ms) | 63.124 | 52.822 | 65.269 |
| Wide max age (ms) | 60.834 | 56.091 | 81.477 |
| Camera runnable wait (ms per second) | 84.595 | 56.649 | 69.018 |

There were no model gaps/invalid odometry in these three measured phases or
their transition intervals. Gyro/accelerometer ages stayed below 34ms.
Core6 reduced the observed tail and runnable wait, with a roughly 3ms mean-age
cost. This supports ending the unproven core5 trial; it does not establish a
failure-rate reduction from a short sequential test, a driving fix, C3 results,
or the cause of older SOF gaps/IFE faults. The earlier core6 IFE/SOF incidents
remain distinct evidence and must not be described as solved by this rollback.

Rollback regression checks: 30 placement/UI/pairing tests passed on Windows
with UTF-8 enabled; five Linux-only UI scheduler tests were skipped. Hardware
imports for UI guards were stubbed. Ruff and the patch whitespace check passed.
The live comparison exercised real Linux affinity/IRQ writes and restoration;
a full native camerad build and driving validation are not established by it.

The selected core6 placement was then applied live without restarting the
vehicle. A further 90 seconds / 1,801 frames per road camera measured maximum
ages of 54.807ms road and 56.935ms wide, with no model gaps/invalid odometry/pose
input failures. Camera runnable wait was 102.093ms/s in this later sample,
so the initial runqueue reduction is not a consistent result across samples.
The observed tail remained bounded in this sample; a short quiet interval does
not establish a fixed failure rate. The vehicle checkout remained `eda4f745`;
live affinity changes require the committed update to persist after restarts
or subsequent power-state reconfiguration.

The user authorized this trial on 2026-09-21 after Ioniq 5 C4 route
`00000f90--96d7dcd525--4` again produced a temporary Location alert.
Its wide-camera SOF interval was 101.038 ms despite consecutive frame and
request IDs. Pairing skipped one main frame, followed by one invalid odometry
message. IMU timestamps stayed within 35 ms and model execution stayed below
42 ms. These observations do not identify the cause of the camera time gap.

## Placement

| Work | Previous | Trial | Scheduling |
| --- | --- | --- | --- |
| On-road/off-road main UI | core0 bootstrap, then core5 | core0 bootstrap, then cores0..3 | SCHED_OTHER, verified at startup |
| camerad | core6 | core5 | Existing normal scheduling |
| Camera IRQ targets | core6 | core5 | Existing IRQ policy |
| card | core6 | core6 | FIFO53 |
| controlsd / selfdrived | core4 | core4 | FIFO53 |
| radarcan | core4 | core4 | FIFO51 |
| radard / plannerd | core5 | core5 | FIFO51 |
| modeld / eGPU worker | core7 | core7 | Existing FIFO54 |

All eight camera IRQ action names configured by Tici.set_power_save move
together: a5, cci, cpas_camnoc, cpas-cdm, csid, ife, csid-lite, ife-lite.
The GPU IRQ stays on core7. Offroad CPU online/offline policy is unchanged.
The UI still starts on core0 before creating its window and retries affinity
changes after OSError. Cores0..3 remain online during power saving.

The common TICI path covers C3/C3X/C4; it is not gated by the eGPU model.
External cluster HUD affinity/FPS, setup/updater apps, models, AGNOS, Bluetooth,
camera synchronization limits and pose validity policy are unchanged.
No new user setting is introduced. No radar detection, lead-selection or
replay code changes are part of this trial.

## Rationale and limits

The incident log places card (FIFO53) and camerad (normal policy) on core6.
Their mean CPU usage is 58.18% and 6.70% of one core. UI consumes 38.64% on
core5. Core5's observed total mean is 74.01%; replacing UI with camerad gives
an arithmetic estimate of about 42%, not a measured post-change result.
Moving the UI increases load on cores0..3. Its normal policy remains mandatory
so sensord, locationd and other realtime work take precedence.

This separates camera userspace work from card but does not reserve core5:
planner/radar realtime work and other allowed background tasks still run there.
Keep camera priority unchanged to isolate the affinity change. Do not infer
that card blocked hard IRQ execution or that same-core contention caused the
observed SOF gap. A core relocation cannot repair a physical sensor fault.

## Validation and vehicle follow-up

Desktop tests execute the real UI main function with mocked GUI/affinity calls
for both UI layouts, including a transient affinity failure. They execute the
real power-save method with mocked sysfs/IRQ operations across on/off/on
transitions and cross-check the C++ camerad target. Separate guards preserve
the control/model placements and UI's prohibition on FIFO promotion.
These are code-level regressions, not on-device scheduling or camera validation.
The Windows run passed 30 tests across placement, UI guards and camera pairing;
five Linux-only scheduler tests were skipped. Existing UI guards were loaded
with hardware/logging imports stubbed because Windows has no fcntl. Ruff and
the diff whitespace check passed. No device scheduler syscall was tested.

After updating and restarting, verify actual thread affinity and policy, all
available camera IRQ affinity readbacks, UI FPS, sensor publication/timestamp
ages, control/planner cadence, and camera SOF/EOF gaps. Compare repeated
similar drives; one quiet minute is inconclusive. procLog's last-CPU samples
can corroborate placement but do not replace affinity masks or IRQ readbacks.
Measure C3 and C4 separately. A full Linux camerad build and physical validation
remain required on the target; neither is established by Windows tests.

Rollback is the reverse of this placement change: UI render affinity core5,
camerad and camera IRQs core6. Keep normal UI/camera policy and all pose/sensor
validity checks. Do not roll back unrelated radar isolation or cluster changes.

Docs-Not-Needed: Internal CPU/IRQ placement trial; no setting or user workflow changes.
