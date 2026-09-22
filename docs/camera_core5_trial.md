# Camera core5 placement trial

## September 22 follow-up: camera placement rolled back

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
