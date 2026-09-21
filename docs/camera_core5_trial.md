# Camera core5 placement trial

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
