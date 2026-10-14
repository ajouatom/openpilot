# eGPU checkpoint latency correction — 2026-10-14

## Cause and correction

After the user power-cycled the comma, its eGPU resumed inference, but the
carrot-nav model emitted only 15.94 Hz and raised modeldLagging/permanent.
This was a separate software regression from the earlier repeated USB link
failure. The recently installed single-invalid-output recovery saved recurrent
state after every good inference using individual Buffer.copy_from dispatches.
That path had not been timed with the physical eGPU before its installation.

The measured worker run took about 56 ms; the existing upload/warp/model/read
boundaries summed to about 37 ms. The approximately 19 ms residual included
checkpointing and validation, not a separately measured old checkpoint timer.
The correction captures all backup assignments into a TinyJit graph during
initialization, warms/captures it before live frames, then reuses it. It retains
separate backup storage, recurrent input/output aliases, synchronization,
finite-backup checks, rollback semantics and the existing fallback policy.
New explicit checkpoint wall/CPU measurements expose this cost in diagnostics.
The driving artifact, its pinned runtime and AMD warp selection are unchanged.

## Verification and installation

- 92 focused output-recovery, adapter, warp and runner tests pass; Ruff passes.
- The actual pinned runtime on the comma passed a CPU-only contract test in an
  isolated temporary folder: five changed-state replay/restore cycles, retained
  aliases/storage, invalid-backup rejection and reset. The device lacks pytest;
  its test function was invoked directly with an equivalent raises context.
- Linux eGPU compatibility CI 38536204337 and Jetson integration CI 38536204338
  both pass on code commit f1db63441d.
- Fresh valid/alive Park, standstill, zero-speed and inactive-control checks
  guarded vehicle installation. Existing generated DBC edits were backed up
  and hash-verified unchanged. The vehicle fast-forwarded from 3808b5733a to
  f1db63441d; only modeld was restarted. The USB address, comma boot, Jetlink
  daemon and locationd process were retained. No power cycle was performed.

| Matched parked measurement | Before | After |
| --- | --- | --- |
| Mean reported inference time | 58.24 ms | 44.42 ms |
| p99 / maximum inference time | 59.94 / 60.78 ms | 45.51 / 46.03 ms |
| Model messages in 30 s | 478 | 600 |
| Consecutive frame IDs | 356 steps of 1, 121 steps of 2 | 599 steps of 1 |
| Invalid cameraOdometry messages | 121 | 0 |
| livePose inputsOK | 37 true, 441 false | 600 true |
| Driver model messages | 600, all valid | 600, all valid |
| Final alert | modeldLagging/permanent | empty |

New checkpoint timing is approximately 4.7 ms. A separate overlapping 65-second
observation has 1,300 valid model outputs at 19.999 Hz and 1,301 valid driver
outputs at 20.004 Hz, with eGPU active throughout. This is parked validation,
not road validation or an injected numerical-fault experiment.

## USB and branch scope

After the user's power cycle, an earlier 65-second observation had no USB
re-enumeration and 18 additional root-port link errors, versus about 679/s
before the cycle. During the corrected-code 65-second observation the same USB
address remained present with 29 additional errors. The checkpoint fix restores
inference cadence; it does not claim to eliminate USB link errors or identify
the defective physical component.

Read-only inspection found an enabled smb2-vbus regulator control and a hub
advertising per-port power switching. Actual selective VBUS interruption,
separate GPU-power removal and automatic power cycling remain untested and
unimplemented; a USB reset is not proof of electrical power removal.

The affected checkpoint implementation exists on carrot-nav. carrot-wip's
worker does not use OutputRecovery or save recurrent checkpoints, so it does
not have this specific regression. Preserve that branch's existing policy;
do not introduce the feature merely to carry a fix. Publish the investigation
on both maintained branches. The separately published background reconnection
code is now present on the vehicle through the fast-forward, but no failed-link
retry/READY/rejoin was exercised by this successful startup test.

Raw before/after telemetry and deployment evidence remain private under the
2026-10-14-egpu-recovery analysis directory.
