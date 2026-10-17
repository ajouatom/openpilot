# locationd CPU experiment withdrawn, 2026-10-17

## Final decision

The user judged the measured benefit too small and explicitly requested stopping
the optimization, using the existing code, and retaining only the record. The
candidate was never pushed to either maintained branch. All implementation and
test changes remain a private experiment, excluded from the documentation-only
publication. The following implementation and trial installation sections are
historical evidence, not the current deployment state.

## Scope and trial implementation

The user requested prioritizing locationd, estimating CPU savings, installing on
their ignition-on C4, and validating the result. Work is isolated from unrelated
signal experiments and generated settings files. No model, control policy,
sensor rate, process affinity, scheduling priority or Kalman equation changes.
The Cython extension must be rebuilt with the Python sources; copying only the
new Python files over an old native module is not a supported installation.

The EKF was already native. Add an opt-in compact result to its Cython boundary:
check finiteness of the returned observation's state/covariance in Eigen and
return owned residual arrays, without copying four state/covariance arrays into
NumPy. The default API remains unchanged for other consumers. Check the returned
observation estimate, not the final filter state after rewind replay; those can
differ for delayed measurements. C++ update, checkpoint and rewind remain intact.

PoseKalman reuses its fixed single-observation noise arrays. Native observation
storage copies those inputs, so retained rewind history does not alias the cache.
locationd consumes the compact result and avoids copying an already-owned
residual again. Preserve the existing camera rotation/translation update order,
last-successful-result finite check, reset timestamp, input validation and debug
observations. The ordinary PoseKalman API remains available.

## Native equivalence and checks

- A 60-second parked capture contains 13,864 valid replay inputs: 5,653 accel,
  5,572 gyro, 1,200 sampled carState, 1,199 cameraOdometry and 240 calibration.
- A hash-verified existing full driving log contains 18,761 inputs: 5,682 accel,
  5,639 gyro, 6,000 carState, 1,200 cameraOdometry and 240 calibration. Speed spans
  approximately 0--57.54 km/h and steering -10.1--17.6 degrees.
- Across all 32,625 inputs, the installed baseline and candidate have identical
  hashes of result enums, state/covariance bytes, stored observations/residuals,
  and complete debug livePose field dictionaries at camera updates. Replay uses
  the same timestamp-sorted input for both variants; it does not reproduce live
  subscriber batching, filter warm-start or closed-loop vehicle response.
- Two native unittest cases pass, including six NaN/+Inf/-Inf subcases across
  state/covariance, and 600 sequential/rewound/equal-time/rejected-old/reset
  observations. Check retained residual ownership and unchanged noise buffers.
- The shared extension's default API also preserves 400 car-filter learner
  updates, state/covariance and liveParameters fields exactly.
- Added tests pass lint. Three existing locationd lint findings are unchanged.
  No vehicle Python package installation is needed for the stdlib native tests.

## Isolated performance

Candidate Cython uses the device's recorded compilation command and existing
unchanged native EKF objects. Build and replay are under /tmp, SCHED_OTHER/nice19,
core0, with no production process changes during these measurements. Build has
a memory-headroom abort. Alternate old/new order over four pairs; use thread CPU
time, not wall duration. Both variants produce the same workload and debug output.

| 60-second recorded input, four runs each | Baseline | Candidate |
|---|---:|---:|
| Mean replay CPU seconds | 23.9431 | 21.7322 |
| Range, CPU seconds | 23.8661--24.0233 | 21.6204--21.9456 |

Reduction is 9.23%. Most savings are in accel/gyro handling. Camera-observation
rewind computation remains the main cost; this is not a complete native rewrite.
The first live baseline measured locationd at 38.85% of one core. Applying the
relative replay gain suggests about 3.6 percentage points of one core; amortizing
the absolute replay saving over its 60.026-second capture suggests 3.68 points,
or 0.92 points averaged over cores0--3. These are projections, not whole-core
onroad measurements or a reduction in the number of assigned cores.

The initial capture polled SubMaster on modelV2 and consequently conflated some
DM/pose observations. Its DM count is not a measured model-rate failure. Final
rate checks use independent non-conflating sockets and frame IDs.

## Vehicle installation and final observation

Selectively installed on vehicle 2ef833b2 with its existing HUD fixes. Only
locationd.py, pose_kf.py and the shared Cython source/binary were replaced; all
four originals were backed up under `/data/locationd-cpu-backup-20261017`.
Fresh Park/standstill/zero speed, disabled lateral/longitudinal controls and
pandas not allowing controls were checked before replacement and restart.

Manager preimports Python/native modules, so terminate it gracefully, wait for
all managed children to exit, then restart using the original stdout supervisor's
environment and manager affinity (drop only the already-released boot lock FD).
Two pre-install checks first stopped without replacing any files: manager's
display title combines argv, and its relocated /proc environment is empty. The
verified supervisor retains the original startup environment. No reboot/full
pull/model or setting change. Same boot ID and base commit; HUD remains enabled.
Verify installed hashes and mapped native inode in locationd and paramsd.

| Measurement | Before, 60.012 s | After, 300.010 s |
|---|---:|---:|
| locationd CPU, 100% = one core | 38.6259% | 35.2422% |
| modelV2 valid/consecutive frames | 1,200/1,200 | 6,001/6,001 |
| modelV2 publication rate | 20.001055 Hz | 20.000104 Hz |
| driverStateV2 valid/consecutive frames | 1,200/1,200 | 6,000/6,000 |
| driverStateV2 publication rate | 20.001152 Hz | 20.000072 Hz |
| livePose valid, inputs/sensors/posenet OK | 1,201/1,201 | 6,000/6,000 |
| livePose publication rate | 20.002340 Hz | 19.999833 Hz |

Observed locationd reduction is 3.3837 percentage points of one core (8.76%
relative), equivalent to about 0.846 points of the four-core pool's average
capacity. locationd remains on cores0--3; no core is removed. All three process
PIDs remain stable within the after interval. eGPU active is verified at both
ends; this is not continuous per-frame backend attribution. Park/inactive guard
remains true throughout the after interval. livePose is approximately 20 Hz,
not a strict >=20 Hz pass.

Core0--3 busy percentages change from 80.82/78.09/88.42/79.32 to
76.56/75.88/88.41/80.51. These whole-core values include other processes/kernel
work and a manager restart, so do not attribute their entire change to this
patch. Tail latency is not uniformly improved: model maximum publication gap
59.35 -> 86.63 ms, DM 72.39 -> 71.00 ms, pose 65.89 -> 91.64 ms. No missing model
or DM frame IDs in the final window; no claim of a hard 50 ms deadline.

Two livePose-invalid startup log lines were seen after restart, with zero
nonfinite-reset lines. All 6,000 final pose messages are healthy. Restart also
reduces total memory use; this is not evidence of a memory-leak fix or a memory
saving caused by the compact result. No closed-loop road or long-duration claim.

## Withdrawal and restoration

After fresh Park/zero-speed/standstill, inactive controls and panda permission
checks, restore all four files from the pre-trial backup. All restored SHA-256
hashes match the originals. Restart the manager gracefully to unload the trial
Python/native modules. The last tmux pane exits with that manager, so recreate
the comma session using the previously saved original environment and affinity.
No reboot, full pull, model or setting changes. Verify the original native
module inode is mapped by both locationd and paramsd, with no deleted mapping;
the boot and base commit remain unchanged and HUD remains enabled.

Final restored-code observation spans 60.012 seconds with Park/inactive guard
true throughout and unchanged process PIDs. Model: 1,201 valid consecutive
frames at 20.000005 Hz; DM: 1,200 valid consecutive frames at 20.003386 Hz;
livePose: 1,200 valid messages with inputs/sensors/posenet OK at 20.001242 Hz.
No missing model/DM IDs. eGPU is active at both observation endpoints (the
earlier immediate startup snapshot was still inactive). Restored locationd CPU
is 40.06% of one core during this window; startup/load variation means this is
not another matched performance comparison. Maximum publication gaps are
61.11/75.06/69.03 ms respectively, not a hard 50 ms deadline guarantee.
One SSH banner timeout required reconnecting before this successful observation.
The experiment is stopped and the vehicle uses its pre-trial implementation.

## Publication and evidence

Retain this record and repository memory only on carrot-nav and carrot-wip.
Do not publish the withdrawn source/tests, vehicle binaries, input logs, device
identity or private environment snapshots. Preserve navigation/model/runtime
differences and unrelated local changes.
Private reproducibility/evidence: `.analysis/scratch/2026-10-17-locationd/` and
device `/tmp/locationd_cpu_20261017/`.
