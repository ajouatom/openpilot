# eGPU background recovery while using the internal model

The user requested a few recovery attempts during internal-model driving,
followed by READY and the same transition opportunities as Jetson. This change
applies to an already-running precompiled generic AMD eGPU that suffers a known
USB transport failure or its worker response timeout. It does not add hotplug
selection after an internal-only startup, retry corrupted/numerically rejected
models, change the selected model/runtime, or deploy to a vehicle.

## Behavior

- Keep existing immediate internal-model fallback. After fallback, require
  three seconds of internal output health. Full post-camera work must remain
  within 50 ms, without camera frame skips, and publication gaps within 100 ms.
  Publications must be valid and their camera capture less than 150 ms old.
- At most three attempts per boot, with 10/30/60-second backoffs. The tmpfs
  budget survives modeld restarts; a reboot resets it. Missing USB waits rather
  than loading. Old worker cleanup must finish before another worker can start.
- Construct and probe the candidate outside the frame thread. Preparation and
  its subprocess use normal scheduling, nice 10 and cores 0-3. Recovery uses
  AMD warp exclusively, avoiding QCOM contention with the internal model/DM.
  Require at least 2 GiB available memory and twice the PKL size plus 512 MiB.
- Require five complete test predictions, with the last three each within
  50 ms including transfers/validation. Reset recurrent buffers in place and
  clear desire/recovery history before handing the candidate to live inference.
  Probe outputs never reach modelV2 or control.
- READY is a tested, live worker waiting for transition. Use the exact shared
  Jetlink gate: fresh (250 ms), valid, alive carState/selfdriveState/carControl,
  and either standstill with abs(vEgo)<0.01 or physical steering override.
  This gate does not command engagement. First live output still goes through
  normal model publication; existing control/validity policy remains in force.
- A candidate waits READY for up to 30 seconds; expiration releases it and can
  consume a later bounded attempt. A 150-second attempt/cleanup deadline stops
  further recovery. No overlapping worker is launched if cleanup is stuck.
- Frame delay/drop or missing internal publications cancels recovery and stops
  further attempts that boot. A separate health watcher also detects a stalled
  frame loop; receive polling remains cancellable even on a partial pipe line.
- If the recovered eGPU fails again, reset the previously used internal model's
  history before fallback. Repeated failures share the same three-attempt budget.
- Both HUDs and Korean/English Carrot Web distinguish RETRY, READY and stopped.
  Status expires after three seconds and READY invalidates the cached HUD layer.

## Validation and limits

Desktop tests cover moving internal-model operation, backoffs, finite budgets
across process restarts, no overlapping cleanup, stale/invalid vehicle gates,
health cancellation including a blocked frame thread, worker ownership transfer,
probe output/reset, pipe cancellation, and status/UI behavior. CI also exercises
in-place recurrent reset using the pinned tinygrad runtime on CPU.

Windows focused recovery/model/API validation passes 267 tests; Jetlink's local
contract suite passes 25 with five platform skips. Web display tests pass 13;
the production bundles were regenerated. Native Linux CI results are recorded
after publication.

These checks do not measure physical USB bus interference, memory pressure,
eGPU timing or road behavior. The health policy reacts to a delay; it cannot
promise that no first delayed frame occurs. No hardware fault injection or
vehicle installation was performed. Existing initial-fault detection/cleanup
and internal fallback latency are not shortened by this feature.

Three old tests in `test_egpu_status.py` were found to be stale before this
change: direct `_render` badge call versus the cached `_paint` path; a 40-second
loader timeout versus the existing 120 seconds; and a removed selfdrived
settling expression. They are recorded separately from recovery validation.

## Rust / CPU review

This task does not rewrite code in Rust. Existing `runtimeTiming` CPU totals
include Python, native libraries and kernel work, and cannot establish the
fraction removable by a language rewrite. The earlier Cython experiment in
`native_cpu_ioniq5_ab_20261002.md` measured radard CPU/loop down 38.6% and card
down 3.6%, with core5 mean 77.66% to 70.86% across different drive segments.
That is neither a Rust benchmark nor a prediction for a full rewrite. A whole
device percentage requires per-process Python/native profiling and matched
workload measurements; GPU/USB elapsed time is not CPU occupancy.
