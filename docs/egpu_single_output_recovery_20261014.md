# Bounded single-output recovery on carrot-nav

The user requested retaining the preceding result after one eGPU output fault,
falling back after two consecutive faults, and installing all committed
carrot-nav work on the vehicle. This explicitly supersedes the previous
single-numerical-fault termination policy for the generic precompiled eGPU path.
Internal and Jetson inference paths are unchanged by this recovery code.

## Behavior

- After a successful generic eGPU result, checkpoint recurrent tensors into
  separate GPU allocations. Preserve the graph's existing in-place aliases.
- One completed inference containing NaN/Inf can be skipped only with a prior
  successful result no more than 100 ms old. Check the saved tensors for finite
  values and restore them before acknowledging the skip. Recheck elapsed time
  after restoration. Restore the parent's desire-pulse history too.
- Do not publish the failed result, reparse the previous prediction, advance
  action/pose from it, or attach a new camera ID/timestamp to old values.
  Existing consumers retain the preceding timestamped publication. At nominal
  20 Hz this creates one missing publication and a roughly 100 ms inter-result
  interval if the next inference succeeds; it is not a new 20 Hz result.
- A successful next result clears the consecutive-error counter. A second
  consecutive numerical fault takes the existing internal-model fallback.
  Device/time-out/protocol/shape failures, missing prior results, stale history,
  and failed state restoration still fail immediately. The legacy non-generic
  compiled graph lacks a complete recurrent-state checkpoint and keeps its
  existing rejection behavior.
- Parent and worker independently enforce one skipped result and the age bound.
  Boot smoke tests require every requested output; a skip cannot pass validation.
- An inference numerical rejection blocks the current boot. On a subsequent
  boot, matching artifact/failure/boot evidence permits the standard artifact
  revalidation and smoke-test path. This includes the old terminal combined
  `ValueError: invalid precompiled model output` for affected existing devices.
  Integrity/contract failures remain blocked. Original failure evidence is kept;
  no vehicle rejection marker is manually deleted.

## Incident and power evidence

The retained October 13 incident shows an invalid precompiled result at
12:18:55 KST, followed by internal fallback and a persistent artifact rejection.
The same rejection remains on the vehicle on October 14. This explains why the
same artifact is unavailable at a later startup. The old combined error did not
record whether the cause was NaN, infinity, or an output-size mismatch. It does
not establish the underlying GPU/model fault.

An October 14 live Jetson query still reports MAXN_SUPER, CPU 1728 MHz, GPU
1020 MHz and EMC 3199 MHz. Eight one-second tegrastats samples report board input
power about 13.4--13.9 W and GPU temperature about 77 C. This is the maximum
configured performance mode, not a statement of maximum continuous electrical
draw or an extended thermal-throttling test.

## Validation and limitations

194 focused desktop tests pass, covering single-fault rollback, separated
faults, consecutive failure, initial/stale-result rejection, restore errors,
protocol abuse, desire history, next-boot evidence, and existing delivery/warp
contracts. Ruff passes. CI also exercises actual state copies and alias
restoration with the pinned tinygrad runtime on CPU; that is not GPU timing.

Commit `dabe4f1d67` passed both Linux CI workflows: 297 model/delivery tests,
38 navigation tests, three pinned-runtime CPU tests (including state rollback),
761 Jetson tests with 11 skips, 119 navigation/cluster tests, and 11 protected
storage tests. The Windows installer checks also passed.

The extra checkpoint requires GPU copies and synchronization after each good
inference. The failed-frame path additionally downloads saved state to check
finiteness. Actual eGPU overhead and recovery latency are not measured because
the eGPU is currently disconnected. Parked Jetson verification cannot establish
eGPU behavior or safe closed-loop handling of a missing frame. The existing
fallback can still take substantial time; this change does not claim to shorten
the previously observed 1.49-second fallback gap.

## Vehicle installation

All committed carrot-nav work through `dabe4f1d67`, including the other tasks'
commits since vehicle HEAD `5e85695197`, was installed after fresh, valid Park,
standstill, zero-speed and inactive-control checks. The five tracked overlays
and three colliding untracked source files were hashed and backed up under
`/data/carrot-nav-update-backup-20261014-dabe4f1d67` before the fast-forward.
No unrelated workstation changes were committed. The vehicle rebooted, finished
its required native build and started the manager, cameras, model and DM.
Tracked vehicle files match the commit. The existing signed Jetson release
`f1838e2c54` and full-block compression OFF policy were preserved.

After startup preparation, a 60.011-second parked sample contains 1,200 model,
1,200 road-camera and 1,200 driver-state messages, all valid. All 1,199 adjacent
model frame-ID increments are one. Jetson RL Nav is active/ready in all 60
one-second status samples. Mean inference is **45.287 ms**, p99 **48.987 ms**,
maximum **61.166 ms**. Publication intervals average 50.009 ms, p99 55.841 ms,
maximum 65.758 ms. The smoothed frame-drop statistic retains startup history;
there are no observed frame-ID skips in this measured interval. This still does
not achieve the 40 ms goal or guarantee every inference completes within 50 ms.

A concurrent roughly 10-second CPU sample gives vehicle aggregate 70.4%,
busiest core 82.5%, and Jetson aggregate 11.4%, busiest core 22.9%.
The eGPU is absent, so the installed recovery code's actual GPU overhead and
fault recovery remain unmeasured. No injected faults were sent to live control.
The navigation status was `route` before reboot (no usable route); this check
does not establish route guidance or driving behavior.
