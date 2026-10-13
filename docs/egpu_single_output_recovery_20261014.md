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

The extra checkpoint requires GPU copies and synchronization after each good
inference. The failed-frame path additionally downloads saved state to check
finiteness. Actual eGPU overhead and recovery latency are not measured because
the eGPU is currently disconnected. Parked Jetson verification cannot establish
eGPU behavior or safe closed-loop handling of a missing frame. The existing
fallback can still take substantial time; this change does not claim to shorten
the previously observed 1.49-second fallback gap.
