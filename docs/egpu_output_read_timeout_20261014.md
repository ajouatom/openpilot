# October 14 evening eGPU output-read timeout

Scope: read-only investigation of the user's latest drive; carrot-nav only.
The NAS route directory had no new October 14 upload. Full rlogs for route
000010c5--74b393d132 segments 79--81 were copied directly from the vehicle and
all three SHA256 values matched the vehicle originals. Full cereal decoding
includes model, camera, pose, DM, process, clock, control and kernel messages.
Raw logs, positions and diagnostic reports remain private.

## Failure and impact

At 18:14:34 KST, the parent reported `precompiled eGPU worker timed out`.
The selected artifact is NAV RL `comma-nav-rl-a03bb1d4-20cae5d0`, pickle
SHA256 20cae5d01d0c59352fa6bf23cb0bf2a9c6c7257f6b9997652f6ef64669e194f8,
with pinned runtime 90970e01f2ba70d5. The failure snapshot identifies worker
68563, inference 43376, stage `output_read`, unchanged for 1176.502 ms.
This is not a recorded nonfinite-output or shape-validation failure. The
single-completed-invalid-output allowance cannot handle an inference that
does not return a result. The artifact is not persistently rejected.

The immediately preceding full minute has 1,200 valid model results, all
consecutive, mean/p99/max inference 44.714/46.118/47.325 ms. Last complete
worker timing before the stall shows output read mean 13.294 ms and checkpoint
mean 4.726 ms. Thus the earlier checkpoint cost regression is not reappearing
as a sustained 16 Hz period in this window.

The last normal model publication is at boot 4920.728699 s, frame 96694.
The first fallback publication is at 4923.758596 s, a 3.029897 s gap. That
publication still carries frame 96695 and its original capture EOF at
4920.705709 s: it is 3.052887 s old and has valid=true. It is an internal-model
result computed after failure using the retained camera frame, not proof that
the failed eGPU output was accepted. A fresh capture's result follows at
4924.065823 s, another 307.227 ms later. Do not call the first fallback result
fresh merely because it has a new publication time or valid=true.

`commIssue` appears, followed by `locationdTemporaryError`; locationd and
modeld retain their process IDs. DM publication also pauses for 1.528718 s,
then has 434.679 and 830.094 ms gaps. Road-camera messages continue. The driver
cancelled before the logged timeout; the snapshots around failure show Drive,
about 24 km/h and gas pressed. This is not a parked-only failure.

The following minute runs the internal model at mean 25.444 ms. A later
read-only live check finds Park/zero speed, eGPU present but inactive. No
restart, USB reset, revalidation, inference injection or parameter change was
performed during this investigation.

## USB evidence and remaining uncertainty

The pinned adapter marks output_read immediately before output.numpy(). That
scope includes GPU completion/synchronization and download, so its timeout
does not distinguish an unfinished GPU job from a stalled USB transfer or
runtime/driver wait. The snapshot's D state and wchan do not provide a full
thread/syscall stack. No explicit eGPU disconnect, SuperSpeed reset or -71
event is retained at this failure time.

The retained USB reset at boot 4930.203756 s is USB2 device 3-1.3, about eight
seconds after the logged error. The eGPU is SuperSpeed 4-1.2; do not attribute
that later display-path reset to the original eGPU stall.

A subsequent 20-second read-only sample keeps eGPU bus 4 address 3 at 5 Gbit/s;
root-port link errors increase 4797 -> 4801. This confirms residual link
errors, not a contemporaneous link-error burst causing this timeout. The
earlier ~679 errors/s pre-power-cycle incident cannot be substituted for this
new drive's missing transfer trace. Cable, bridge, GPU execution and host
runtime causes remain unresolved.

Comma memory available is about 444 MiB immediately before/during the stall;
thermal status is green. CPU7 reaches 100% and DM remains runnable with large
publication gaps. The eGPU worker and modeld have higher realtime priority on
that core than DM. The pinned USB implementation contains polling paths.
This supports investigating failure-path CPU starvation but does not identify
the exact stalled syscall or prove which component originally failed.

## Runtime deployment mismatch

Files on disk are at f1db63441d, and the new worker checkpoint timings confirm
that the separately launched worker loaded the checkpoint correction. However,
the modeld exception has the old 3808b5733a call/handler line numbers (484/488),
not the corresponding current code lines. Its cloudlog context also retains
3808b5733a, and no carrot-egpu-recovery.json status exists after the failure.

The manager preimports PythonProcess modules, then forks modeld; restarting
only that child can reuse the parent's cached old module. These observations
strongly indicate that modeld retained the old fallback path while the worker
loaded the newer file. The earlier deployment note's presence of background
recovery files is not proof that background recovery was active. A fresh
manager process/reboot and runtime verification are needed to establish it.
Even then, the measured ~444 MiB available memory is below the recovery
feature's 2 GiB minimum; do not weaken that guard just to obtain a retry.

## Next corrections to validate

Keep the timeout classified separately from numerical-output failure. Capture
thread/syscall/USB completion evidence at a stall to distinguish GPU waiting
from USB waiting. Investigate a bounded fallback path that does not spend
seconds on the frame thread, protects DM scheduling, and reacquires a fresh
camera frame before publishing internal fallback. Neither merely extending
the timeout nor re-publishing the old prediction solves the recorded gap.
These are follow-up changes, not fixes deployed by this read-only analysis.
