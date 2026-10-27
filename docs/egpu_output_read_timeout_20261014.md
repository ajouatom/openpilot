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

## Authorized reboot and warm initialization follow-up

The user then requested an actual reboot and initialization test. With fresh
Park, standstill, disabled controls and Panda controlsAllowed=false guards,
the comma was rebooted without changing code, model artifacts or settings.
Boot ID changed to 0547df25-747c-4c93-8cdd-2cf9876253f9. The eGPU initialized
successfully with the same NAV RL artifact. New modeld runtimeTiming context
(`backend=PrecompiledModelState`, `usbgpu=true`) confirms that the current
modeld module, not only the separately executed worker, is now loaded.

Two steady-state parked windows were verified against full rlogs; all ten
downloaded completed segment files matched remote SHA256 values. Initial
live-monitor misses were observer losses, not missing model publications.

| Test | Full-log model results | Inference mean / p99 / max | Maximum publication gap |
| --- | --- | --- | --- |
| 300.02 s after reboot | 6,002, consecutive and valid | 44.563 / 45.833 / 46.880 ms | 58.547 ms |
| 120.00 s after modeld-only restart | 2,399, consecutive and valid | 44.480 / 45.650 / 46.386 ms | 58.935 ms |

Camera, odometry, pose and driver-model messages remained valid in both
windows; all recorded pose input/sensor/posenet flags were true and no alert
was recorded. Startup/restart gaps are excluded from these steady-state
windows. A separate 120 s, 20 ms polling watcher saw no worker stage over
150 ms; this sampling cannot exclude every shorter stall.

Reboot restored operation but did not clean the physical link: the root-port
counter increased 2,550 -> 2,654 during the first window and 2,702 -> 2,737
during the second, with the same USB address and 5 Gbit/s enumeration. These
are root-port PORTLI errors, not per-eGPU inference-error counts. Available
memory remained below the background recovery guard. Successful ordinary
initialization does not demonstrate fault recovery or road reliability.

## External navigation correlation

Full logs around the original failure (boot 4915--4925 s) contain 191
carrotMan and 191 navInstructionCarrot messages, all valid. The navigation
instruction stream's largest gap is 199.525 ms. The route stays at the same
ID with 143 coordinates and two maneuvers; positions are nonzero and change.
Reported position-fix age spans 0.040--2.699 s. This local publication stream
does not prove phone packets or original GPS fixes arrive at 20 Hz.

The preceding minute includes navigation inactive/zero-position periods and
subsequent activation. Thus navigation was present near the failure and cannot
be categorically excluded by correlation alone. MapInput uses nonblocking
VisionIPC receive and accepts a bounded 256x256 image with route/freshness
checks. The recorded failure occurred later in worker output read, not in a
blocking wait for the navigation image. The last mapsd timing summary before
the stall reports map_valid=true, 143 route points, mean render 25.220 ms and
maximum 307.276 ms across 23 renders over 5.054 s. This includes a slow render
but does not explain a worker output-read stall by itself. No per-frame map-input trace or
controlled navigation ON/OFF comparison establishes causality here.

## User cable reseat comparison in progress

The user physically reseated the cables while retaining the hub/display
configuration. Kernel logs show hub/display/eGPU disconnect at boot 935.284--
935.309 s and re-enumeration at 951--953 s. The corresponding inference error
at approximately 935.452 s is `AMD failed with -1` in state checkpoint. This
is a user-induced disconnect test, not another unexplained spontaneous stall.
The background recovery status was now emitted (`stopped`, `probe_failed`,
one attempt), proving that this code path is loaded; that status alone does
not identify the preparation exception or demonstrate successful recovery.

After reseating, the eGPU was present but inactive. A guarded modeld restart
retained UsbGpuStartupFailed and therefore skipped eGPU initialization; it
must not be described as a new GPU-load failure. A subsequent 19.996 s
read-only sample with no eGPU inference increased the root-port counter
36,463 -> 43,770 (7,307 errors, about 365/s), with the same USB address.
This is not a matched-load inference performance comparison. It shows a severe
link problem even without eGPU model execution, so map inference alone cannot
explain the post-reseat link condition.

The user then disconnected only the display. USB inventory confirms that
the display disappeared while Genesys 05e3:0625 and the eGPU remained at the
same addresses. With eGPU still inactive, the first 20.001 s sample increased
62,557 -> 62,684 (127 errors, 6.35/s). A further 59.998 s sample increased
62,781 -> 63,096 (315 errors, 5.25/s). All adjacent sample intervals retained
the same address and nondecreasing counter. Thus the reduction persisted, but
link errors remained. Without a reconnect/reversal trial this does not prove
the display itself is defective; shared power, cable positioning and hub/link
state remain possible confounders. The user next offered to bypass the hub.

The direct connection was subsequently confirmed in sysfs: eGPU 3801:0001
at 4-1, 5 Gbit/s, with no intervening hub and no USB display. A 20.000 s
sample with eGPU inactive kept the link counter at 3 throughout, zero
additional errors. This narrows the practical comparison to the bypassed
hub/cable/contact configuration, without identifying an individual defective
part. A guarded reboot was then performed to clear the startup failure via
the normal startup procedure.

## Direct connection with inference and navigation

On new boot de694892-8c07-4280-a81d-f61e90ae1a33, normal boot validation and
eGPU initialization succeeded with the same code/model. A 120.001 s parked
window produced 2,399 consecutive valid model results; full rlogs match the
live observations. Inference mean/p99/max was 43.172/44.445/45.301 ms, maximum
publication gap 59.061 ms. Camera, odometry, pose and DM each had 2,399 valid
messages, with no alerts and all pose health flags true. USB stayed at bus 4
address 2, 5 Gbit/s, and the link counter stayed zero in all 120 samples.

Navigation initially lacked a fresh position, then recovered. An additional
60.001 s parked test recorded map preview reason=ready in all 60 samples and
12 mapsd timing summaries with map_valid=true. Nine summaries used stationary
position hold and three did not; this is not moving-GPS/route-steering testing.
Map render weighted mean/max was 17.111/61.132 ms. The live monitor recorded
1,200 consecutive valid model results, inference mean/p99/max
43.276/45.473/46.665 ms, maximum publication gap 59.227 ms. Camera, odometry,
pose and DM also each had 1,200 valid messages; no alerts, PID changes or link
error increments occurred. Thus a rendered navigation map and eGPU inference
coexisted without reproducing the failure in this bounded parked test.
Full rlogs reproduce the additional window's counts and timing, and all five
downloaded direct-test rlogs match remote SHA256 values. The final read-only
check at boot 473.353 s still shows Park/zero speed, controls inactive,
eGPU active, map ready and root-port counter zero.

The direct configuration improves the observed link behavior and achieves
20 Hz here, but does not meet the requested <=40 ms inference margin. These
tests do not prove which hub, connector, cable, PHY/power or link-state
interaction caused the original road stall. They do not certify road
reliability. The USB display remains physically disconnected for this test;
keeping it requires validating a suitable hub/cable configuration under the
same display-plus-eGPU load. No firmware, safety gate, memory guard or model
selection was changed to obtain this result. Vehicle code remains f1db63441d;
only investigation documentation is committed/pushed on carrot-nav.
