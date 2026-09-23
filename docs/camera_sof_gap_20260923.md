# Ioniq 5 camera SOF gap and startup phase mismatch

## Incident and causal boundary

Route `00000594--abf5912e57--7`, Ioniq 5 PE C4, recorded commit
`1fbfe331567304f3546e191bccb0b0514b468c32` with `gitDirty=true`.
The later `dc2d92ca` change concerns CANFD stopping, not camera timing.
Times below are relative to the segment's first `carState`.

- Wide-camera BOOT_TS advanced by **102.044320 ms**. Its frame and request IDs
  remained consecutive: wide 8883/request 8922 to 8884/request 8923.
  Road and driver continued. The road/wide frame-ID difference changed from
  one to two for the remainder of the segment; there was no short catch-up
  interval that restored the old relationship.
- Model input advanced from road 8884 to 8886. `cameraOdometry` was invalid
  once at +24.175918 s; `livePose.inputsOK` was false for about 304 ms.
  `sensorsOK` and `posenetOK` remained true. CAN was valid throughout.
- `selfdriveState` entered softDisabling at +24.189127 s and returned to
  enabled at +24.561324 s. `enabled` and `active` remained true. The visible
  locationd warning lasted about two seconds because of alert display duration.
- Maximum model execution was 40.751 ms. Replaying all camera metadata through
  the actual pairing function, with either a 10 ms or 20 ms bound, produced
  the same 1,200 pairs and the same road-frame gap. Accepted-pair skew was at
  most 0.295228 ms. The pairing-bound change does not create this gap.

This establishes a wide-camera SOF-path gap followed by invalid model/pose
inputs. It does not establish a physical sensor failure. Camerad subscribes
to `V4L_EVENT_CAM_REQ_MGR_SOF_BOOT_TS`: the kernel reads boottime during SOF
handling, separately from the CSID hardware timestamp. IRQ/tasklet delays or
event loss cannot be excluded using BOOT_TS alone.

The relevant kernel is the AGNOS builder's recorded submodule
`eccd146599f2e2f159d951092642689bede91632`. See
[CSID timestamp acquisition](https://github.com/commaai/agnos-kernel-sdm845/blob/eccd146599f2e2f159d951092642689bede91632/drivers/media/platform/msm/camera/cam_isp/isp_hw_mgr/isp_hw/ife_csid_hw/cam_ife_csid_core.c#L2034)
and [IFE SOF processing](https://github.com/commaai/agnos-kernel-sdm845/blob/eccd146599f2e2f159d951092642689bede91632/drivers/media/platform/msm/camera/cam_isp/isp_hw_mgr/cam_ife_hw_mgr.c#L3804).
The source was checked; the incident device's entire binary set was not
independently hash-verified.

## UI and request-queue checks

Core6 averaged 89.6% total use; UI was SCHED_OTHER/nice19 and camerad remained
normal/nice0. UI is not restricted to idle CPU time: nice changes its weight
among normal tasks. Nevertheless, the log does not prove UI caused this gap.
UI draw time immediately before the incident peaked at 46.883 ms, versus
60.123 ms elsewhere in the segment. Earlier `00000f90--96d7dcd525--4` showed
the same kind of wide-camera gap while UI was on a different core.

The camera queue has 18 slots. Under the successful enqueue path, wide request
8923 was submitted when request 8905 was handled, over 908 ms before the
observed request-8923 SOF. This is a source/log inference, not a kernel apply
trace, but it does not support a simple one-frame UI stall starving the
request queue. Exposure commands remained inside sensor limits.

The persistent Panda `registerDivergent` flag is not evidence of a camera
timer fault. Captured addresses `0x40020010` and `0x40020028` are audio DMA
stream control registers on STM32H7, not TIM1 FSIN registers. The audio code
tracks hardware-changing DMA target bits; correcting that separate firmware
diagnostic is outside this camera change. It must not be described as a
newly observed camera failure at the incident timestamp.

## Confirmed startup mismatch and correction

The same route's startup capture reports `camera first frame sync timed out`
at raw frame 41, with driver `staggered=1`. A later startup and another C4
capture repeat it.

Upstream [openpilot change #37628](https://github.com/commaai/openpilot/commit/1777d548bf0023297d8a685ff8dc6917a1e1db44)
introduced a 25 ms driver-camera offset and updated Panda together.
Its companion [Panda change #2307](https://github.com/commaai/panda/commit/c10b82f8ff03c7d677a9420c21c0c50126a5071e)
moves driver FSIN to TIM8, triggered halfway through TIM1's period.
This repository contains the camerad expectation but its bundled
`panda/board/drivers/clock_source.h` still drives all FSIN channels from TIM1
in phase. The build uses this vendored Panda tree; pandad compares the device
signature with the bundled firmware before starting.

Five full logs corroborate the current phase contract: Ioniq 5 and K9 C4
OS04C10 cameras have driver/road nearest-SOF median differences of
0.013--0.037 ms. EV9 C3-family OX03C10 has a 0.026 ms median, with larger
kernel timestamp jitter. These are not observations of 25 ms staggering.

Driver `staggered_sof` is therefore set to false. The 0.2 ms startup tolerance,
timeout fallback, sensor handling, model pairing, validity checks, process
placement and scheduling priorities are unchanged. Camera/onroad hardware
tests now require all three cameras to be in phase. Future physical
staggering requires a coordinated firmware/camerad/test change.

The startup synchronizer stops checking after it succeeds or times out.
Consequently this fix addresses the unnecessary startup timeout and initial
frame-ID alignment; it is **not a demonstrated fix for the SOF gap seven
minutes later**.

## Passive diagnostics and follow-up

`camera SOF timing` records long BOOT_TS intervals and late userspace event
handling, including raw frame/request IDs, SOF status, receive time, previous
validated IDs, last requeue time and the number of suppressed reports.
The diagnostic thresholds are 75 ms, with at most one message per second per
camera. The monitor has fixed-size state, no per-frame allocation and no
effect on frame acceptance, requeue decisions, warning grace or validity.
It uses the existing event subscription; no extra IRQ/event stream is added.

A large SOF delta with a small event age points to the sensor/kernel SOF path;
regular SOFs with a large event age point to late userspace handling. These
fields still cannot distinguish a missing sensor exposure from a kernel
SOF/IRQ problem. That requires correlated CSID hardware timestamps and
IRQ/tasklet observations from the same frame, plus request-apply/fence timing.
Adding a second V4L event subscription would first require filtering by
`ev.id`; passing both streams to the current frame handler would process
frames twice.

On the target, check that startup logs report synchronization without the
raw-frame-41 fallback, then compare repeated C3/C4 drives separately. Inspect
the new timing message at any recurrence alongside camera/model/pose logs.
No vehicle startup, driving improvement or full Linux camerad build is
established by the desktop checks below.

## Desktop validation

- Four compiled C++ diagnostic cases passed 216 assertions: normal cadence,
  the recorded 102.044 ms SOF interval versus late userspace arrival, bounded
  repeated logging, and repeated/backward SOF timestamps.
- A temporary host harness compiled the actual `syncFirstFrame` method and
  declarations, loading the current flags from `hw.h`. Six cases passed 62
  assertions: in-phase startup, reproduction of the old raw-frame-41 timeout,
  rejection of real 25 ms misalignment, the unchanged 0.2 ms bound, a disabled
  driver camera, and unchanged offsets after a later frame gap. Kernel/camera
  dependencies and logging were stubbed; this is not a full camerad run.
- All 15 existing model camera-pairing tests passed. Windows ran them with
  `--noconftest -o addopts=` because the repository-wide fixture requires the
  Linux Params extension. Ruff, Python parsing and the diff whitespace check
  passed for the changed files.
- The existing Linux CI builds its normal desktop targets. Camerad itself
  remains a device-only target (`arch == larch64`), so the portable diagnostic
  test has a separate SConscript loaded on desktop and device builds with
  extras enabled. CI runs that test before the model regression suite. The
  initial CI attempt exposed the device-only build guard; the test target was
  then moved out of that guard. A desktop CI pass is not an ARM camerad build.

The portable host compiler was Zig 0.13.0/Clang with the repository's Catch2
2.13.10 headers. No firmware, device parameters or live vehicle processes
were changed during this investigation.

Docs-Not-Needed: Internal camera startup compatibility and passive diagnostics;
no setting or user workflow changes.
