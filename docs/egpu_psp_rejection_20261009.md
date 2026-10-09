# eGPU PSP startup timeout and persistent rejection (2026-10-09)

A parked C4 on carrot-wip recognized its eGPU USB bridge at SuperSpeed, but
UsbGpuCompiled/Active were false. A previous boot's Cinque v3 load failed in
AMD PSP firmware initialization with `TimeoutError: BL not ready` (10 seconds).
The isolated worker serialized the traceback; the parent raised RuntimeError,
so the TimeoutError type check missed it and permanently rejected the model.
Following reboots skipped that artifact instead of retrying initialization.
This establishes the persistent software block, not the cause of the initial
GPU firmware readiness failure.

`precompiled_model.py` now recognizes that specific terminal PSP timeout during
load/boot validation as a device startup failure. It removes no validity checks
and adds no same-session retry. Historical rejection can recover only with a
matching pickle hash, a recorded load/boot-validation failure and a different,
known boot ID. Recovery revalidates model/runtime hashes and invalidates any old
smoke-test receipt. Unknown errors, artifact corruption and inference errors
remain subject to the existing rejection policy. Cinque v3 remains selected.

Regression coverage includes worker ERROR serialization, boot validation,
previous/same/missing boot identity, hash mismatch, corrupted artifact repair,
invalid/missing diagnostics, inference phase and misleading timeout strings.
All 104 focused model/runtime/build/startup tests pass on desktop.
The recorded device traceback also matches the new narrow classifier.
The device was updated from 6eeb19999f to 1eebe42689 while live CAN reported
Park, zero speed, standstill and disabled/inactive control. The normal manager
reboot path was used once. After a transient startup DNS wait, normal delivery
revalidated the artifacts and removed the historical rejection. C4 boot smoke
testing succeeded with the pinned Cinque v3 checkpoint (load 23.46 seconds),
and modeld loaded in 23.8 seconds. UsbGpuActive became true, first model output
was published and the existing delayed USB HUD startup completed. No model
cache deletion, GPU power reset, timeout increase or validation bypass was used.
This recovery does not establish why the original GPU PSP initialization failed.

A subsequent 50.002-second parked observation received 1,000 modelV2 and 1,000
cameraOdometry messages, all valid. All 50 one-second samples retained
UsbGpuActive=1 with no startup-failed flag. Mean/max model execution was
40.57/43.06 ms; the largest subscriber receive interval was 75.19 ms (not a
sensor frame-gap measurement). Car state and selfdrive state remained valid,
Park/zero speed and disabled/inactive. This is a short parked recovery check,
not sustained driving validation or proof that all GPU stalls are fixed.

Raw captures remain local under `.analysis/archive/2026-10-09-egpu-live/`.
Do not publish device settings or raw tmux captures.
