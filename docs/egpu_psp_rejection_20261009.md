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
The recorded device traceback also matches the new narrow classifier.
Device recovery and sustained operation validation are pending.

Raw captures remain local under `.analysis/scratch/2026-10-09-egpu-live/` until
archived. Do not publish device settings or raw tmux captures.
