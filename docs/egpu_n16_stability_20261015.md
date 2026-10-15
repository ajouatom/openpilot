# NAV n16 model and eGPU failure containment

The user requested correcting the directly connected eGPU failures, required
normal driving-model output at 20 Hz, and then requested the newly published
NAV model. These are parked tests, not road or closed-loop validation.

## Model identity and scope

Upstream [afbf2e547e](https://github.com/commaai/openpilot/commit/afbf2e547e9319e2eb9f515e1e4330af63705cec)
is `n16 nav-wm model`. Compared with a03bb1d4 it changes only the compiled PKL
pointer. It does **not** update USB code or the tinygrad submodule.

- PKL: 822465506 bytes, SHA256 `52205044e136efb64a6a841c9b016cf35b1db81186dd0f941b62eee5ebdcde24`.
- Checkpoint: `2931d3d9-7c90-4988-a6af-654ed71e0af5/56320/96244b1f-6c4e-4adc-848d-eccd0b44b182/12800`.
- Runtime: tinygrad `d3f09c9bbd542fbfbe68c8569a1173550c408969`, archive
  `90970e01f2ba70d553294c0f9d5c6fe1678827f750705cab679a7b19b7bcb0b2`.
- [NAS manifest](https://upload.shind0.synology.me/models/comma4-big-nav-n16-96244b1f/manifest.json).

The full input/output specs and output slices match the prior RL model; only
the embedded checkpoint changes. This selects the new PKL on carrot-nav.
carrot-wip retains its own driving model. Shared failure-containment changes
belong on both maintained branches with single-parent commits. Internal/DM and
Jetson models, driving controls and signal-assistance policies are unchanged.
There is no matching new ONNX in this upstream commit; the package does not
mislabel the previous ONNX as n16.

Official LFS size/hash, local package and NAS copies were verified. The vehicle
downloaded the new HTTPS package, verified its hash/checkpoint, and reused it
successfully with network functions disabled. This download test alone is not
physical GPU validation.

## Failure mechanisms addressed

The three prior direct USB4-1/5Gbps incidents remain described in
[the investigation](s3_wifi_egpu_investigation_20261015.md). Their initial GPU or
bridge stall is still not established. USB autosuspend was off in all three
snapshots; blaming an intervening hub does not explain them.

The actual pinned runtime's `Compiled._wait_signal` supports a spin window and
`on_sleep`, but its USB `sleep` implementation is empty. In the isolated worker,
`egpu_usb_wait` supplies a 200 microsecond sleep after a 1 ms spin window for this
exact verified runtime hash. Runtime files, compiled kernels, signal-completion
checks, device-error checks and synchronization deadlines are preserved. Modeld
and its worker retain their original FIFO54/core7 placement; background recovery
retains its own low-priority placement. Other runtime hashes are untouched.

A synthetic never-completing signal executed through the actual installed
`Compiled._wait_signal` source on C4 kept its 100 ms timeout, while thread CPU
fell from 99.604 ms to 7.517 ms. This isolates the wait behavior; it is not a
reproduction of the original physical GPU/bridge stall.

Lowering the whole worker to normal scheduling produced about 18.9 Hz. FIFO4
on core7 produced about 19.7 Hz, and moving FIFO4 to cores4/5 produced 16.3 Hz.
These trials were rejected under the user's 20 Hz requirement and are not the
final implementation.

The parent now applies the deadline to the entire response, including fragmented
lines. The live inference limit is 150 ms (three model periods), instead of a
full second; the separate 20 s first-execution initialization budget is retained.
A failed worker is killed immediately, with at most 100 ms synchronous
reaping; an uninterruptible exit is reaped off the publication thread. Healthy
shutdown retains graceful termination. A retiring-worker guard and process-life
file lock prevent opening the same GPU while an old worker still owns it.
Ownership contention is transient, not a corrupt-model rejection.

Fallback discards the failed camera frame and receives a new synchronized pair.
A first controlled stopped-worker test exposed another delay: the internal model
had been loaded but not executed. It produced a 2.740 s publication gap and an
804 ms-old first internal result. Startup now executes its full warp/policy/readback
three times without shadow publication, then resets synthetic recurrent history.
This work runs before live eGPU publication, not during driving or recovery.
The startup probe uses one allocated main-camera buffer for both synthetic inputs;
it does not require draining the extra camera's startup backlog into a live pair.
An earlier paired-buffer warmup trial restarted modeld while that backlog was
unsynchronized and was corrected before final validation. Live pairing is unchanged.

A bounded watchdog emits at most three stalled-frame Python stack dumps per
worker after 75 ms, checked every 50 ms. The existing shared progress record remains the timeout evidence. This
helps distinguish later Python/USB/dispatch waits; it is not proof of the original
hardware fault.

## Parked verification

All vehicle actions require fresh Park, standstill, zero speed, inactive control
and Panda controlsAllowed false. Source files are backed up and hash checked.
Full restarts clear manager's cached Python modules; file replacement alone is
not treated as activation.

Initial n16 plus cooperative-wait test, before the standby warmup follow-up:

| 60.001 s window | Model | Driver monitoring |
| --- | ---: | ---: |
| Received valid frames | 1200 | 1200 |
| Missing/nonconsecutive frame IDs | 0 | 0 |
| Mean modelExecutionTime | 43.358 ms | 31.603 ms |
| p99 modelExecutionTime | 44.639 ms | 38.854 ms |
| Maximum producer publication gap | 59.340 ms | 87.402 ms |

These are nominal 20 Hz streams with real timing jitter, not a claim that every
individual interval is <=50 ms. The independent 20 s link check received 400/400
valid frames with direct 5Gbps USB and link_error_count remaining 0.

Controlled stopped-worker tests (SIGSTOP, Park/inactive) distinguish recovery
from the original physical fault. With warmup and the old one-second deadline,
the observed maximum model gap fell to 1.537 s and first internal capture age to
61.5 ms. With the final 150 ms deadline, the eGPU-active flag cleared after
505 ms; the first internal result arrived 718 ms after injection with 105.7 ms
capture age. The largest observed DM delivery gap was 374 ms. This is improved
containment, **not** seamless fault-time 20 Hz or guaranteed continued engagement.
No stale eGPU result is republished as current, and normal validity/control
checks remain responsible for disengagement during missing output.

Desktop verification: 298 focused carrot-nav tests and 257 carrot-wip tests pass, including the actual fallback
handler, fragmented response deadlines, delayed worker reaping/ownership,
synthetic-history reset, bounded camera warmup, output recovery and model pins.
Lint passes for the changed helpers/runtime tests/package builder.

Final continuous post-fault validation, with all final sources active:

| 300.001 s window | Model | Driver monitoring |
| --- | ---: | ---: |
| Received valid frames | 6000 | 6000 |
| Producer rate | 20.0002 Hz | 20.0002 Hz |
| Missing/nonconsecutive frame IDs | 0 | 0 |
| Mean modelExecutionTime | 43.551 ms | 31.927 ms |
| p99 modelExecutionTime | 45.313 ms | 38.978 ms |
| Maximum producer publication gap | 59.927 ms | 72.270 ms |

No invalid output or fallback occurred in that window. The active worker used
the n16 `52205044` artifact with FIFO54/core7; eGPU remained active and S3 remained
connected. The model's observed capture age p99/max was 98.5/121.6 ms. Observer
arrival gaps were up to 96.0/91.3 ms; the table uses producer timestamps instead.
An additional 20 s check received 400/400 valid results over direct 5Gbps USB.
Its link_error_count stayed **1 -> 1**, not zero: the time/cause of that cumulative
error is not established. Do not claim the physical USB link is proven fault-free.

This establishes normal 20 Hz in the parked test, not continuous 20 Hz through a
GPU failure or a guarantee against recurrence during driving. The initial physical
stall remains unresolved; the new stack snapshots support diagnosing recurrence.

Raw logs, frame timing, diagnostic snapshots and deployment records stay in
private `.analysis/scratch/2026-10-15-egpu-s3/`; model packaging evidence is in
private `.analysis/scratch/2026-10-15-nav-n16/`. No drive captures or credentials
belong in the published artifacts.
