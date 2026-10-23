# Image-only native input conversion trial (2026-10-23)

Base: `fc02df9c0d17dc3e8c2f6d53e17dd46ebf042888`, carrot-nav-pv5.

## Result

On the connected Orin, replacing only the existing NumPy uint8-to-FP16 image
lookup with a small native C loop saved approximately **0.34 ms/frame** of CPU
preparation. Map direct staging was disabled in both arms; existing map decode,
scalar validation and copies remained unchanged.

| Synthetic image | Scope | Baseline mean ms | Native mean ms | Baseline P99 ms | Native P99 ms |
|---|---|---:|---:|---:|---:|
| Random bytes | Image conversion only | 0.6981 | 0.3605 | 0.7353 | 0.3781 |
| Random bytes | Full input preparation | 0.9249 | 0.5898 | 0.9650 | 0.6235 |
| Repeated 0–255 ramp | Image conversion only | 0.6966 | 0.3603 | 0.7304 | 0.3767 |
| Repeated 0–255 ramp | Full input preparation | 0.9227 | 0.5869 | 0.9613 | 0.6214 |
| Zero | Image conversion only | 0.6973 | 0.3606 | 0.7325 | 0.3814 |
| Zero | Full input preparation | 0.9332 | 0.5929 | 0.9744 | 0.6293 |

This is about 48% less image-conversion time and 36% less full preparation time
for the random-byte case. It is **not** a 36% improvement in total inference.
As arithmetic only, a preserved 0.34 ms saving in a 40 ms full call would be
about 0.85%; this trial did not measure camera/IPC/USB/GPU end-to-end latency.
The prior map-only 0.14 ms result must not simply be added as a verified combined
result. Combined candidates require a separate test.

## Implementation and correctness

`CARROT_TRT_NATIVE_IMAGE=1` enables the experiment; the default remains off.
The native helper is compiled into a private temporary directory and self-tested
once during model loading. A missing compiler, compile/load failure or failed
self-test retains the original NumPy implementation. There is no compiler or
process creation in the inference loop.

The helper reads a uint8 source and writes the exact uint16 FP16 bit pattern
from the same 256-value table used by the existing conversion. It performs no
floating-point arithmetic, interpolation, normalization or lossy quantization.
Only the `new_img` FP16 branch changes; FP32 inputs use the original assignment.

The Python boundary rejects incorrect dtype/count, strided buffers, unaligned
or read-only destinations and overlapping source/destination memory before
calling C. The table is read-only and the function closure owns the loaded
library and table. Source/destination references remain alive during the call.
The C function does not retain pointers. The owning session still stages inputs
synchronously before invoking the engine.

No model, control calculation, recurrent state policy, output validation, image
warp, map expiry, USB framing/deadline, HUD behavior or power policy changes.
The existing opt-in map conversion remains independently selectable and was
off for this comparison.

## CPU-only test method and limits

- Confirmed comma offroad and Jetlink ready before testing.
- Actual Orin CPU; CUDA-pinned host buffers with N16 engine input dtypes:
  `new_img`/`state_img_q` FP16, all other inputs FP32.
- Existing runtime services remained active; no second GPU inference workload,
  service restart, runtime installation, model switch or onroad simulation.
- Synthetic images/maps/scalars, no actual-camera or driving-model output trial.
- For each of six cases, 100 warm-ups per arm followed by three ABBA rounds of
  200 calls/block: 1,200 measured calls/arm, 14,400 calls in total.
- CPU frequency snapshots before/after both showed 1,728 MHz on all six cores;
  no clock/power/affinity policy was changed. The test process used nice 10.
- Actual ARM binary: all input tensor bytes matched in 24 preparation pairs,
  including blank/expired maps; separate source lengths 0/1/7/16/31/255/256/257
  and 393,216 checked exact values and untouched destination guard regions.
- 124 focused PC tests passed. The two session tests were then strengthened to
  require the actual FP16 native callback, and the 28 native-image tests passed
  again. Coverage includes every source byte, guard regions, invalid buffers,
  image source preservation, full/compact protocol replies, raw-map fallback,
  load failure fallback and existing device-state/overlap regression tests.
- Actual-device evidence: workspace `jetson-nvme/image-cast-20261023/bench.py`
  and `orin-result.json`; per-block wall/thread timings are retained there.

## Driving-impact assessment

The tested model input values are bit-identical and there is no intentional
change to driving decisions or temporal state. The improvement is in CPU
staging, not GPU computation. This establishes the conversion's numerical
equivalence under the tested conditions, **not** complete road-driving safety.

## Isolated TensorRT acceptance follow-up

The owner explicitly confirmed a detached bench setup and approved pausing only
the operational Jetlink service for at most three minutes with automatic
restoration. A separate systemd supervisor enforced a 155-second deadline and
an independent service-start cleanup action. The earlier simultaneous-GPU trial
was not run. The operational service and test engine did not run concurrently.

The cached N16 TensorRT 10.3 Orin engine was loaded with GPU recurrence, finite
scan and CUDA graph enabled. Map direct staging remained off. The paired test
changed only the image converter, with synthetic image/map/scalar inputs and
identical initial state. No engine rebuild, clock/power/affinity change, vehicle
control, camera or USB inference request was used.

- All non-state input bytes, full driving output bytes and actual GPU recurrent
  input-state bytes matched for **32 frames per arm**, including reset at frames
  0 and 16 and alternating present/expired-zero map data. GPU-state readbacks
  were confined to correctness checks, outside timing.
- Non-finite scalar rejection/reset and recovery matched the initial baseline.
- Injecting a non-finite value into the private engine's GPU recurrent output
  exercised the real finite scan and feedback rejection/reset; recovery again
  matched the initial baseline. No production engine state was injected.
- Two ABBA rounds, 120 measured frames/block, 20 warm-up frames/block and 20 Hz
  pacing yielded **480 measured frames per arm**. The engine column includes
  transfers, execution, synchronization and feedback validation; it is not a
  pure GPU kernel timer.

| Scope | Baseline mean ms | Native mean ms | Baseline P99 ms | Native P99 ms | Baseline max ms | Native max ms |
|---|---:|---:|---:|---:|---:|---:|
| Input preparation | 0.9789 | 0.6347 | 1.1338 | 0.7735 | 1.2147 | 0.8679 |
| Engine and validation | 24.1537 | 24.1371 | 24.3495 | 24.3165 | 24.5198 | 25.9314 |
| Preparation + engine + validation | 25.1326 | 24.7718 | 25.4741 | 25.0612 | 25.6563 | 26.5660 |

Preparation saved **0.3442 ms/frame**; the measured total saved **0.3608 ms
(1.44%)** on average. The total P99 improved by 0.4130 ms, but its maximum was
0.9097 ms higher in the candidate. Neither arm exceeded 50 ms (0/480 each).
This short synthetic run supports the preparation improvement, not a guarantee
of worst-case latency improvement. It does not measure camera/IPC/USB end-to-end
P99 or resolve the previously observed 100+ ms USB tail.

The trial exited successfully and the service was restored after **57.77 s**.
The result collector initially rejected TensorRT's textual stdout prefix; the
valid final JSON record was subsequently recovered without rerunning the trial.
The restored service was active with the original release and model ready;
comma returned to Jetlink ready/offroad. No runtime source or selected package
was replaced. Evidence is retained privately under workspace
`jetson-nvme/native-image-acceptance-20261023/` (script, per-frame results,
supervisor and restoration checks). The final focused PC suite passed **126
tests**, including the strengthened FP16 session and actual model-initialization
checks. Expanded CI caught a missing `os` import in the shared environment-option
initialization path. This startup error was fixed; new tests cover initialization
both with the native converter enabled and with the default environment. The
native conversion implementation and the measured TensorRT loop did not change.

No numerical or reset-policy regression was observed in these tests. This is
not a complete road-driving safety assessment or long-duration loaded transport
test. The change is published **default-off** for further controlled evaluation;
enable only with `CARROT_TRT_NATIVE_IMAGE=1` in a runtime containing this source.
No release package selection or production installation is included.
