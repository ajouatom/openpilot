# Map-only input staging experiment (2026-10-23)

Base: `809616e8167bd8df75956daadba79a564de72875`, `carrot-nav-pv5`.

## Result

The actual Orin CPU input-preparation microbenchmark improved by approximately
0.14 ms per frame for the installed engine's FP32 navigation input. This is a
small, repeatable improvement in this isolated test, not a demonstrated reduction
in camera-to-result latency, GPU computation, USB tail latency, or missed frames.

| Synthetic map | Baseline mean ms | Candidate mean ms | Saved ms | Baseline P99 ms | Candidate P99 ms |
|---|---:|---:|---:|---:|---:|
| Dense byte values | 0.9321 | 0.7944 | 0.1377 | 0.9714 | 0.8180 |
| Sparse lane-like bytes | 0.9268 | 0.7918 | 0.1350 | 0.9615 | 0.8149 |
| Expired, all zero | 0.9306 | 0.7901 | 0.1405 | 0.9697 | 0.8132 |

The cached N16 TensorRT engine was inspected without creating an execution
context: `new_img` is HALF, `nav_map` and all other context inputs are FLOAT.
The table uses these actual non-state input dtypes. The synthetic state arrays
were FLOAT in that arm; they are initialized but not advanced or timed by this
preparation-only benchmark. No TensorRT inference was executed.

FP16 context was also measured as a compatibility case, saving 0.242–0.333 ms.
Those larger numbers do **not** describe the installed engine's map input.

If an entire model call took 40 ms and the isolated saving survived unchanged,
0.14 ms would be about 0.35%. This is arithmetic, not an end-to-end measurement.
At 20 Hz, it represents about 2.8 ms less CPU work per second, not a major CPU
capacity increase. GPU computation and wire byte counts are unchanged.

## Change and scope

The previous implementation from `433acbb50c2998420f06d02253e44c4f5e353a7d`
combined native image conversion and direct map staging. Its approximately
0.5 ms combined improvement must not be attributed entirely to map staging.

This experiment adds `CARROT_TRT_DIRECT_MAP=1`, default **off**, to the host
session. For negotiated compact byte maps it validates scalar inputs and scatters
directly into the existing engine-owned bindings. It avoids the intermediate
float32 map expansion, byte serialization and concatenated packed-context copy.
Image conversion, raw float fallback, protocol, output projection, GPU recurrence,
finite-output checks, update validation and all vehicle controls are unchanged.
No map is cached across frames. Expired/zero maps overwrite the destination.

Binding identity, address, shape, dtype, alignment and writability are checked.
All scalar values are validated before any input write; byte maps are inherently
finite. Noncanonical maps (fractional, out-of-range or negative zero) still use
the original raw path. Receive-buffer views are consumed synchronously before
engine execution and are not retained.

## Method and validation

- Actual Orin, separate bounded CPU benchmark, CUDA-pinned host buffers.
- Existing Jetlink/HUD services kept running; no device installation, service
  restart, power/clock policy change, USB reconfiguration or onroad simulation.
- Unchanged FP16 image conversion included in both measured paths.
- 100 warm-up calls per arm; three ABBA rounds with 200 calls/block: 1,200
  measured calls per arm, map pattern and dtype, 14,400 total measured calls.
- Deterministic synthetic images, changing scalar inputs and eight map buffers.
  CPU wall and thread time recorded. Task ran at nice 10, no affinity override.
- All input bytes matched in 48 actual-device prechecks across the six cases.
  PC tests also cover changing/expired maps, buffer reuse, invalid length and
  non-finite input, binding replacement, raw fallback and projected/full replies.
- 96 focused PC tests passed, including GPU-state reset/validation routing and
  the existing deferred-CUDA state tests. These are not actual GPU output tests.
- Broader codec/state-buffer test collection was unavailable in the PC venv
  because its separate `onnx` dependency is absent; those tests are not claimed.

Artifacts: workspace `jetson-nvme/direct-map-20261023/`, including `bench.py`,
`orin-result.json`, `engine-info.json` and private device status receipts.

An additional simultaneous TensorRT inference trial was rejected by automatic
approval review because a second GPU workload could compete with the running
service. It was not executed or retried indirectly. Accordingly, there is no
fresh driving-output equality or end-to-end P99 result for this candidate.

## Decision

Retain the implementation as a default-off experiment. The approximately
0.14 ms input-preparation saving is real under the measured conditions, but is
too small to justify claiming a material 20 Hz or USB-tail improvement. Before
enabling it in a release, use an isolated bench inference session with no
competing production model, compare identical recurrent sequences and outputs,
and measure full request/result P99 and 50 ms exceedances. No new package is
selected by this change.

Cleanup confirmed: the temporary Orin directory and this task's phone/USB
diagnostic tunnels were removed. Comma remained offroad and Jetlink `ready`;
the Jetson service remained active with the same PID and installed release.
