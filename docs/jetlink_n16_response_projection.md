# Candidate 3: negotiated N16 driving-output reply

This reduces a successful N16 USB reply from 18,580 to 2,066 float32 values.
It saves 66,056 payload bytes/frame (1.32112 MB/s at 20 Hz), without changing
model inputs, TensorRT execution, recurrent state, USB write scheduling or deadlines.

## Compatibility and validation

- Both comma and Jetson must advertise the exact `carrot_n16_driving_output_v1`
  descriptor during protocol-v2 HELLO. It pins the model SHA, complete ModelSpec
  SHA, little-endian float32 format and exact prefix length. Other models and
  protocol v3 keep their existing response format.
- The comma daemon separately advertises `carrot_n16_driving_output_ipc_v1` to
  modeld. Local request bit 0 remains reset; bit 1 requests the compact reply.
  Old local clients still send 0/1 and receive full replies. New clients talking
  to old daemons or old Jetsons also receive full replies.
- `SEND_RAW_PRED` keeps the full response. No synthetic zero feature outputs
  are inserted into the parsed result.
- USB request bit 5 selects projection, and response envelope bit 5 confirms it.
  A mismatched marker, frame ID, byte count, telemetry or nonfinite value fails
  through the existing link-error/fallback path. Returned arrays own their data.
- The host validates the **full** output and all next-state tensors and commits
  recurrent feedback before slicing. NaNs in omitted features or next state
  still fail. Projection must never replace these checks.

The selected Jetson package manifest is intentionally unchanged. Deploying only
this comma commit remains compatible but does not enable the reduction with an
old host. A host package built from these sources is required on Jetson too.
This commit does not upload/install that package or change update acceptance.

## Evidence and limits

An earlier isolated live-device prototype compared 2,400 selected frames per
condition, using real cameras plus simulated map/cruise inputs on a bench:

| Condition | Mean call ms | P99 ms | Maximum ms | Calls >50 ms |
|---|---:|---:|---:|---:|
| Baseline | 41.480 | 42.297 | 45.174 | 0/2400 |
| Response projection only | 39.155 | 39.965 | 42.961 | 0/2400 |

This is about 2.33 ms less call latency in that setup, **not** less GPU computation.
The final negotiated production implementation in this commit has additional
compatibility checks and was checked on PC, not reinstalled for this commit.
Do not treat the prototype timing as a fresh production benchmark or proof of
road safety. Confirmed full HUD/preview/auxiliary workload and longer mixed-version
onroad-equivalent testing remain necessary before deployment.

PC checks: 265 passed, 1 skipped across output projection, daemon timing, navigation,
output validation, hotplug, protocol compatibility, stateful protocol, app models,
daemon compatibility, v3 and resaction-session tests. New tests exercise the real
host/session, USB client, local IPC client, state queues and parser with a synthetic
CPU engine; they do not execute TensorRT. ONNX-dependent packed/block codec suites
could not collect in this PC environment because the `onnx` package is absent.

The intermittent 100+ ms USB write tail is a separate unresolved issue. This
change neither proves its cause nor re-enables asynchronous USB writes. The
64 KiB chunking experiment was slower and is not included.
