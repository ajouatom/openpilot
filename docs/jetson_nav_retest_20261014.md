# Jetson NAV parked retest and eGPU incident

Scope is carrot-nav only, as explicitly requested in this conversation. The
user authorized connecting today and the existing approval covers follow-up
20 Hz corrections and parked application. Fresh valid/alive Park, standstill,
zero speed and inactive/disabled control checks preceded service changes and
reboots. Private source recordings and device diagnostics are not published.

## Actual installed code matters

The vehicle started at `5e85695197` with clean tracked files. Its reflog contains
four resets to HEAD on October 13 at 19:59--20:01; the operator is not established.
These removed the tracked client/manifest overlays. The old manifest then
selected host `955a6f2d39` on boot. Existing untracked codec files alone did not
activate the new client. The known overlays were restored with old-hash checks
and backups; the signed host `f1838e2c54` was restored and its cached engine reused.

Restarting only jetlinkd was also insufficient. PythonProcess.prepare imports
the module in the manager parent; forked children inherit that module cache.
Even after a confirmed new child PID, runtime timing lacked `request_bytes`,
while a fresh standalone interpreter saw the updated code. Rebooting the comma
loaded the updated client and produced actual request-byte measurements. A
peer advertising a codec is not proof that the active client used it. The
October 13 departure results therefore did not establish LZ4 performance.

## Compression comparison

Before reloading the parent, a stable 60-second sample produced 1,181 valid
model outputs, 1,200 valid DM outputs and 19 skipped camera frames: 19.68 Hz.
After a full reboot, the actual LZ4 client produced 1,094 valid model outputs
in 60.010 seconds (18.23 Hz), 1,200 valid DM outputs and 107 skipped frames.
The external model stayed active/ready throughout this latter sample.

Actual compressed request size averaged 358,293 bytes. USB-send timing includes
encoding: it rose from 7.06 to 10.83 ms; jetlinkd thread CPU rose from 5.67 to
11.05 ms/frame. Model loop mean was 54.85 ms. Reduced bytes did not compensate
for encoding cost. The client now requires explicit `allow_block_compression`
opt-in; its default retains exact integer-map packing without full LZ4 encoding.
The server continues to support both wire formats. No pixel quantization,
model/engine change or relaxation of validity checks is involved.

After applying client `008b942c26` and rebooting the comma again, a stable
120.000-second camera/USB/HUD sample yielded **2,400 valid model outputs,
2,400 valid DM outputs and 2,400 camera messages**. Every successive model
frame ID advanced by one (2,399 transitions). External NAV was active and
ready in all 120 status samples. Mean publication interval was 49.995 ms,
p99 55.568 ms and maximum 59.328 ms. Model execution mean was 44.994 ms,
p99 48.562 ms and maximum 53.782 ms. This establishes sustained 20 Hz in this
parked sample, not a per-frame hard 50 ms bound. The decaying frameDropPerc
statistic still contained startup history; direct frame IDs had no skips.

Requests were consistently 458,996 bytes, proving map packing with full-block
compression disabled. Send/encoding mean was 5.232 ms; jetlinkd thread CPU
returned to 5.706 ms/frame. Model loop mean, including camera wait, was
49.949 ms. Host total mean was 28.719 ms. The separate host/HUD services had
zero restarts and remained running.
The final Jetson telemetry reported 69.2 C and thermal health OK.

Concurrent ten-second CPU samples (busy fraction, not one-core process CPU):

| Device | Aggregate % | Per-core % |
| --- | ---: | --- |
| Comma | 72.4 | 79.8, 78.5, 86.3, 85.4, 58.2, 69.2, 69.4, 52.0 |
| Jetson | 11.4 | 6.2, 5.4, 23.1, 21.7, 5.2, 6.6 |

Comma remains the busier device. The signal observer consumed 42.6% of one
core in this sample; it and driver monitoring were left enabled and unchanged.
Jetson's HUD encoder consumed 30.7% of one core, HUD 21.7%, inference service
11.7%. Aggregate CPU is not proof of serial-path latency margin. Full-block
compression stays opt-in, and the tiny GPU state-copy gain stays experimental.

There was **no active navigation route** (`carrot_nav_status.reason=route`),
and HUD map received count was zero. Actual routed map rendering, route turns,
road response and hot-device performance remain unvalidated. The user was
asked to supply an external destination for that additional measurement.
The NAV model remains active on the parked vehicle; eGPU rejection was not
cleared. Vehicle HEAD remains `5e85695197` with four backed-up tracked overlays
(client, daemon diagnostics, startup gate, signed host manifest). Host remains
the already signed `f1838e2c54`; changing the comma client required no new host
package or engine rebuild. The installed client bytes match the local tested
client (SHA256 `3b2c32b8d378979dceccc63a1c712914bac334be111e529317c9024fc90a1a84`).

Validation of `008b942c26`: 32 focused desktop tests passed, three native-LZ4
tests skipped on Windows. Linux CI passed 752 integration tests (11 skips),
119 navigation/cluster tests and 11 protected-storage tests. Windows installer
and eGPU compatibility CI passed. A supplementary Windows socket suite had
33 passes and nine failures because Windows sockets lack `sendmsg`; the Linux
integration run includes those tests and passed. Ruff and whitespace checks
passed. Compatibility CI is not reproduction of the invalid eGPU output.

## GPU state-copy experiment

The standalone benchmark now supports `/tmp` placement with an installed
PYTHONPATH and Jetson's Python 3.10 (streaming SHA256 instead of file_digest).
Production inference was stopped for this isolated test. Existing TensorRT
10.3.0 Orin engine SHA256 is
`7ba226dbd894833212826ee381d7cba1711b63764092bd40b5c1298f8bbb0ff1`.
Each A/B/A variant ran 150 synthetic frames with resets every 25 frames.

| Variant | Input ms | Engine including copies ms | Feedback ms | Total mean ms | Total p99 ms |
| --- | ---: | ---: | ---: | ---: | ---: |
| Shared host staging, before | 0.908 | 24.701 | 2.034 | 27.643 | 27.955 |
| GPU-to-GPU previous state | 0.928 | 24.371 | 2.053 | 27.353 | 27.537 |
| Shared host staging, after | 0.924 | 24.758 | 2.030 | 27.711 | 27.864 |

All six outputs matched bit-for-bit on every frame. Each output's injected
nonfinite value cleared state and reset recovery matched. CUDA graphs were
captured for all variants. The roughly 0.3 ms gain does not by itself solve the
full pipeline's deadline, so this candidate remains an offline experiment.

## October 13 eGPU failure

At 12:18:55.588 KST, the NAV RL eGPU worker rejected an inference result with
`ValueError: invalid precompiled model output`, then fell back to the internal
model. The active artifact was `comma-nav-rl-a03bb1d4-20cae5d0`, SHA256
`20cae5d01d0c59352fa6bf23cb0bf2a9c6c7257f6b9997652f6ef64669e194f8`.
Its persistent `rejected` marker and failure record still existed today. This
was automatic invalid-output rejection, not a manual eGPU disable for Jetson.

The check combines output element count and finiteness. NaN/Inf is plausible,
but the rejected tensor was not retained, so the exact failed condition and
underlying cause are unproven. Logged upstream inputs were finite; they do not
reconstruct actual camera/map tensors or recurrent state. Retained kernel logs
show no eGPU USB disconnect/reset around the event. A display-port reset much
later is unrelated evidence. Nearby recovered SPI errors have no established
causal link to the invalid model output.

Model publication paused for 1.491 seconds during fallback, with temporary
soft-disable alerts. Execution means before/after were 39.53/24.85 ms. Sixteen
archived source files were reverified against their recorded SHA256 hashes.
The eGPU is absent from today's USB inventory, so no live eGPU reproduction
was possible. Its rejection marker remains intact. A future reproduction should
record bounded output shape/nonfinite counts and input/state diagnostics while
preserving fallback and rejection checks.

## Follow-up with external navigation connected

The user connected external navigation and requested another measurement.
This follow-up was read-only: no process restart, code/settings change or
relaxation of GPS freshness. The same vehicle boot/client and signed host ran.

A 120.000-second parked sample produced 2,399 valid model outputs and 2,400
valid camera and DM messages: 19.99 Hz with one skipped camera frame (2,397
unit frame-ID steps and one step of two). NAV inference remained active/ready
in all 120 status samples. Mean publication interval was 50.016 ms, p99
56.380 ms, maximum 70.273 ms. Model execution mean was 45.170 ms, p99
49.511 ms, maximum 66.252 ms. Thus this run is approximately 20 Hz with an
observed drop, not the earlier no-drop result.

Separate 20-second runtime-timing sample:

| Stage | Mean ms | Maximum ms |
| --- | ---: | ---: |
| Jetson GPU execution including memory copies | 25.498 | 27.431 |
| Jetson total request processing | 28.840 | 30.889 |
| Comma model inference including transport | 45.143 | 52.839 |
| Full model loop including camera wait/postprocessing | 49.931 | 56.951 |

These are nested scopes, not values to add together. Requests stayed at
458,996 bytes. Concurrent aggregate CPU was 76.4% on comma (busiest core
89.5%) and 11.9% on Jetson (busiest core 25.2%).

The display path received 960x540 external map images: the host counter rose
from 385 to 2,441, with zero decode drops, stalled=false and a final image age
of 92 ms. This verifies transport/decoder state, not physical panel appearance.
The model's separate map renderer had a 144-point route, but its sampled
reason was `ready` only four times and `position` 116 times. Timing also
captured a briefly valid render followed by invalid-position intervals.

Read-only position diagnostics explain the distinction. External vehicle
metadata remained at sequence 9 throughout a 20-second probe; its original
receipt age grew from 73.3 to 92.0 seconds despite a live/valid carrotNavi
heartbeat. The stored external coordinates were nonzero, but the fused output
expired to zero. gpsLocationExternal messages were arriving without hasFix;
the other GPS stream was unavailable. The exact upstream reason for stopped
position updates (sender, GPS acquisition or transport) is not established.
Repeated heartbeats must not renew an old position fix. This result does not
validate continuous model-map rendering load or routed driving; new actual
position updates are needed for that remaining measurement.

All raw route/position diagnostics remain private. Only this summary is
published to carrot-nav.
