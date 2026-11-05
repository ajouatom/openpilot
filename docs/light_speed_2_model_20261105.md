# Light speed 2 model integration, 2026-11-05

Official source: commaai/openpilot b92fc4828664f8767bce5e92f1630ed319cbe509
(PR39236). Preserve N16 Nav, Flip Flop and Light speed; add Light speed 2.
N16 remains the default and internal CD210/DM are independent.

## Verified model identities

- Official PKL: 767228907 bytes, SHA256 b4dead3ee091d66ab002a6b8e716c26f4f79e3c931e4be5f48e017ba92f6e960.
- Official ONNX: 768749892 bytes, SHA256 909a140019050a9d7f00db22dd4f4085928d2af9474a82244672ddab964f95a8.
- Hugging Face path: ea2f516f-f758-42be-a245-913bb818a64f/32768/big_driving_supercombo.onnx.
- Both metadata checkpoints: f22e06e5-3b3f-48d2-a6b9-2f32dc20e707/56320/ea2f516f-f758-42be-a245-913bb818a64f.
- Isolated tinygrad ec91b74b5b6b0fc924b72d3f27689679ca2438b9, deterministic runtime SHA256 96c6ecc92fe5085096cddb4a0683e10d5ed5c8693184af301cd5f21753acf2dd.

Full official download hashes, ONNX checker, inert PKL metadata and ONNX input/output
shape/slice/checkpoint comparison pass. No nav_map/navigation inputs: NOO unsupported.
Matching metadata is not proof of numerical equality between AMD and TensorRT.

## Runtime compatibility

Keep each older model/runtime pair. ec91b74 retains persistent-buffer serialization,
byte-addressed buffers, captured transfer API and aliased next-state outputs. Its
USB sleep remains a no-op; enable the existing cooperative wait only for this exact
runtime hash. USB link error state/reap guards remain intact. The cast-zero USB
memset fix is already upstream; do not apply the old d39227 backport to this runtime.
Native CPU CI exercises warp, persistent loading, changing input/output copies,
state restoration and the already-fixed USB matcher with this exact runtime.

Delivery uses the existing boot-frozen external-model selection shared by eGPU and
Jetson. Git Pull, select Light speed 2, restart comma and Jetson; first Jetson use
builds TensorRT. The signed host bundle includes the selected ONNX pin. Physical
loading, timing and drive acceptance must be reported separately from CI results.

Baseline parked N16 measurement before another concurrent vehicle update: 60 seconds,
1206 model frames, mean40.662/P9945.356/max48.789 ms, no invalid odometry/livePose,
no link failure. This is a stationary baseline, not a moving-drive comparison.

## Publication and interrupted physical verification

Published model catalog/release in 7c6be06a5b and 9faaee0ab9. The signed host
package pins source7c6be06a5b; all1186 payload files, signature and HTTPS hashes
verified. Model binaries and isolated runtime are independently HTTPS-verified.
Desktop39 catalog/runtime checks and17 web checks pass. CI42008426400 passes
506 contract checks,57 navigation checks, native old runtime3/2skip and Light
speed/Light speed2 five each; Jetson42008426504 passes2364/56skip plus supporting
suites and Windows installer. User docs pass. Full build42008426377 was still
running when physical work paused.

Owner's car was concurrently updated by a separate CAN task. Preserve its newer
f8e02ee19d source. With fresh P/standstill/inactive-control and Panda controlsAllowed
false verified, use IsTakingSnapshot plus hardwared restart to re-evaluate startup
conditions; actual IsOffroad and Panda noOutput were verified before selecting
light-speed-2 and requesting a normal manager reboot. Snapshot alone does not
stop an already-onroad hardwared session. New comma boot6de2c5dd selected Light
speed2; manager startup returned to onroad/P/inactive normally. No CAN test sent.

Jetson downloaded and verified909a1400 ONNX to its ordinary model cache. Before
new-model build DATA had5.6GB free. A Jetson reboot was requested to use the normal
signed selected-model boot gate. At this point the owner explicitly paused Jetson
work because the separate CAN task was operating the same device. Stop further
Jetson settings, reboots and inference trials until resumed. Boot-time automatic
installation may still proceed; the pending SSH status read timed out. **New-model
TensorRT build completion, active inference and road readiness are NOT verified.**
Do not interpret the selected model or completed downloads as running-model proof.

## Resumed live verification, 19:23–19:25 KST

Owner resumed the paused task. Read-only inspection confirms automatic signed
installation completed: Jetson source7c6be06a5b, ONNX909a1400, TensorRT10.3.0
Orin-sm87 FP16 engine769895844B. Sidecar spec matches the committed selected pin;
engine build47.0s, completed19:19:53 KST. Server and HUD active with NRestarts0.
Comma actual sourcef8e02ee19d/boot6de2c5dd has boot selection Light speed2,
IsTakingSnapshot=false, onroad/P/standstill/control inactive, clean checkout.
Web API lists all four choices, Light2 selected/effective, NOO=false and no reboot
required; internalCD210 remains independent. No new device writes or restarts
were necessary after resumption. Physical eGPU inference was not tested.

120.004 seconds of live subscription:2405 model/odometry/each-camera/livePose
frames, zero frame-ID gaps, zero invalid messages, all livePose inputs/sensors/
posenet healthy, no selfdrive alerts or missing processes. All120 sampled link
states active/ready with no error. Navigation disconnected, as in the baseline;
this model has no navigation input. No road/engaged-control acceptance is claimed.

| Metric (ms) | Earlier N16 baseline | Light speed 2 |
| --- | ---: | ---: |
| Published model execution mean | 40.662 | 37.569 |
| Published model execution P99 | 45.356 | 41.971 |
| Published model execution max | 48.789 | 44.868 |
| Host GPU mean | 24.646 | 21.159 |
| USB exchange mean | 34.030 | 31.417 |

Observed mean difference3.093ms (7.6%). This is a stationary before/after sample,
not an isolated model A/B: concurrent CAN work changed telemetry cadence and
both devices restarted. Execution-time>50ms count0 in both samples. Do NOT claim
all scheduling intervals stay below50ms: Light2 modeld loop max61.614ms and
jetlinkd loop max63.149ms were recorded, despite no model frame loss in this sample.

Comma thermal green, max56.171C. Jetson snapshot GPU70.5C. DATA4.0GB free after
ONNX/engine installation. Host journal after19:20:10 contains two slow-response
warnings at19:20:12/19:20:47; no subsequent link closure in the inspected excerpt.
Earlier19:20:00 boot handoff includes one bootstrap-message mismatch and USB
reconnect before19:20:05 normal session; this is not counted as a clean boot.

Full build42008426377 built successfully but its integration step has4 Bluetooth
imports failing with `No module named tools.jetlink`,1153 passing. The preceding
895905bd20 full run42006880280 has the exact same4 failed test names,1152 passing.
These failures predate this model addition. Focused eGPU/Jetson/user-docs CI passes.
