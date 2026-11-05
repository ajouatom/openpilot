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
