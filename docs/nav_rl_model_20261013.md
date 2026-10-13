# RL navigation model update (carrot-nav only)

The user requested the new nav model and its ONNX for later Jetson work, explicitly restricting publication to `carrot-nav`. This updates the eGPU selection only. Internal/DM models, Jetson selection, navigation source policy, signal assistance and control limits are preserved.

## Artifact identity

Later user request: Jetson selection is now updated separately by the
[Jetson navigation integration](jetson_nav_integration_20261013.md). The
eGPU-only scope above describes the original artifact update.

Source: comma [a03bb1d4e80050cae0007780ecb8eb7cf06d0cb9](https://github.com/commaai/openpilot/commit/a03bb1d4e80050cae0007780ecb8eb7cf06d0cb9), with subject `rl nav driving model (80eedcdd/12864)`. Both official LFS pointers are in this commit. The complete embedded checkpoint is identical in PKL and ONNX:

`cf8f6120-49ee-48f8-b2d7-d663a4708271/56320/80eedcdd-f97b-46d0-9b94-780685292acf/12864`

| Artifact | Bytes | SHA256 |
| --- | ---: | --- |
| big_driving_tinygrad.pkl | 822465506 | `20cae5d01d0c59352fa6bf23cb0bf2a9c6c7257f6b9997652f6ef64669e194f8` |
| big_driving_supercombo.onnx | 814591106 | `1d18c42ea4cd95c8bffab4635844b78b8e2d4e25b9de44a9c5ac2e0961371a2e` |

The upstream tinygrad submodule remains `d3f09c9bbd542fbfbe68c8569a1173550c408969`. The deterministic runtime archive remains SHA256 `90970e01f2ba70d553294c0f9d5c6fe1678827f750705cab679a7b19b7bcb0b2` (2739152 bytes).

The separate NAS package is [comma4-big-nav-rl-80eedcdd](https://upload.shind0.synology.me/models/comma4-big-nav-rl-80eedcdd/manifest.json). It includes both models, runtime, `precompiled.json`, and a separate `onnx-manifest.json` retained on the NAS. The [ONNX download](https://upload.shind0.synology.me/models/comma4-big-nav-rl-80eedcdd/big_driving_supercombo.onnx) is an archive for later integration, not the active Jetson selection. The ONNX HTTPS endpoint returns 200 with the expected length; the auxiliary ONNX manifest is NAS-local because the server does not expose that filename. Upstream downloads and NAS copies are size/SHA256 verified; binary weights remain outside Git. Local originals are under `.analysis/scratch/2026-10-13-nav-model/`.

## Required compatibility change

| Contract | Previous nav model | New RL nav model |
| --- | --- | --- |
| nav_map | uint8 `[1,1,256,256]` | float32 `[1,1,256,256]` |
| navigation | float32 `[47]` | float32 `[1,47]` |
| Extra history | state_map_q, state_nav_q | state_map_features_q, state_navigation_features_q |
| Flat output count | 18452 | 18580 |

The generic adapter accepts these two verified navigation layouts and rejects unrecognized combinations. Declared-dtype packing handles the float map; runner assignment converts/broadcasts the shared uint8 image and 47-value instruction. Pixels remain 0..255: the ONNX graph divides by scalar 255 inside its map encoder. Do not normalize a second time before upload. Missing navigation inputs are still cleared on every inference.

State feedback matches named inputs to their `next_` outputs, including the new feature histories. Metadata-defined output slices now include two additional 64-value features. Existing motion parsing accepts the plan dimensions and slices. The previous nav metadata fixture remains tested alongside the new official fixture.

The float map adds 196608 bytes per upload, approximately 3.93 MB/s at 20 Hz. This is a size calculation, not measured CPU/USB/GPU timing. No new process, render pass, thread or affinity change accompanies the update.

## Verification and limits

ONNX structural checking passes. Its input/output specifications and metadata match the PKL; it has no external-data tensor files, IR version 10, standard opset 20, and an `org.tinygrad:Contiguous` local function. This does not establish TensorRT compatibility, inference parity or a compiled Jetson engine. Preserve the original export for later Jetson work.

Desktop model/delivery/startup/navigation checks: 256 passed, eight platform-dependent skips. Coverage includes both input layouts, state pairing, pixel range, clearing missing inputs, output parsing, artifact pinning and startup behavior. Lint passes for the changed runtime, package builder and relevant tests. The Linux compatibility job uses the new package's unchanged pinned runtime for native CPU warp/buffer checks.

An actual empty-cache HTTPS installation verified the model/runtime hashes and checkpoint. Subsequent reuse with all network calls forbidden passed. Download/install success is distinct from GPU boot validation. Vehicle deployment, physical eGPU inference timing, Jetson execution and driving validation are not claimed by this branch update.
