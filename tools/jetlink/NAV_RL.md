# Experimental RL navigation model on Jetson Orin

## carrot-nav integration (2026-10-13)

Imported from `tjdgus200/openpilot` commit `38b06d6a48` with its three
ResAction/runtime prerequisites, using single-parent commits on `carrot-nav`.
The sections below describe the contributor's local experiment and evidence.
**This maintained branch instead selects a signed `host_release.json`.** No
PC package registration is required on a Jetson that already has the boot
updater. The local experiment helpers remain available for separate tests, but
`local_host_release.json` is absent here and cannot override the signed pin.

The published host bundle includes the selected pin under both `jetson_model.json`
and the legacy `cinque_v2.json` filename. This is needed because installed older
bootstraps check the latter before executing any candidate code. The repository's
historical Cinque specification stays unchanged for other clients and tests.
Only the branch's immutable signed release is selected; the shared stable channel
and `carrot-wip` are not changed.

At the next normal Jetson boot with the updated comma attached, its existing
updater downloads the original ONNX and the signed runtime bundle, builds a
device-specific TensorRT engine and probes finite recurrent inference before
activation. A failed build/probe preserves the previous files and withholds
Jetson readiness; it does not silently activate the wrong driving model.

No Jetson is connected for this integration. TensorRT compilation, native USB
timing, CPU load, thermal behavior and driving behavior remain unverified.
The imported host implementation copies and validates recurrent state through
host buffers; sustained 20 Hz must be measured on Orin before claiming its
performance. GPU-resident state is a possible follow-up only if profiling shows
that these transfers are a bottleneck.

See [the integration verification record](../../docs/jetson_nav_integration_20261013.md)
for checks performed on this maintained branch.

## Contributor experiment (historical instructions)

Branch: `tjdgus200/openpilot:codex/jetson-nav-rl`.
Base: `ajouatom/openpilot:carrot-nav` at
`d6a92d91e2cada40ab588427d039b935ef063a1a`.

This branch connects Carrot's existing map/route inputs to the Jetson transport
and TensorRT host. It preserves the existing local experiment selector and the
signed official Internet update path. **The new PC package must be registered
on Jetson before this comma branch can activate its model.** The previously
installed ResAction package cannot run this model by itself.

## Exact model and runtime

- Upstream: [a03bb1d4](https://github.com/commaai/openpilot/commit/a03bb1d4e80050cae0007780ecb8eb7cf06d0cb9),
  RL nav driving model `80eedcdd/12864`.
- Original ONNX: 814591106 bytes, SHA-256
  `1d18c42ea4cd95c8bffab4635844b78b8e2d4e25b9de44a9c5ac2e0961371a2e`.
- Local profile: `nav-rl-80eedcdd-v1` (separate from `resaction-2fb4ac4-v2`).
- Existing protected DATA runtime: aarch64, L4T 36.4.7, TensorRT 10.3.0.
- QSPI, JetPack, operating system, partitions and official signing key are not
  changed by package registration. Model weights remain outside Git.

The author's ONNX is public at
[the RL nav package](https://upload.shind0.synology.me/models/comma4-big-nav-rl-80eedcdd/big_driving_supercombo.onnx).
Its bytes were checked against the official hash above. The supplied tinygrad
PKL is for the existing eGPU backend; this Jetson path builds from ONNX.

## Input and execution path

Comma prepares the native camera warp, map and maneuver vector. Jetlink sends
the newest images and float32 context through local IPC and USB. Orin owns all
five state buffers and returns the named driving output.

| Field | Shape / behavior |
| --- | --- |
| Warped images | uint8 `[2,6,128,256]` |
| Original scalar context | desire 8, traffic convention 2, action delay 2 |
| Map | float32 `[1,1,256,256]`, pixel range **0..255** |
| Navigation | float32 `[1,47]` |
| Context payload | 65595 floats / 262380 bytes, excluding images and headers |
| Driving output | 18580 float32 values, metadata-defined slices |
| Host history | image, desire, features, map features, navigation features |

Map normalization is inside the graph. Missing or expired inputs are zeroed
each frame. Existing map age, instruction age and route identity checks are
retained. A fresh active Jetlink nav status requests map rendering without
misusing `UsbGpuActive`, which controls the separate eGPU path. Map rendering
stops when that demand expires unless HUD/Web/eGPU independently needs it.

Existing Carrot navigation source selection, map tile credentials/availability,
driver monitoring, control/validity policies and internal fallback remain from
the base branch. No map or route is fabricated when a provider is unavailable.
The guarded external-model join and failure/disengagement signaling are retained.

Both the v2 host and v3 compatibility adapter preserve the complete navigation
payload. Only a queued legacy model's `prev_feat` tail is omitted in v3.
All five next-state tensors must be finite and match the host buffers before
feedback is committed; a bad state clears all histories.

## One-time PC preparation and official restoration

From this branch's committed checkout, with the Python dependencies installed:

```sh
python tools/jetlink/download_experiment.py --output /path/to/nav-rl/model.onnx
python tools/jetlink/build_host_bundle.py /path/to/nav-rl/host.tar.gz
python tools/jetlink/prepare_local_experiment.py \
  --bundle /path/to/nav-rl/host.tar.gz --model /path/to/nav-rl/model.onnx \
  --output /path/to/nav-rl/package
```

Register the complete package against the existing DATA `runtime` from a
stopped/offline Jetson recovery environment. Identify that mount before running
the installer; the paths below are placeholders, not block-device targets.

```sh
sudo python3 /path/to/nav-rl/package/install_local_experiment.py \
  --package /path/to/nav-rl/package --runtime /mounted/DATA/runtime
```

Registration preserves `current`, official cached releases and both local
profiles. The next normal boot verifies/builds/probes the selected release
before activation. A failed candidate retains the previous runtime.

| Comma branch at Jetson boot | Selection |
| --- | --- |
| `codex/jetson-nav-rl` | Root-registered `nav-rl-80eedcdd-v1` |
| `codex/jetson-resaction` | Previously registered `resaction-2fb4ac4-v2` |
| Official Carrot branch | Its signed official release via existing Internet updates |

Change the comma branch, then **restart Jetson with comma USB attached**.
Changing the comma branch while keeping Jetson powered does not rerun the boot
selector. Uncached official releases require an Internet connection. Local
selection never accepts an executable URL/path from the comma and does not
replace the official signing key.

## PC validation and remaining device work

The original hash-verified export and its Orin input adaptation passed ONNX
`full_check=True`; input/output shapes and dtypes were checked against the pin.
For CPU comparison, the reference only removes the one arithmetic-free tinygrad
layout hint. The candidate additionally promotes the byte-preserving image
queue to FP16, as the TensorRT backend does. Map/state weights are unchanged.

ONNX Runtime 1.31.0 CPUExecutionProvider compared **140 continuous frames plus
8 frames replayed after reset** through the actual Jetlink client, host session,
state queues and Carrot output parser. All 18580 driving values and all five
next-state tensors were exactly equal (maximum absolute difference 0).
Inputs cover all byte values, black/white/noise images, desire transitions,
both traffic conventions, varying action delays, map+instruction, instruction
without tiles, no navigation, and navigation recovery. Reset replay matched.
A NaN injected into map history was rejected and all histories were cleared.

Reproduce the full-model check on a PC with numpy, onnx and onnxruntime:

```sh
python tools/jetlink/validate_nav_rl_cpu.py \
  --model /path/to/nav-rl/model.onnx --output /path/to/nav-rl/validation
```

The output directory contains reference/candidate graphs and a JSON report.
CPU loopback timing is not an Orin performance result. Unit/integration checks
also exercise map expiry/demand, complete v2/v3/IPC payloads, fallback and
local-to-official-to-local switching in temporary Linux directories. Those
update transaction tests use fixture artifacts and simulated device probes.

The isolated Linux source-snapshot suite passed **661 tests, with 11 skips**:
nine require a built native map rasterizer, one requires the external Jetlink
reference checkout, and one requires its separate root integration opt-in.
Two existing test files were excluded because this PC environment lacks their
dependencies: `test_route_engine.py` needs the native Params/messaging build;
`test_hud_health.py` needs pyray. Source was copied with LF endings to a Linux
filesystem so shell syntax and Unix permissions were tested accurately.
Ruff introduced no new findings, and `git diff --check` passed.
See [the machine-readable PC report](validation/nav_rl_pc.json). The full-model
run records the pre-commit base/patch identity; file hashes identify the tested
runtime sources independently of the later documentation commit.

**Not validated:** TensorRT compilation/execution on the user's Orin, TensorRT
numerical parity, sustained 20 Hz with native camera warp, physical USB/thermal
behavior, real map-provider availability and vehicle behavior. The earlier
ResAction device timings do not validate this new model. This is an experimental
branch prepared and checked on PC; no device installation is implied.
