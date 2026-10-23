# N16 map and native image staging integration - 2026-10-23

## Review

The owner requested integration of contributor commits
[fc02df9](https://github.com/tjdgus200/openpilot/commit/fc02df9c0d17dc3e8c2f6d53e17dd46ebf042888)
and [e812f68](https://github.com/tjdgus200/openpilot/commit/e812f683d761956c97a8527564eb474c29809217).
The second is only the initialization import repair; its prerequisite
[936ea57](https://github.com/tjdgus200/openpilot/commit/936ea57879d3bf68ababb6d89b25bcc1fb7f69c0)
contains the actual native converter and is included. No unrelated upstream
runtime selection or display changes are imported.

Direct map staging consumes the existing negotiated uint8 map and writes the
same engine-owned input buffers without float32 expansion, reserialization and
packed-context allocation. It checks binding identity/address/type/shape and
validates all scalar fields before input writes. Expired zero maps overwrite
old maps. Fractional, negative-zero and other noncanonical maps retain the
original raw float path; no new quantization is introduced.

The camera converter replaces NumPy table lookup with a small C loop using the
same 256 FP16 bit patterns. All uint8 values are exactly representable in FP16;
there is no arithmetic, scaling or interpolation change. Compilation, loading
and self-test happen once at model load, never per frame. Missing compiler,
compile/load error or self-test failure retains NumPy. References own the DLL
and buffers; dtype/size/alignment/contiguity/writability/alias checks precede C.

The contributor shipped both switches default-off. This integration deliberately
selects both by default ONLY for the exact existing N16 model SHA AND complete
spec fingerprint already pinned by the output contract. Other models keep the
previous defaults. CARROT_TRT_DIRECT_MAP=0 and CARROT_TRT_NATIVE_IMAGE=0 remain
independent diagnostic overrides. Low-level make_queues defaults remain off.
No user setup or manual device edit is needed.

Acceptance is based on exact input equivalence, synchronous buffer lifetime,
unchanged state/output checks and the combined regression suite below. It is
not based on adding the contributor's separate timing numbers. Their map note
requested further isolated inference evaluation before default activation;
our decision changes that rollout policy for the pinned contract, without
claiming such a new Orin experiment was performed here. Their image-only Orin
report includes 32-frame full output/state equality and isolated ABBA timing.
These are contributor records, not independently reproduced measurements.

The N16 model, TensorRT graph, GPU recurrence, projection, camera warp,
USB protocol/deadlines, display traffic and controls are unchanged. Carrot-wip
has a different model/state runtime and receives the integration record only.

## Validation

Initial Windows focused suite: 94 passed, 34 skipped. The skipped checks need
a real Linux C compiler or POSIX socket and are explicitly delegated to Linux
CI; no Windows doubles are counted as native conversion results.

Additional combined acceptance checks exercise both optimizations through the
actual session/client path, FP16 and FP32 contexts, CPU and device-owned state,
32-frame sequences, changing/expired maps, raw fractional/negative-zero fallback,
full/projected replies, explicit reset, nonfinite rejection and recovery.
Every tensor is compared by complete-byte digest before a deterministic CPU
reference engine runs. The reference outputs and reset/validation calls must
match. This is not a TensorRT, camera or physical USB experiment. Separate
native tests check all byte values, tail lengths, destination guard regions,
source preservation, malformed buffers, actual initialization and fallback.

[Source Linux CI](https://github.com/ajouatom/openpilot/actions/runs/39912647388)
passed: 1,117 broad checks with 56 existing skips, including the actual native
converter and four combined FP16/FP32 and CPU/device-state cases. Also passed:
94 updater, 119 navigation/HUD, 11 real-storage and 11 network-namespace checks,
plus the Windows installer. The final local signed-pin/bundle/NAV checks passed
26 with one Windows skip.

## Delivery and limits

Publish a new immutable signed host runtime and select it in carrot-nav.
Normal installation is comma Git Pull followed by BOTH comma and Jetson reboot,
with offroad Internet available for the existing boot updater. The model file
and TensorRT plan compatibility stay unchanged. No SD rewrite or USB bootstrap
patch is required. The vehicle/Jetson remain untouched during this task, as the
owner selected deployment first with devices off.

Contributor image-only preparation saved 0.3442 ms and preparation plus engine
saved 0.3608 ms on average; its maximum increased by 0.9097 ms. The separate
map microbenchmark saved about 0.14 ms. Neither a combined 0.48 ms gain nor
improved USB tails, a 50 ms deadline or a locationd cure is established here.
Imported original records: [map](jetlink_direct_map_20261023.md) and
[image](jetlink_native_image_20261023.md). This document governs our defaults
and publication status, superseding their default-off/no-package statements.

## Published release

Source: `e4779e059a5642d4a8efe989937a84a58210d220`.
[Signed release manifest](https://upload.shind0.synology.me/models/jetlink-host-e4779e059a5642d4a8efe989937a84a58210d220/manifest.json).
Bundle: 64702473 bytes; SHA-256 `f17dd14ab93204d18f0f341904de46ad0d8e5dad22d9cca7e7d7b5da93fdffaa`.
All 1,097 regular payload files match committed source (the existing selected
model alias for older bootstraps is preserved). No private task files or signing
key are included. Full anonymous HTTPS download matches size/hash and the
existing trusted public key verifies the signature. The carrot-nav selection
pins this release and preserves model SHA `01effc67...`.

The rollout selection and review record are pushed on carrot-nav; carrot-wip
receives documentation only. No device installation or local hardware speedup
is claimed. A normal reboot updater, not a per-device overlay, activates it.
