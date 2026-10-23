# N16 response projection and same-frame input overlap — 2026-10-23

The owner requested review and integration of contributor commits
[136119c](https://github.com/tjdgus200/openpilot/commit/136119c818566586588fa9c820d2b1749bac539a)
and [2e971ad](https://github.com/tjdgus200/openpilot/commit/2e971adb3ac1d542062524707ffdaef5d00dd780),
then explicitly selected publication without a new vehicle experiment because
the vehicle/Jetson were off. The comma read-only preflight had no live driving
services; no vehicle process, parameter or installed source was changed.

## Review and implementation

Accept both changes for carrot-nav. The exact N16 model SHA and complete spec
fingerprint select a 2,066-float driving-output prefix instead of 18,580 floats.
This reduces output payload from 74,320 to 8,264 bytes, saving 66,056 bytes per
frame (88.88%, 1.32112 MB/s at 20 Hz), before transport framing/telemetry.
The omitted hidden/map/navigation feature outputs and padding are not consumed
by the driving-output parser. They are not fabricated as zeros. The host still
validates the complete output and recurrent state before replying. GPU state
stays owned and checked by the existing stateful runtime.

Protocol-v2 HELLO and local IPC negotiate independently. Old hosts, old clients,
other models, protocol v3 and SEND_RAW_PRED retain full replies. Exact byte count,
frame ID, projection marker, allowed flags, finite values and telemetry parsing
remain enforced. Full/compact response choice may change between requests;
reset bit zero remains separate from local projection bit one.

The second change fills the current frame's CPU inputs after submitting its GPU
warp and before reading the warp result. The existing finally block drains GPU
work even when input preparation fails. USB submission still waits for both.
No camera-frame pipeline, asynchronous USB write, deadline, scheduling priority,
model weight, state history or control policy is changed.

The original contributor diffs apply cleanly to the current NAV implementation;
the unrelated contributor display-statistics additions are not imported.
Carrot-wip has a different queued-model runtime without the N16 stateful
prerequisites. It receives this review record only; its model/runtime stays intact.

## Verification

119 focused desktop tests pass; one real POSIX-stream test is skipped on Windows.
This includes the contributor's full/compact/full output and recurrent-feedback
comparisons, 64 seeded parser comparisons, same-frame image/context/desire byte
equality, route disappearance, and context/GPU/USB failure fallback ordering.

Additional tests exercise the currently enabled DeviceStateQueues path:
omitted-output NaNs and device-state validation failures still reset/reject the
frame. Ten local-IPC cases reject short/long/nonfinite/stale/wrong-contract
replies. Mock IPC tests are portable on Windows; the added real socket test
uses actual stream framing, full/compact switches and telemetry on Linux.
Earlier attempts to run native socket/daemon suites on Windows failed because
fcntl, AF_UNIX and sendmsg are unavailable; they are not counted as passes.
[Linux integration CI](https://github.com/ajouatom/openpilot/actions/runs/39888833740)
passes 1,062 broad checks (56 skips), including the added actual-stream test,
94 updater checks, 119 navigation/HUD checks, 11 real-storage and 11 network
namespace checks. The Windows installer job also passes.

The contributor's 41.480 -> 38.898 ms bench comparison was an earlier prototype
with real cameras and simulated navigation/cruise inputs. Their final negotiated
implementation was PC-tested, not freshly device-benchmarked in those commits.
No local hardware speedup, 50 ms hard deadline, road equivalence or cure for
the previously observed latency/locationd events is claimed here.

## Delivery

Both comma and Jetson code must support projection to reduce bytes. Publish a
new immutable signed host bundle and pin it in carrot-nav, retaining the exact
N16 ONNX and existing TensorRT plan compatibility. Normal delivery is Git Pull
on comma, then comma and Jetson restart/reboot with offroad Internet available
for the boot updater. The updater verifies signature/hash, probes the candidate
and keeps its rollback behavior. Updating comma alone remains compatible with
the old Jetson but does not enable response projection. No SD reflash or USB
patch is needed for the owner's existing signed bootstrap.

The imported contributor notes describe their original commits and experiments:
[projection](jetlink_n16_response_projection.md) and
[input overlap](jetlink_input_warp_overlap.md). The release status here supersedes
their original statement that no selected host package was changed.

Published host source: `e7ff007ddc5c539ec34ea83d7b9ac7de1df1a6ae`.
The signed [release manifest](https://upload.shind0.synology.me/models/jetlink-host-e7ff007ddc5c539ec34ea83d7b9ac7de1df1a6ae/manifest.json)
selects a 64,695,651-byte bundle, SHA-256
`e72bab4b064bffa62d44531ed269688c41782e0c5b5074333d360f6f4310b7ff`.
All 1,091 payload files match committed source, including the existing selected
model alias for older bootstraps. No private signing key or task evidence is
included. The unauthenticated HTTPS bundle download matches the complete byte
count/hash; the existing trusted public key verifies the signature. The NAV
host selection now points to this release and preserves ONNX `01effc67...`.
An additional 26 local signature/bundle/NAV checks pass (one Windows skip).
Publication is complete; installation and speedup on the owner's powered-off
Jetson are not claimed.
