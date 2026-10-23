# Jetson USB display signal observations - 2026-10-23

The owner requested displaying the existing traffic-signal processing results
on the USB panel attached to Jetson, using the HUD transfer during inference.
This change is in carrot-nav's existing signed host runtime. Carrot-wip receives
the review record only; its different model/runtime selection is not changed.

## Behavior

The existing signal-assistance selection on comma controls publication. Forward
only the bounded observation schema: red/green/unknown evidence, track ID,
coordinates, confirmation state, source session, frame ID and capture timestamp.
No extra image, camera inference, USB message type or transfer frequency is added.
The receiver reuses the existing camera-independent colored signal status ring
and the R/G/? detection boxes over the road-camera preview. Previous results
retain the existing dimmed/marked UI semantics; they are not control permission.
Wide-camera images do not receive road-camera boxes. Turning signal assistance
off, losing metadata or losing the snapshot clears the remote results. No new
setting is required; the existing signal-assistance mode must already be on.

Original camera dimensions and VisionIPC timestamp_eof accompany the existing
full-field JPEG preview. The receiver binds metadata only after that preview
has actually been decoded/drawn, translates sender times into its own monotonic
clock domain, and retains the earliest mapping for duplicate observations and
camera frames. Camera restart/frame-ID reuse also changes the decode key.
Old senders without capture metadata remain compatible and show no inferred
boxes. We never invent a full-resolution camera frame from the JPEG dimensions.

Source observation reads are bounded to 8 KiB and allowlisted to a 4 KiB signal
envelope, at most 20 tracks. The existing source/receiver signal freshness,
frame-skew, session-reset and two-second marked UI-memory gates are preserved.
Duplicate packets cannot renew the observation/color lifetime. Optional signal
metadata is dropped first if a snapshot would exceed the existing 96 KiB limit;
it must not displace camera or NAV previews. Unknown/malformed data hides the
signal information without blocking the rest of the HUD.

Read/validation/JSON construction occur in the existing publisher process.
The USB owner continues to transmit one ready snapshot during inference, under
the existing transport scheduling/deadline. Receiver adaptation is scoped to the
Jetson renderer process and does not write observation/control files. Host HUD
status now reports signal enabled/color/held/online for subsequent diagnosis.
No model, recurrence, inference contract, traffic control or scheduling priority
is changed. The preceding N16 input-staging optimizations remain selected.

## Validation and timing limits

98 focused desktop checks pass; two native transport checks are skipped on
Windows and covered by Linux CI. New coverage includes malformed/oversized data,
mode-off, expiry, different clock domains, duplicate timestamps, session change,
raw unknown/held color, legacy/missing snapshots, road/wide and frame mismatch,
actual renderer rectangle/status-ring calls, actual remote-camera draw through
CPU graphics doubles, frame-ID reuse, import-order adapter installation and
snapshot budget fallback. Existing local signal display tests also pass.

Synthetic compact serialization adds 232 bytes for one track, 500 for five and
1,516 for twenty (specific test values, excluding camera metadata and enclosing
packet delimiter). At the existing 10 Hz snapshot rate these are approximately
2.32/5/15.16 kB/s. The maximum signal envelope is 4 KiB plus the JSON field name;
capture metadata adds only scalar fields to the existing previews. There is no
additional signal image or model input bandwidth.

These byte counts are not a measurement of vehicle roundtrip latency. The
expected small transfer fits within normal GPU execution, but actual scheduler,
USB and receiver contention can still affect completion. The vehicle/Jetson
remain unmodified and no measured latency-neutrality, physical screen check,
road validation or locationd/50 ms-tail cure is claimed.

## Delivery

Publish a new immutable signed host package, verify its complete HTTPS size/hash
and signature, and pin it in carrot-nav. Normal activation is comma Git Pull
followed by BOTH comma and Jetson reboot with offroad Internet for the normal
updater. Updating only comma is backwards compatible but an old host will not
render the new signal metadata. No SD rewrite or USB bootstrap patch is needed.

[Source integration CI](https://github.com/ajouatom/openpilot/actions/runs/39924677333)
passed: 1,142 broad checks with 56 existing skips, 94 updater checks, 119 NAV/HUD
checks, 11 real-storage and 11 network-namespace checks; the Windows installer
also passed. Local signed selection/bundle/NAV checks passed 26 with one Windows
skip. Counts cover overlapping suites and are not a count of independent tests.

Published source: `54e706fa0573e91dc3b4a0d4f625b9d81a6b68cc`.
[Signed release manifest](https://upload.shind0.synology.me/models/jetlink-host-54e706fa0573e91dc3b4a0d4f625b9d81a6b68cc/manifest.json).
Bundle: 64707580 bytes; SHA-256 `42c2951e92bd47029ff22fb058f577a0a3d334eeca62c92aa7b6ef479227787c`.
All 1,099 regular payload files match committed source (including the existing
selected-model alias for older bootstraps), with no private task evidence or
signing key. Full anonymous HTTPS download matches byte count/hash; the existing
trusted key verifies its signature. The selected N16 ONNX stays `01effc67...`.

The new host selection and source are published on carrot-nav. Carrot-wip
receives this record without changing its separate model/runtime. Installation,
physical USB-screen appearance and actual vehicle latency remain unmeasured.
