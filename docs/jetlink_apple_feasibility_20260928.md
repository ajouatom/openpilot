# Jetlink Mac and iPhone interoperability review

Reviewed upstream commit
[`df954e50f08d837546576163721f84ce247217e9`](https://github.com/zoompilot/jetlink/tree/df954e50f08d837546576163721f84ce247217e9)
on September 28, 2026. This is a source-level feasibility review, not an Apple
device test or authorization to change the deployed runtime/model/channel.

## Relationship to Carrot

Carrot vendors zoompilot/jetlink revision `194ff6dc71cd27282378fff155e94daa7770152b`
(see `third_party/jetlink/UPSTREAM.md`). Its separate adapter pins Cinque v2,
owns the comma transport in a daemon and adds optional HUD/navigation, Jetson
health and Wi-Fi provisioning. Deployment adds NAS signature verification,
recovery and the protected-image candidate. These are extensions of the same
inference architecture, not an unrelated protocol.

Direct comparison found `jetlink/protocol.py` byte-identical across these two
revisions (SHA256 `25328ac2a8322d4bc58cefe254515610351fab71de662c0dc1db7205b024d406`).
Both use wire version2, a32-byte header and the same inference messages. Swift's
generated `JetlinkKit/Sources/JetlinkKit/Pinned.swift` also specifies version2.

Both ModelSpec implementations deserialize and serialize Carrot's exact Cinque
v2 manifest identically. Checked model dimensions, warped shape, packed layout
and input/output sizes: warped uint8 `[2,6,128,256]`, packed65584 bytes,
output73808 bytes. This is metadata compatibility, not numerical inference
parity, transport timing or proof that the latest app works unchanged.

## Mac: nearest practical integration

Carrot already has `tools/jetlink/run_mac.sh`: Apple silicon, ONNX Runtime/CoreML,
verified Cinque v2, USB transport. It is a command-line launcher, not our tested
Mac GUI distribution. Upstream now provides a native Mac UI, Python and Swift
server choices, and newer Neural Engine/GPU preparation.

The first trial can preserve the current comma adapter and model: import the
exact NAS Cinque v2 ONNX into the upstream Mac app, prepare it before connecting,
then verify USB3 handshake, full spec equality and inference. Carrot's current
daemon passes no ONNX path to `ensure_engine`; a missing host model raises
EngineMissing, and its build wait is30s. Upstream's automatic comma-to-host
model transfer must not be assumed to exist in Carrot. A polished integration
needs NAS-based provisioning and readiness handling without changing onroad
validity deadlines or restoring LFS model delivery.

Upstream requires Apple silicon/macOS15+, recommends16GB, and publishes parked
M1 Pro/comma4 measurements. Those measurements belong to upstream, not Carrot.
Our existing launcher selects the older CoreML path; do not attribute upstream's
new Neural Engine/GPU results to it. First acceptance needs numerical parity,
20Hz end-to-end timing, sustained thermal load, sleep/unplug/reconnect, and
existing disengagement/fallback behavior. Native Swift USB is separately
unmeasured in upstream's referenced performance notes.

Sources: [Mac app](https://github.com/zoompilot/jetlink/blob/df954e50f08d837546576163721f84ce247217e9/docs/macos-app.md),
[Mac measurements](https://github.com/zoompilot/jetlink/blob/df954e50f08d837546576163721f84ce247217e9/docs/mac-performance.md).

## iPhone/iPad: additional comma transport required

Upstream's iPhone guide explicitly labels the app experimental and not yet run
on a phone. It documents USB-C iPhone/iPad with iOS/iPadOS26.1+, Xcode26 build
and signing on a Mac, and no App Store/TestFlight distribution at this revision.
The app must remain foreground/unlocked; suspension can interrupt inference.
Phone thermal and charging performance still need actual sustained testing.
Separate cable/network observations in transport.md do not prove phone-model
inference readiness.

iOS apps do not use the existing vendor-specific USB interface. Upstream adds
a CDC-NCM interface to the comma gadget, configures comma192.168.60.1 with local
DHCP and no router/DNS advertisement, then accepts the phone's connection on
port5599. Inference still uses Jetlink messages. This is TCP over the USB cable,
not a proposed Wi-Fi driving path.

Carrot currently only builds the plain gadget and opens FunctionFS endpoints.
Required work: confirm current AGNOS NCM/configfs/kernel support; add the
network interface/address/limited DHCP lifecycle; accept and cleanly own the
phone socket in our daemon; support offroad-only transport selection; preserve
native-model fallback and frame/model identity checks. Reuse upstream's Swift
app/backend where practical, retaining MIT attribution. Do not wholesale
replace Carrot's owner/scheduling with upstream's different modeld endpoint
lending architecture.

Carrot currently requires negotiated SuperSpeed. Keep that requirement for an
initial phone trial and check actual phone/cable/hub speed rather than assuming
all USB-C phones provide USB3. A powered hub may also be needed for sustained
operation. Current kernel support, actual USB roles, Apple build/signing and
phone performance remain untested here.

Sources: [iPhone guide](https://github.com/zoompilot/jetlink/blob/df954e50f08d837546576163721f84ce247217e9/docs/iphone-app.md),
[transport](https://github.com/zoompilot/jetlink/blob/df954e50f08d837546576163721f84ce247217e9/docs/transport.md),
[installation reference](https://github.com/zoompilot/jetlink/blob/df954e50f08d837546576163721f84ce247217e9/docs/installation-reference.md).

## Proposed scope

Start with Mac interoperability using the existing v2 model, then consider iOS
as a separate experimental transport once hardware is available. Mac/iPhone
model serving does not automatically implement Carrot's external navigation,
USB display, Wi-Fi provisioning or Jetson-specific health extensions. Those
capabilities need explicit negotiation and platform implementations.

No vehicle/runtime code, model selection, deployed image or update channel was
changed during this review. Adding these hosts does not itself require rebuilding
the newly recorded Jetson image. The existing R2 physical boot/recovery/update
validation remains pending independently. No new experiment branch was created.

## Follow-up: unchanged upstream Mac app, no Jetson changes

The owner has no Mac and requests an unchanged upstream-app integration, with
Jetson behavior preserved. The immediate request is review, not implementation.

Python `Session.on_hello` and Swift `Session.onHello` merge the backend's
`describe()` into HELLO. Neither provides a dedicated OS/platform identifier.
Both ORT implementations report backend `ort` and device tags such as
`ane-Apple_M1_Pro`, `coreml-Apple_M1_Pro` or `ane-whole-Apple_M1_Pro`. Device
names come from the CPU brand on Mac; Swift can fall back to Metal's device
name. This is peer-reported identification, not authenticated device identity.

At review time Carrot `host_label` recognized only `ort` plus a `coreml` substring.
Consequently the upstream default `ane` reports generic Jetlink; a phone's
CoreML tag could instead be labelled MAC. Do not use this display helper as
the gate for Mac-specific model provisioning.

Proposed eligibility requires the plain USB bulk transport, `ort`, a recognized
CoreML/ANE device prefix and an Apple M-series chip tag, while explicit Carrot
Jetson identity and existing TensorRT peers retain their original path. NCM/TCP
is excluded, so an M-series iPad does not enter this Mac path. Missing, conflicting
or unrecognized identity must not be guessed as Mac. CPU mode and future unknown
device naming may require explicit support. For the current ordinary upstream
app this is practical classification; the protocol does not certify macOS.

To make an already-installed Mac app work without manual model import, add a
Mac-only comma provisioning step: download the exact pinned Cinque v2 from NAS,
verify SHA256/size, supply that local ONNX to the existing upload messages if
the host needs it, and wait for preparation offroad. Reuse cached artifacts on
reconnect. Full model-spec checks, frame validity, switching and fallback rules
remain unchanged. No shared vendor-runtime refresh, Jetson server/image/update
channel change, USB-network gadget or iOS integration is needed for this scope.

Desktop verification can cover recorded/synthetic HELLO classifications, upload
framing, model identity rejection, preparation/reconnect and Jetson regression
branches. Real Mac USB, CoreML output parity and sustained20Hz performance still
require a Mac owner to validate; passing desktop tests cannot establish them.

## Mac-only implementation for external testing

The owner subsequently authorized implementation with an unchanged upstream Mac
app, preserving Jetson behavior. `jetlink_peer.is_mac_peer` now gates this path
on protocol2, plain USB, ORT and a recognized Apple M-series CoreML/ANE tag.
Conflicting host metadata, iOS/TCP, CPU and unknown device tags are excluded.
The MAC display recognizes the same upstream tags.

The comma's new `jetlink/mac.py` first requests the exact existing model. A
missing host model triggers offroad-only NAS download into a dedicated comma
cache, SHA256/size verification, and existing protocol upload. Download has a
30-minute overall bound and10-second network timeout; preparation allows15
minutes. Unverified/incomplete files are never installed. Cache reuse works
without network. Ignition-on/disconnect cancels preparation at chunk/read poll
boundaries; an already-started app build may continue. Existing ready engines
can reconnect onroad, subject to unchanged full-spec and model-switch checks.

Upstream HELLO's `engine_state` is scoped to the session request and normally
returns `none` before the first ENGINE_REQ, even with a resident model. The
ready reconnect hint therefore uses the exact `loaded` SHA and still requires
a full ENGINE_RESP spec match. An upstream HELLO implementation is exercised
in the desktop wire test so this is not inferred from a synthetic ready label.

Non-Mac peers execute the original ensure_engine call with its30-second timeout,
without Mac callbacks/downloads. No Jetson host source, image, model, signed
channel, USB gadget policy, runtime validity or process placement was changed.
The preparation transport wrapper is removed before normal inference. Tests
cover identification, corruption/short/oversized/redirected downloads, cache
reuse, interruption, unchanged non-Mac calls, actual protocol upload/inference,
wrong-checkpoint rejection, cached reconnect and partial-message polling.

See [bilingual tester instructions](jetlink_mac_testing.md). Actual Mac USB,
CoreML numerical parity, sustained20Hz and driving behavior remain unvalidated.
The Windows focused integration suite passed206 tests with25 platform-dependent
skips; this includes the Mac protocol tests and existing Jetson/HUD/navigation/
Wi-Fi/storage/update checks. The user-docs validator also passed for this change.
