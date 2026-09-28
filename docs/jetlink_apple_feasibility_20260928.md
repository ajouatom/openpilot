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
