# Jetson boot update and first-time transition

The user requested checking the comma-selected Jetson release at startup,
holding Jetson inference/HUD while an update is required, waiting through
Internet loss, and showing the reason on the comma. Existing cars power the
Jetson off with ignition; the user approved a one-time maintenance wait to
transition those installations without per-car SSH or rewriting cards.

## Existing installations

Carrot Web > Tools provides **Jetson first-update wait** for a connected older
Jetson. Entry requires one continuous second of fresh valid Park, raw zero
speed, standstill, and disabled/inactive selfdrive and lateral/longitudinal
controls. The action persists `JetsonLegacyUpdatePending`; generic parameter
writes, backup, QR and profile restore exclude this internal state.

`hardwared` then performs a real offroad transition, even with physical ignition
still on. Manager stops onroad assistance, while the always-running Jetlink
daemon continues USB network provisioning and supplies a fresh offroad release
selection. No fake ignition or onroad telemetry is sent to the host. The device
shows an offroad notice, and the Web offers cancellation. A later boot remains
offroad until the new runtime is confirmed or the user explicitly cancels.
Hosts predating the Wi-Fi control channel receive a minimal HUD-format snapshot
with the actual road state and signed pin, without starting preview workers.

The existing host updater checks after about two minutes from boot, then every
15 minutes with up to 30 seconds of timer jitter. It stages the signed pinned
bundle while the comma reports offroad, and applies it on the next Jetson boot.
Keep ignition and Internet on while parked for this first download. Existing
hosts do **not** transmit download completion or source identity; elapsed time
is not proof of completion. The UI must not claim completion. If power is
cycled before staging completes, the persistent wait continues next boot.

The newly installed signed runtime migrates the stable updater using the
passwordless sudo already configured for `jetlink` in published images. It
atomically copies helpers under `/opt/carrot-jetlink/updater`, then writes an
enable marker. It makes no protected base-OS changes and defers the new policy
until the following boot. A normal server HELLO reports the source commit and
installed marker. Only an exact match to the comma's current signed host pin,
plus that receipt, clears the first-time hold. A bootstrap HELLO, an older
source, disconnect, or timeout cannot clear it. Custom images lacking the
published service account privileges are not established as compatible.

## Subsequent boots

The existing apply unit starts a separate root USB-only bootstrap service using
`systemd-run`. This avoids the old oneshot apply unit's finite startup timeout
while Internet is unavailable. Runtime entry points wait before loading the
inference backend or renderer. The bootstrap accepts the signed release pin
and private Wi-Fi provisioning; it has no inference/engine-upload capability.

The comma selects the release through its committed `host_release.json`.
Matching installed source needs no Internet. A differing source downloads with
the existing size/hash/signature/ABI/storage checks, stops guarded services,
uses the existing isolated synthetic candidate probe and atomic activation,
then starts fresh runtime processes. It does not require another ignition cycle
for these subsequent updates. No model, runtime ABI, base image, or stable NAS
channel change is intended.

Download failures retry after 30 seconds. Rejected candidates preserve the
previous files but do not grant this boot permission to run an incompatible
release. An interrupted transaction is recovered before comparison. Permission
is tied to the current kernel boot ID and installed source. USB disconnection
invalidates the remembered pin's freshness. Private provisioning files retain
the `jetlink` owner when written by the root bootstrap, permitting the normal
unprivileged server to replace them later in sticky `/dev/shm`.

The comma displays checking, downloading, verifying/applying, waiting for
Internet, or retry failure. Stale status expires. This normal boot gate holds
the Jetson; it does not force the comma offroad or change its existing local
model fallback policy. The one-time maintenance mode is the explicit exception
which stops the comma's onroad operation.

## Validation and limits

Desktop tests cover signed selection, boot-specific permission, offline retry,
failed candidate retention, stale/disconnected selection, provisioning-only
protocol, old/Mac compatibility, migration deferral and one-time installation,
vehicle entry checks, backup/write exclusion, receipt-based completion, status
expiry, and Web entry/cancellation. The Web bundle is rebuilt from its sources.
The focused Python set passed 126 tests with 3 platform skips; the device badge
source check passed, as did 5 Web tests. New-module Ruff checks pass. The two
broader eGPU failures below were also confirmed against unchanged HEAD sources.

No physical Jetson installation, systemd boot ordering, ignition power cycle,
network transfer under vehicle power, or C3/C4 display/assistance transition has
been validated. Existing Linux-only filesystem/update tests skip on Windows.
Two broader pre-existing eGPU source-contract tests expect an old loader
timeout and settling expression; they are unrelated to this change and are
tracked separately from the focused checks.

## Published release

Source: `84087a5b78118acc40234bfd8ef64235421f1aab`.
The signed comma pin selects the immutable NAS directory
`jetlink-host-84087a5b78118acc40234bfd8ef64235421f1aab`.
Bundle size: 64,518,963 bytes; SHA-256:
`6e9706ab60008695f9cb28d3ef792be8465c9f1eb2e961f1c43726e3f9e56d91`.
Signature validation, committed-source byte comparison, NAS copy verification,
and complete HTTPS readback of both bundle and manifest passed. Private keys,
untracked work and captures are absent. The pinned Cinque v2 model and
L4T/TensorRT ABI are unchanged; the SD-image/stable-channel pointers are unchanged.

## Follow-up: first-update wait screen

The iconless maintenance card originally placed its sole body in the shared
54px icon grid column (44px at the mobile breakpoint). The body now spans both
columns. Production CSS/markup were rendered in Chrome at 1030px and a real
390px iframe viewport; the text/button stay inside the card without horizontal
overflow. The localized waiting text now explicitly distinguishes the roughly
15-minute legacy check interval from download time and completion.

The published older updater was checked directly: boot check after two minutes,
15-minute subsequent interval, and up to 30 seconds randomized delay. A parked
device reporting fresh offroad and the correct signed pin completed an explicitly
triggered existing stage service in about 18 seconds, with the 64.5MB bundle and
the 766MB model already cached. Its status was `staged`; this was download and
verification, not runtime activation or a general duration guarantee. Triggering
the service preserves its offroad/signature/ABI checks. SSH diagnostic access
requires an individually authorized key; no fleet-wide SSH credential is added.
