# Bluetooth physical indicators and fast button input

The owner authorized publishing only `carrot-nav-blink` and installing through
normal Git Pull plus a signed Jetson update. This branch starts at the unchanged
`carrot-nav` tip `24c1ba5e4b`; `carrot-nav` and `carrot-wip` are not updated.
Installation and live acceptance are recorded separately below when completed.

Carrot Web → Tools → Bluetooth remotes now offers **왼쪽 방향지시등 켜기** /
**오른쪽 방향지시등 켜기** (`blinkerLeft` / `blinkerRight`). They request physical
one-touch indicator operation. Existing `laneLeft` / `laneRight` retain their
lane-change meaning. No hazard, hold, steering, cruise, or ATC command is added.

Each remote also has **더블클릭 사용** and **길게 누르기 사용** options. New remotes
default both off. Existing files infer enabled gestures from assigned mappings;
saving makes the two options explicit. Disabling an option preserves its mapping.
With no competing gesture assigned/enabled, a key fires on its first complete
press report, never again on repeats/release. Touch swipes fire when direction
crosses the existing threshold; taps wait for release to avoid mistaking the
beginning of a swipe for a tap. Enabled double/long gestures retain 350 ms / 700 ms
timing. Learning mode still identifies gestures without vehicle commands. A key
already held at connect/reload must be released first. Web status polls every
250 ms instead of 1 second; this is display cadence, not measured actuator latency.

## Optional hardware path

Bluetooth owns a separate `blinker` journal. A negotiated Jetlink extension,
separate from read-only HUD messages, delivers bounded, expiring requests to an
independent normal-priority Jetson worker. Unsupported/unconfigured hosts create
no worker and receive no indicator messages. There is no HTTP command endpoint,
network listener, arbitrary CAN payload API, or dependency on a desktop SSH relay.

The only current profile is experimental `hyundai-b1-3c1`: classic PEAK PCAN-USB
`0c72:000c`, Linux `peak_usb`, 500 kbit/s B1, standard ID 0x3C1, and 0x413 feedback.
It is based on this owner's stationary captures, not a universal Hyundai or
USB-CAN protocol. Matching wiring, vehicle protocol, driver and adapter are
prerequisites. This experimental branch includes an explicit bundled profile for
`HYUNDAI_IONIQ_5_PE` and auto-selects exactly one verified PEAK interface. No adapter
or multiple matching adapters leaves the channel disabled. A local disabled
profile overrides the bundled profile. Bluetooth mappings still require explicit
selection through Carrot Web.

The signed runtime includes unmodified upstream Linux v5.15.148 PEAK sources,
license and source hashes. On the exact `5.15.148-tegra` kernel, startup builds
against installed matching headers and loads the module for that boot, before
inference starts. No precompiled kernel binary, permanent modules-load setting,
network download or source overlay is needed on the vehicle. An optional driver
failure leaves inference available and the indicator unavailable.

The owner subsequently requested manual operation while driving. The negotiated
capability is now `carrot_blinker_manual_v1`; it requires fresh valid vehicle/CAN
state and the supported fingerprint, but does not require Park, zero speed or
disengaged controls. It does not change steering/cruise actions or their guards.
Existing lane-change automation can react to the resulting physical indicator,
just as it can to the stalk. This is not a road validation claim.

One accepted request sends the measured left six-frame
or right seven-frame tap/release sequence, then suppresses further requests for
five seconds. Busy/expired requests are dropped rather than queued. Original
stalk/lighting bytes, observed CRC, modulo-15 counter and idle lamp feedback are
checked; counters are reserved before TX and persisted outside the release.
This does not establish an official OEM counter/checksum specification.

Session tokens, sequence checks, startup cutoff, per-device mapping/learning/
disconnect checks and conservative receiver-clock deadlines reject old requests.
CAN uses one-shot transmission and no retry. A driver-state change, stale guard,
CAN error or late schedule aborts. Parent EOF and a separate root lease watchdog
return the owned interface DOWN/listen-only. The watchdog is scoped to its actual
interface index and PEAK driver. Model scheduling, model pins, HUD formats and
production defaults remain unchanged; optional-channel latency is unmeasured.

## Validation scope

Tests cover immediate press versus release, repeat suppression, startup-held
keys, disabled/retained gestures, enabled double/long timing, recorded seven-button
Yiser input, separate command journals, observed CAN vectors and wrap, bounded
six/seven-frame output, busy/replay/expiry/guard/driver-state rejection, session
gates and normal source deadlines. On Jetson, an isolated temporary process test
replaces the CAN device entirely with a fake: normal completion, EOF mid-sequence
and guard expiry all clean up; no physical CAN operation is reachable in that test.

Earlier owner-observed stationary L/R/L/R injections each produced three flashes.
Those were separate manual tests, not validation of this new Bluetooth/USB path.
The earlier two-minute Bluetooth observation found no connected Yiser/input.
Actual remote-to-lamp timing, real adapter hotplug/lease recovery and physical
operation of this integrated release remain to be tested after installation.

Final focused checks: 72 Bluetooth tests pass on Linux, including the actual
daemon loop with fake input pipes/vehicle state, fast/legacy gestures, multiple
remotes and preservation of the input timestamp for physical requests. This uses
a temporary copy of the code and pure-Python pytest dependencies; nothing is
installed into the vehicle runtime. Desktop Bluetooth API coverage also passes.
Eighteen indicator protocol/CAN tests, the existing 48 daemon-timing cases and
25 LPM cases pass on desktop (one native LPM check skipped). The 14 HUD/WLAN tests
also pass. Web build and all 829 web tests pass; the projection test requires the
venv Python path. Ruff and compilation pass for changed Python sources.

The Windows desktop has no native evdev/fcntl or Unix-domain sockets. Broader
phone/USB-owner/Unix-IPC tests still need their Linux environment; partial desktop
platform stubs are not treated as native validation. The current publication runs the Linux integration workflow on the experimental
branch. Final CI and installation results will be recorded after verification. Counts above overlap earlier runs.


## October 25 deployment preparation

34 focused indicator checks pass, including moving/engaged manual operation and
rejection of stale/future/invalid CAN, unknown gear, nonfinite speed, missing panda
state and other fingerprints. The packaged upstream driver source hashes match.
The first end-to-end remote-to-lamp acceptance remains a stationary test; enabling
moving operation does not establish measured road safety or successful road use.


The first live installation exposed a source-loop timestamp-ordering defect:
`SubMaster.update()` stamps newly received messages after the pre-update clock,
so the future-time guard rejected them. Sample the clock again after reception;
retain the future/expiry checks. The regression exercises the actual source loop.
An isolated native source then accepts 40/41 actual vehicle samples (initial
subscription sample is not ready), with no CAN path attached. This correction
runs on comma only; the signed Jetson receiver and N16 pin remain unchanged.


## Installed trial and final stationary observation

Only the experimental branch was published. Signed runtime `277fd6c9a7`
(64,796,760 bytes) was downloaded and hash-verified over its immutable NAS HTTPS
URL. The normal Jetson boot updater installed it after reboot. The bundled
PEAK source built/loaded successfully for the actual kernel; `can1` is owned in
listen-only mode between requests. No source overlay, seeded runtime cache,
new service or permanent modules-load change was used. The original N16 SHA
`01effc67` is unchanged.

Comma installed `39ded79393` through normal Git Pull/restart. Carrot Web's running
API advertises both physical actions and normalized gesture options. The existing
S2 mapping has left/right physical indicators and double/long disabled, saved
through the ordinary guarded web API. Other mapped actions remain as configured.
The temporary offroad holds were restored and their restoration monitors exited.

Final 60.001-second stationary observation: 1,200 valid consecutive model outputs,
1,200 DM/odometry/pose samples, 1,201 main-camera and 1,200 wide-camera samples,
all valid; no model frame gap or unhealthy pose input. External N16, USB HUD and
indicator ready status each hold for all 60 samples. Model execution mean
40.506 ms / maximum 47.645 ms. This is a parked installation check, not an A/B
latency improvement, deadline guarantee or road validation.

CI40173150535 and final correction CI40173798398 pass Linux integration and the
Windows installer check. The initial broad suite passes 1,502/56 skips, plus
62 codec, 94 updater, 119 navigation, 11 storage and 11 NAT checks. The correction
adds the source-loop regression (35 focused indicator tests). Pin CI40173247912
also passes. Counts overlap. Earlier 72 native Bluetooth and 829 web tests remain
applicable; the current web mapping tests/build pass.

The integrated path has not yet transmitted: the S2 is paired but disconnected.
One normal web reconnect attempt timed out. The owner has been asked to wake it,
cycle the stock stalk once to synchronize the prior manual-test counter, and test
left/right while parked. Actual remote-to-lamp acceptance and latency remain
pending; no successful driving/physical-blinker claim is made here.


The owner replied that the flashes were not observed. Subsequent readback still
shows no HID capture/event and zero physical CAN TX packets. Thus there is no
unseen successful trial to infer. PEAK RX continues, TX/RX errors are zero; 51 RX
drops remain unchanged between the two final observations. No physical failure
or success is attributed without an actual connected-remote request.

## Automatic-control extension

The owner reports rapid left/right requests sometimes failing. The original code imposed a five-second cooldown and an all-lamps-off gate, and permanently stopped the worker on an OEM lever/light change. Live telemetry also reported `driver lever/light state changed`.

A parked, continuously guarded 45-frame prototype showed why simply removing the cooldown was insufficient: a right request 0.8 seconds after left did not produce a right lamp transition. Repeated full-lever frames also left residual blinking after cancellation. That implementation was discarded, including its independent neutral-transmission watchdog. A later queued-reversal trial stopped on a stale remote guard before any CAN write. These are failed/partial experiments, not acceptance results.

The final host sends only the previously tested one-touch sequences. A fresh input while its own signal is active replaces the pending intention, waits for actual neutral lamp feedback, and expires after at most 3.2 seconds. The driver's OEM neutral fingerprint, source session, vehicle freshness, device connection and hold cancellation revoke the pending intention. There is no five-second cooldown. Driver changes cancel only the request and return the worker to passive reception.

`AutoBlinkerControl` defaults to 0. Mode 1 adds Bluetooth hold initiated at <=5 km/h and ATC turn-desire indication. Mode 2 also follows ATC laneChangeStarting/Finishing, excluding preLaneChange. DesireHelper publishes maneuver origin without changing admission. The source does not join an episode already active at startup or after stale state/setting changes. Bluetooth completion uses selected-direction steering >=30 degrees, then <=10 degrees for 0.4 seconds while moving >=1 km/h; departure alone does not cancel. Maximum hold: 300 seconds.

USB-CAN starts the signal once. Only a matching host transmission acknowledgement plus observed lamp activation enables the 350 ms local hold lease. `controlsd` forwards this lease in the indicator-only HUD field; the existing Hyundai `ADRV_0x1EA` LEFT_BLINK_HOLD/RIGHT_BLINK_HOLD path supplies maintenance. The isolated native CAN-packer comparison verifies only these fields and their checksum differ. Legacy holds remain outside an owned lease. Same-button cancellation, opposite input, driver override and stale/disabled input release ownership; OEM residual flashes can remain. No repeated B1 held-lever transmission is used.

The source/reader are limited to the explicit Ioniq5PE profile. Capability v2 prevents old hosts accepting the new workflow. Delivery remains the experimental branch, signed host bundle, normal Git Pull and both device restarts; N16 and maintained branches remain unchanged.

Korean/English guides and localized help are updated. Generated setting explanations are staged in `tools/docs/wiki_settings/patches/auto-blinker-control-trial.patch`; public Wiki generation is owned by carrot-wip, which lacks this trial-only setting, so its catalog is not replaced.

Validation before publication: 119 indicator/daemon desktop tests (including 66 indicator tests), 28 exact-source DesireHelper checks on native comma, and 8 native Hyundai hold/packet checks. Native checks use isolated modules, mocked Params and no physical CAN sends. Normal installation, native hold acceptance and queued-reversal acceptance are recorded separately below when completed. No road behavior claim.
