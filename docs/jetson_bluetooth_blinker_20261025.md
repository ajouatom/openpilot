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


## Automatic-control release and final checks

A second guarded queued-reversal trial used conservative receiver-clock samples
without widening freshness gates. It sent 13 frames: left at 1.004 seconds and
right intent at 1.801 seconds. CAN feedback shows three left flashes (onsets
1.127, 1.929 and 2.727 seconds), followed by three right flashes (3.635, 4.416 and
5.221 seconds), then both off. This establishes queued reversal on this stationary
vehicle; it is not CCTV confirmation or immediate reversal while left is active.
The temporary sender restored the interface after the bounded eight-second trial.

Only carrot-nav-blink was published. Source 605e7e2d7c and signed runtime
64,801,709 bytes were fully HTTPS/hash verified; the normal Jetson boot updater
installed this runtime. Normal comma Git Pull/restart installed b1f5fe4fe2,
including the comma-only correction that discards a hold with no confirmed
initial activation after four seconds. A fresh stock override also clears it.
The signed host pin is 7648333636; N16 SHA01effc67 is unchanged. No second host
bundle is necessary for the comma-only policy correction.

CI40176869863, CI40176926962 and CI40177214703 pass Linux integration and Windows
installer checks. Final Linux broad coverage is 1,535 passed/56 skipped, alongside
62 codec, 94 updater, 119 navigation, 11 storage and 11 NAT checks. The last
correction passes 67 focused indicator checks. The two staged Wiki pages validate
and all 25 Wiki tests pass; counts overlap earlier checks.

After normal restart, the actual web API exposes AutoBlinkerControl with values
0 through 2 and current value 0. Both new cereal fields are present. Native model,
indicator protocol v2 and host health return ready/active/ok. A final 60.002-second
stationary observation records 1,200 valid consecutive model outputs and 1,200
valid samples each for DM, main/wide cameras, odometry and pose. No model gap,
unhealthy pose input or parked/inactive guard failure occurs. Model execution
mean 40.188 ms, maximum 49.603 ms. N16, indicator readiness and health are good
60/60 samples; USB HUD is connected 59/60. These measurements do not establish
latency improvement, a hard deadline or road acceptance.

Both temporary installation holds are released/restored and their watchdogs have
exited. IsTakingSnapshot is false, IsOnroad true. Actual PEAK can1 is 500 kbit/s
LISTEN-ONLY, TX packets zero since reboot, RX/TX errors zero. RX drops remain 82
across the two final reads. S2 remains disconnected with no captured buttons.
Automatic-control mode stays 0. Physical Bluetooth hold/cancel, steering-based
completion, ATC turns and actual lane-change indication therefore still need
owner-observed acceptance; no successful physical hold or road claim is made.
Raw CAN, device, timing and installation evidence stays private.


## October 26 audioTurn timing correction

The owner requests ATC turn indication at audioTurn, before turn desire if needed.
SelfdriveD now records a monotonic timestamp and direction in additive
SelfdriveState fields at the exact existing audioTurn event. The event conditions
and audible-alert priority are unchanged; a higher-priority sound does not lose
the cue. Persisting the timestamp avoids losing a one-cycle event at the 20 Hz
indicator source. No onroadEvents polling or stale alert replay is used.

The comma source consumes each fresh cue once, maintains the request while
waiting for turn desire, then releases at the end of the matching turn desire.
Navigation cancellation before entry, opposite turn desire, stale inputs,
disengagement, manual cancellation and existing hold expiry release it. Startup,
reconnect and setting changes do not join old cues. Actual lane-change origin and
mode 2 admission remain unchanged. The common low-speed Bluetooth behavior stays.

Desktop checks: 83 focused protocol/CAN/policy/source tests pass, including actual
SelfdriveD event-method execution, actual source-loop injection, Capnp roundtrip,
pre-turn hold, turn completion, stale/replay suppression and mode-2 regression.
Python lint passes. KO/EN guides, catalog and staged Wiki patch explain the timing.
The installed host605e7e2d7c protocol is unchanged; this correction runs on comma
and does not need a new Jetson runtime or model. Physical acceptance remains pending.


CI40239177298 passes Linux integration (1,551 passed/56 skipped broad coverage,
plus 62 codec, 94 updater, 119 navigation, 11 storage and 11 NAT) and Windows
installer checks. The two staged Wiki pages and 25 Wiki tests pass; the user-docs
validator passes with the explicit unchanged-driver-monitoring exception.
Counts overlap the focused tests.

Normal Git Pull installed ab4fc46f2a on the existing carrot-nav-blink vehicle.
Fresh panda samples confirmed ignitionLine/ignitionCan/controlsAllowed all false,
IsOffroad true and IsOnroad false before pull/restart and after completion. Normal
restart retained the boot account/environment and RT limits. The ordinary native
build completed and manager, UI, panda and Carrot services started. Actual web API
shows the new audioTurn description, and both additive cue schema fields exist.
The owner's current mode 2 was preserved; the agent did not change settings.
IsTakingSnapshot remains absent/false, with no temporary hold used. Jetson pin and
model are unchanged; no Jetson reboot, CAN injection, onroad transition or live
turn was performed. The next real ATC turn still needs lamp acceptance.


## October 26 intersection-stop correction

The owner's latest recorded drive runs the audioTurn source ab4fc46f2a, with AutoBlinkerControl=2 and AlwaysLateral=1. Five closed full-schema rlogs (66,893,681 bytes, independently checked local/remote SHA256) confirm a left cue at route-relative195.697s, lamp state on195.919s, lateral output disabled197.882s at1.05km/h, desire absent197.932s, standstill198.177s, and lamp state off198.409s. Navigation continues to announce the same left turn throughout the approximately62-second stop. These are recorded CAN-derived lamp states, not external video. The old source cancels when latActive drops; it also incorrectly treats turn-desire disappearance as turn completion. The earlier right turn ends its request with steering still approximately-111degrees.

The correction separates a held turn's lateral permission from momentary steering output. New automatic cues and lane changes still require latActive. An already-owned turn can persist while selfdrive remains active or AlwaysLateral is enabled, with latEnabled, no steering fault, fresh vehicle/model/navigation inputs and drive gear. This preserves the owner's AlwaysLateral behavior during a manual longitudinal stop. Driver override, setting/session/stale gates, unconfirmed-start4s expiry and maximum300s hold remain. No controlsd, steering output, CAN payload, receiver protocol, N16 or host pin change is made.

ATC completion requires moving at least1km/h, steering at least30degrees into the indicated turn, at least20degrees of measured body-heading change in that direction, then returning within10degrees for0.4s while moving. Fresh healthy livePose supplies wrapped NED heading; stopped heading drift does not accumulate. A stale/invalid pose cancels the turn without replaying its cue; lane-mode admission and Bluetooth completion stay unchanged. Navigation cancellation before physical entry still cancels; navigation advancement after entry does not truncate the corner. Same-direction repeated cues do not reset progress.

A first unshipped steering-angle-only candidate wrongly ends the left request at261.792s after the departure steering correction peaks near49.5degrees; that candidate was rejected. The revised open-loop replay starts the same right/left cues as the original. Original requests end149.936/197.907s; revised requests end153.188/270.410s after physical heading change and centering. All1,054 replay samples in the selected60-second stopped interval retain the left request. This replays recorded input into the automatic request policy; it does not recreate live feedback, manual stalk ownership, CAN acceptance or physical lamp performance.

Desktop validation passes109 focused indicator/source/protocol checks and25 Wiki checks, including an actual source-loop regression that fails the original implementation, stop/wait/departure-correction cases, both heading-wrap directions, stationary drift, unhealthy pose, cancellation and unchanged lane modes. KO/EN guides, localized catalog and two validated staged trial Wiki pages describe the corrected behavior. Public Wiki remains owned by carrot-wip; only the trial patch is updated here. Integration CI and normal comma-only installation results follow after completion. Raw logs and replay scripts remain private.


CI40264474584 passes Linux broad1,577/56skips,62 codec,94 updater,119 navigation,11 real-storage and11 network checks, plus Windows installer. Production Python lint and the user-docs validator pass. Only carrot-nav-blink is published. Normal Git Pull installs b757f3923c and normal restart.sh completes manager/panda startup. Fresh offroad/panda ignition/control flags are false before deployment and after startup; no artificial onroad transition, snapshot hold or CAN injection is used. Final installed policy/source/catalog hashes match the reviewed PC files. AutoBlinkerControl2 and AlwaysLateral1 are preserved. The605e7e2d7c host pin and N16 remain unchanged, with no Jetson update/reboot. Physical stop-through-turn lamp acceptance and onroad/model-health measurements are not claimed from this offroad installation.
