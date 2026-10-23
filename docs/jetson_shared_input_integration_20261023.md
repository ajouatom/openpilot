# Shared RAM input and owned DHCP shutdown integration - 2026-10-23

The owner requested review, integration and device validation of contributor
[6914f6ce](https://github.com/tjdgus200/openpilot/commit/6914f6ce5a236b606defff5097546b009e50b47a)
and [fbe9a887](https://github.com/tjdgus200/openpilot/commit/fbe9a887d0346e0d0eb136689010be181a4e3788).

## Accepted scope

On carrot-nav, modeld and jetlinkd negotiate one anonymous, size-sealed memfd
per Unix connection. GPU readback fills that connection's image destination;
scalar context is copied to a disjoint region. The local socket carries a
56-byte token plus four-byte framing instead of the 655,596-byte N16 input
payload plus request/framing headers. USB framing, image/context values, N16,
GPU recurrence, HUD/display traffic and reply validation are unchanged.

The daemon maps read-only; the producer exclusively owns writes until an exact,
finite, correctly sized reply releases the slot. Nonce, monotonic generation,
frame/reset flags and 150 ms token age are checked. Timeout, malformed reply or
other uncertain completion retires the connection; a late consumer retains its
old mapping, and reconnecting cannot reuse that allocation. File descriptors
are closed after mapping and on error. Legacy peers and explicit
JETLINK_RAM_INPUT=0 retain the socket path. No asynchronous USB is introduced.

Review addition: an invalid/read-only warp destination must still drain the
already-submitted GPU operation before local fallback. The contributor's early
validation raise skipped that fence; corrected and covered by failure tests.
Add two scalars to existing once-per-second runtime timing aggregates:
ipc_request_bytes (including four-byte prefix) and ipc_shared_input. These
permit actual default-activation verification without per-frame payload logging.

DHCP cleanup waits up to five seconds for the already verified process to exit,
returning immediately for fast exit. Executable/arguments/PID/start-time checks,
pre-signal identity verification, SIGTERM-only behavior and live-process refusal
remain. Disappearance after validation is successful cleanup. No USB rebind or
ownership guard is bypassed. This changes recovery, not inference execution.

Carrot-wip receives the common DHCP correction and documentation only. Its
separate queued-model implementation does not receive the N16 IPC integration.
No new Jetson bundle/model pin is needed: normal comma Git Pull and reboot
activate these comma-side changes; the signed signal-display/N16 host remains.

## Verification

Windows suite: 108 passed, 52 Linux/POSIX skips including the four actual
daemon/socket integration cases delegated to Linux CI. Native SCM_RIGHTS/memfd tests cover exact bytes,
1,000 generations, old-reader isolation over 100 reconnects, FD/map lifetimes,
malformed descriptors, legacy negotiation, timeout/bad reply and frame wrap.
Additional tests run actual daemon/client framing through a fake USB endpoint,
full/projected replies and interleaved reserved/legacy payloads. Copyout/fence
checks cover shared destinations and context/destination failure. DHCP cases
exercise both pidfd and legacy /proc ownership/exit paths on Linux.

Before application, the vehicle was freshly confirmed P/zero/inactive. A
120.066-second read-only baseline had 2,401 valid consecutive model outputs,
mean 38.852 ms, P99 42.717 ms, max 45.374 ms, zero >50 ms; model/DM/cameras/pose
were healthy. No navigation route was present. This is our current baseline,
not the contributor's detached-bench ABBA result.

Contributor timings imply mean 36.941302 -> 36.350218 ms (0.591084 ms saved),
P99 37.713840 -> 37.072065 ms (0.641775 ms saved). Their detached bench lacked a
physical HUD panel. Do not describe 0.64 ms as their mean gain or our measured
gain. Their DHCP reproduction and 1,200 hashes are contributor evidence, not
our device validation. See imported [RAM](jetlink_shared_input_20261023.md) and
[DHCP](jetlink_dhcp_cleanup_20261023.md) notes for original scope and limitations.

## Integration and native checks

NAV source f1a9626c19 first passed the real memfd/daemon/DHCP tests, but the
broad suite exposed three old selected-model fixtures without the new warp
buffer interface (1,196 passed, 56 skipped, 3 failed). Correct and strengthen
those fixtures to cover shared and legacy destinations across all three model
contracts in 819ce67c7d. Final Linux run 39947773540 passes 94 updater, 1,202
broad (56 skips), 119 navigation, 11 real-storage and 11 real-NAT checks.
Wip a7658f4b6a run 39947474483 passes 820 broad (2 skips), 119 navigation and
11 real-storage checks. Suite counts overlap.

Fresh park/zero/inactive/panda guards permitted a bounded snapshot maintenance
hold, then ordinary ff-only Git Pull to 819ce67c7d. During actual offroad with
model/DM/camera stopped, 64 actual QCOM operations compared the same submitted
result's copyout with numpy readback, and checked full image/context receiver
hashes. All matched; four reconnect nonces were distinct and four retained old
reader buffers were unchanged. A diagnostic-only inherited root cache path
failed on the first attempt before measurement; an explicit private CACHEDB
fixed the diagnostic environment. This is synthetic native copy validation,
not a full camera-warp/model equivalence trial.

## Diagnostic interruption and recovery

The first reboot cleared the temporary snapshot hold normally. However, an
observer started as root before operational publishers created two ordinary
msgq files (cameraOdometry/livePose) owned by root, mode 0644. This denied the
comma processes access and caused restarts; it was an agent-induced diagnostic
error, unrelated to the anonymous jetlink memfd. Preserve that failed window,
exclude it from performance claims. Verify the exact two regular files' UID,
GID and creation timestamp before restoring their owner to comma. All later
messaging observers drop to the comma account before opening subscribers.

Operational processes recovered. A 120.067 s interim window had 2,400 valid
consecutive model outputs, mean36.874/P9939.662/max42.709 ms, zero >50; DM,
both cameras, odometry and pose were healthy. HUD publication was still absent
because its original child had exited during the permission fault. This window
is NOT a like-for-like speed comparison. A second freshly guarded normal reboot
restarts the entire session including the HUD child; final results follow.

## Naver route inspection

The sender identifies itself as naver-6.10.0.16-nm127-release. The receiver has
an active v2 control connection and increasing valid message counts, with no
receiver or cereal errors. The sender reports guidance_active=true but
route_present=false/off_route=true, and explicitly sends route/current+next
guidance/vehicle items absent with reason=expired. record_json retains that
sender reason; the receiver did not invent this expiry. No render:map_main
stream was present in that inspected snapshot.

Thirty seconds of observation had zero route coordinates and zero maneuvers;
the N16 model was active. Read-only inspection found modeld writable and
jetlinkd read-only mappings of the same memfd inode, with matching zero-valued
nav_map/navigation fields. Two consecutive reads agreed in the recorded
samples, but these unsynchronized snapshots are not a per-frame USB/engine
input hash trace. A fresh valid Naver route has not yet been demonstrated;
never describe an active N16 model as proof that navigation is being used.
The owner supplied a local APK for further diagnosis (details below). Do not bypass
route/age/off-route gates or substitute stale geometry.

## Final normal boot and timing

Second normal reboot runs unchanged source 819ce67c7d, clears the maintenance
hold, and restores all monitored model/vehicle/pose services and the HUD child.
Jetson has not rebooted or changed release: 54e706fa05 with the same N16 SHA.
The anonymous input mapping is writable in modeld and read-only in jetlinkd,
with the same inode. Across 2,402 calls represented by timing aggregates,
ipc_request_bytes is exactly60 and ipc_shared_input exactly1. USB framing and
transport remain unchanged. The 60-byte claim is local IPC, not total USB data.

The final 120.071 s window contains 2,400 valid consecutive model outputs at
19.9974Hz, mean37.8447/P9941.5557/max43.6924 ms and zero >50 ms. DM has2,399,
both cameras/odometry/pose each2,400 valid consecutive observations, with no
unhealthy pose. Link ready and N16 active120/120; signal source fresh120/120.
HUD connected119/120 (one false sample at index86); retain that exception.
Jetson subsequently reports fresh snapshots with both camera frames and fresh
signal metadata; no actual signal tracks were present in the parked scene.

Compared with the pre-update no-route baseline38.8522/P9942.7172/max45.3736,
the observed mean is1.0075 ms lower and P99 is1.1615 ms lower. This is a reboot
comparison with thermal/startup confounding, not a controlled ABBA attribution.
Server GPU mean24.6764 ms, server total25.5079 ms, USB exchange31.4184 ms;
HUD snapshots averaged16,595 bytes per model cycle and media payloads zero.
NAV reason remained route120/120, so this does not validate active navigation,
real road stability, a universal50 ms deadline or all locationd errors.

DHCP reconnect completed through the normal boot path. The contributor's exact
three-second ICMP-probe condition was not forced on this vehicle; the changed
five-second identity-preserving exit behavior is covered by19 Linux tests.

## Supplied Naver APK

Read-only inspection of CarrotNM-1.28-ROUTE-GPSFIX.apk (149,893,361 bytes,
SHA2562a0925ae87b032eed249713374576a93056564ee1599fdf15fa7f676443c2870)
finds package com.nhn.android.nmap, versionName6.10.0.16/versionCode61000009.
Its Bridge control URL and diagnostics still contain nm127-release, so that
network label alone does not prove the installed app is an older APK. The
installed APK bytes have not been retrieved and compared.

DEX inspection locates connectedStatus() constructing guidance_active=true,
off_route=true and route_present=false as a fallback state. Consequently the
received off_route flag does NOT establish that the parked vehicle is actually
off its intended route. JsonStreams.Worker.run in classes46 retains a2,000 ms
snapshot-age gate that emits Snapshot.absent("expired"); completeGuidance in
classes44 also rejects extraction completion older than2,000 ms. The main
Bridge run loop has a branch bypass around its separate age check, but that
does not bypass the independent JSON worker gate. Do not patch these gates
away: investigate source callback/extraction/refresh and session invalidation.

After the final comma reboot, the sender reconnects automatically without a
receiver parse error but still sends absent route/guidance. map_main likewise
reports absent/expired. No Android ADB device is currently attached; runtime
app callback/extraction logs are therefore not available. The exact reason for
snapshot updates stopping is not proven by APK inspection alone. No APK or
route-validity code was changed, and valid Naver route-to-model verification
remains incomplete. Raw vehicle/device/APK-derived evidence stays private
under .analysis; it is not included in the public repository.


A final additional60.035 s observation has1,200 valid consecutive model/DM/
both-camera/odometry frames and1,201 healthy pose messages. Model mean37.8335,
P9941.4014,max44.1009 ms,zero >50. Link/N16/HUD/signal are fresh60/60; the
independent simultaneous Jetson renderer observer also has60/60 fresh online
signal-display states. The earlier one-sample HUD lapse remains recorded; this
is bounded recovery evidence, not proof that display freshness never lapses.


## Follow-up: Naver guidance arrives without another code change

The owner then reported Naver content appearing. The same sender session now
publishes mode=guidance, guidance_active=true, off_route=false, route_present=true,
17 route coordinates (1.2km lookahead), valid vehicle fixes and fresh current/
next TBT. The current maneuver is a left turn1,064m ahead. These supersede the
earlier absent-input observation; the exact action/environment that restored
sender updates remains unknown. Neither receiver nor APK was changed.

In30 sampled seconds, mapsd reason=ready30/30 and N16 active30/30. Fresh
navInstructionCarrot contains one maneuver30/30. The observer initially had
not received the periodic navRoute publication, then sees17points29/29.
Actual modeld/jetlinkd shared input views have matching finite nav_map and
navigation hashes30/30; each double read agrees. nav_map has5,955 nonzero
pixels (range0..255), navigation has2nonzero values, including distance10.64
in the existing100m encoding. This demonstrates populated production model
input and daemon input, not an independently captured GPU engine input trace
or a road maneuver outcome.

The actual Jetson screen receiver advances960x540 map sequences592->1157,
with age81->30ms and no stalled flag. Cumulative decoder drops remain11 in
both snapshots; do not describe the entire stream as lossless. The separate
nav_model_map preview reports false because the user has screen mode0; its
receiver explicitly gates that preview on mode7. This is not evidence of an
empty model input. Signal/HUD snapshots and both camera previews remain fresh.


The subsequent60.030s active-NAV/video window has1,200 valid consecutive model,
DM, both cameras, odometry and healthy pose messages. Model mean37.8972,
P9942.0846,max43.4481 ms; zero >50. Link/N16/HUD/signal60/60 fresh. This is
bounded parked active-NAV evidence, not a causal timing comparison or road
validation. No additional code or device changes were required for arrival.
