# Jetson USB panel over Wi-Fi; ESP32-S3 retirement (2026-10-20)

The Wi-Fi-only display policy below is superseded by the October 21
[bounded USB restoration](jetson_usb_hud_restore_20261021.md). ESP32-S3 remains
retired; this page preserves the earlier investigation.

The owner explicitly requests removing ESP32-S3 display support and using a USB
panel attached to Jetson. The requested topology is:

    comma -- dedicated Wi-Fi HUD/navigation --> Jetson -- USB --> TURZX panel
    comma -- existing USB model/provisioning --> Jetson

ESP32-S3 discovery, network display, codec overrides, pairing/provisioning,
firmware sources, web installer and download route are removed on both maintained
branches. Existing pairing files are no longer read; no device credential deletion
or remote device reboot is performed. Historical investigation records remain.
The shared renderer now constructs TuringUsbDisplay directly; Jetson's existing
USB presence/hotplug/status wrapper therefore remains in effect. Independent
on/off, ignition, debug, brightness and model control policies are preserved.

The new comma-to-Jetson Wi-Fi runtime is on **carrot-nav**. Contributor source
b09e20a1 is integrated as c93e05dcba, with USB HUD fallback removed at both ends
and all Jetson display pumps skipped in the inference loop, even for old peers.
The dedicated AP is created only offroad; separate encrypted/authenticated TCP
snapshot and media streams do not wait inside inference. A failed/missing WLAN
path cannot fall back to USB display. Existing non-Jetson hosts retain their
previous transport. The wip branch receives S3 retirement only and keeps its
separate model/runtime selection. See jetson_wifi_review.md on carrot-nav for
implementation and contributor evidence, distinguished from this integration.

## Release and card

Signed runtime: b7b463a20b597e0b1b4c0f1cb0375f21cf47ae19.
Bundle: 64,670,114 bytes, SHA-256
9755e63336425983b722ac274603fb22216ecb8ea0d4d78d9e5ba20b49fa18e7.
Immutable NAS download was read back and matched its signature/size/hash.
The nav pin selects this runtime. Jetson ONNX remains 80eedcdd/12864, SHA-256
1d18c42ea4cd95c8bffab4635844b78b8e2d4e25b9de44a9c5ac2e0961371a2e.
The latest n16 ONNX experiment remains deferred; eGPU pins are unchanged.

The owner's attached SD was identified by serial, full device identity, size and
GPT. A 25,767-byte hash-pinned enrollment ZIP plus the signed bundle were copied
to CARROTSETUP. Only 6,144 bytes covering the existing 5,756-byte APP first-boot
script were rewritten, with the original script embedded verbatim and original
sector padding retained. No inode, allocation, partition table or DATA writes.
DATA's pending journal is left for Linux's existing repair/mount procedure.
The independent ext4 readback matches the patched hash
4a7d9ce2add6223c154436dfa9016804fc3d5a0c1fbe66fe0129c0def26d2bf7;
GPT and DATA updater/key bytes still match, and the existing pinned ONNX file
has 814,591,106 bytes (full model hash is checked by the normal boot updater).
Private backups, disk identity and deployment scripts remain outside Git.

On the next protected boot, the one-time hook backs up the updater, verifies
and caches the signed runtime, and installs the previously requested USB-update
receiver. The hook preserves the current runtime/model/key; only the existing
boot gate activates the comma-selected signed release after its native probe.
No USB-delivered release is selected. A root-owned completion marker prevents
re-enrollment on subsequent boots. At this preparation stage it was not yet boot-tested; see the hardware follow-up below.

Use the updated carrot-nav on comma, safely remove/reinsert the card, connect the
panel to Jetson's USB host port and keep comma's model USB connection. Boot while
offroad so the dedicated AP can start. First-boot caching/validation may take time.
At preparation, physical display, RF recovery, sustained rate and road inference were untested.

## Validation

- Desktop Wi-Fi/USB-policy integration: 52 passed, including actual loopback
  authenticated encrypted records, tamper/replay rejection, missing/failing
  negotiation, both receiver states and model reply without display pump calls.
- S3 retirement / USB panel / lifecycle / viewer: 41 passed, 4 Windows symlink
  skips; native Params/hardware imports doubled on desktop.
- Linux CI39359804302: 94 root update cases, 845 broad checks (55 skips), 119
  nav/cluster cases, 11 actual protected-storage loop filesystem cases; Windows
  installer and eGPU compatibility CI39359804354 also pass. Counts overlap.
- SD helper replay uses the real signed bundle twice, validates unchanged current
  and trust root, stable original backup and verified cache. POSIX ownership and
  symlink operations are doubled; native receiver ownership checks run in CI.

The above preparation checks alone make no physical boot, new-model equivalence,
20 Hz or driving claim. The later bounded hardware observation is recorded below.

## Owner hardware follow-up

The owner subsequently connected the Jetson and reported a default-only display
with ignition on in Park. Read-only inspection found the SD enrollment/cache
completed correctly, but the current runtime was still 989386eb8571. Its earlier
boot gate had approved that release before the current comma boot. The updated
comma correctly refused legacy USB HUD, and its new dedicated AP was waiting for
an actual offroad state; Park alone does not satisfy that condition.

The owner explicitly authorized a temporary offroad transition. Fresh valid/live
vehicle messages confirmed Park, standstill, zero speed, disabled/inactive
selfdrive and carControl, and all pandas disallowing controls. The existing
IsTakingSnapshot startup hold plus OnroadCycleRequested produced a real
hardwared/manager offroad transition. A separate 30-minute restoration watchdog
was armed; IsOnroad/IsOffroad were never forged. The dedicated AP then started.
After verifying fresh offroad provisioning and the signed cached bundle hash,
Jetson was rebooted through its normal boot updater. Candidate native inference
passed, the selected b7b463a20b runtime was activated, protected storage remained
read-only, and Jetson joined the dedicated WLAN automatically. The temporary
hold was removed and ordinary onroad operation resumed; IsTakingSnapshot was
restored to its original absent state, with ClusterHud still 1.

On the owner's physical devices, USB/H.264 renderer logs report 10.0 Hz, actual
Park/vehicle values and 26/26 live subscriptions. The renderer's "no-can" label
means no raw CAN subscription, not missing carState. All 12 Jetson samples over
a 60-second observation had connected snapshot/media sockets and fresh USB-send
status; all 13 concurrent comma samples showed the AP active and Wi-Fi HUD
connected. Final ClusterHudConnected was true. Navigation media had zero bytes
and no map frame, so this observation does not validate map rendering. Physical
panel appearance was not independently photographed or confirmed by the owner.

Concurrent 60.001 seconds yielded 1,200 valid consecutive modelV2 frames at
19.9981199 Hz and 1,200 valid consecutive driverStateV2 frames at 19.9985720 Hz.
This is approximately 20 Hz, not a strict >=20 Hz pass. The original 80eedcdd
Jetson ONNX remained active throughout that final interval; no latest-model
replacement, road test, RF recovery or long-duration stability claim is made.
Raw device/connection evidence remains private. This hardware follow-up applies
to carrot-nav; wip still retains its separate runtime selection. No new complete
SD image or Windows package was built by this device operation.
