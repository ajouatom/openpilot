# Jetson USB panel over Wi-Fi; ESP32-S3 retirement (2026-10-20)

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
re-enrollment on subsequent boots. This is **prepared on SD, not yet boot-tested**.

Use the updated carrot-nav on comma, safely remove/reinsert the card, connect the
panel to Jetson's USB host port and keep comma's model USB connection. Boot while
offroad so the dedicated AP can start. First-boot caching/validation may take time.
Physical display, RF recovery, sustained rate and road inference remain untested.

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

No physical Jetson boot, new model equivalence, 20 Hz or driving claim is made.
