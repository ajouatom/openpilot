# Jetson SD image candidate and update design

Scope: `carrot-jetlink` experiment, Jetson Orin Nano Super developer kit, L4T
36.4.7 / CUDA 12.6 / TensorRT 10.3.0. This does not change carrot-wip or the
existing eGPU runtime. A generated image is a **candidate until tested by booting
the spare SD**; file checks do not establish boot, inference or USB operation.

## Image preparation

`tools/jetlink/build_sd_image.py` runs on the prepared reference Jetson as root.
It checks the reference SD layout and creates a 24 GiB sparse image under
`/home/yun/jetlink-image-build`, never writing the running physical SD. It copies
required boot partitions, creates a new ext4 APP, and copies OS/runtime files
with personal home directories, keys, network configuration, logs, package
caches and device state excluded. Dpkg locks and a before/after status hash
prevent a concurrent package transaction from silently changing the snapshot.

`finalize_sd_image.py` installs a checksum-verified, committed host bundle.
Source lives in `/opt/carrot-jetlink/releases/<commit>`, selected by `current`;
the existing `carrot` alias supports inventory tools. The prepared venv and
model cache are retained, with entry-point paths migrated. Services use the
new `jetlink` account and the control socket lives under `/run/carrot-jetlink`.
It removes remaining per-device state, creates locked accounts, clears machine
identity and SSH host keys, checks filesystem/imports/sudoers, and zeros free
ext4 blocks. Build artifacts and provisioned cards are never automatically
published. Preserve redistribution notices and confirm the OS/model/runtime
licenses before making a public download available.

The FAT `CARROTSETUP` partition contains an example configuration. Copy it to
`setup.json`, supply the owner's SSH **public** key, and optionally Wi-Fi SSID
and WPA passphrase. Remove `wifi` for Ethernet DHCP. The public image has no
personal key, Wi-Fi credentials or shared password. The owner's configured
`jetlink` SSH account is an administrator with passwordless sudo. Keep the
private SSH key on the owner's computer.

At first boot, `image_first_boot.py` generates host keys, assigns a unique default
hostname, imports configuration, removes `setup.json`, and writes a non-secret
result file. Setup is retried after failure. If no SSH key was supplied, it
allows configuration on a subsequent boot. The SD and its unallocated space
must not be redistributed after personal configuration. Removing a FAT file
is not secure erasure of its previous contents.

`image_grow_root.py` verifies the marker, exact SD device, expected partition
start and that APP is the last physical partition before moving the backup GPT
and extending APP. It never selects another USB disk by label. An interrupted
growth is retried until successful. Provisioning and expansion are first-boot
work; normal ignition startup does not install packages, download models or
compile TensorRT engines.

An SD image does **not** update QSPI firmware. A fresh or differently configured
Jetson may need matching NVIDIA firmware prepared first. The image is not a
generic original Jetson Nano, Orin NX, NVMe or Mac image. NVIDIA describes SD
image creation and APP expansion in its [flashing documentation](https://docs.nvidia.com/jetson/archives/r36.4.4/DeveloperGuide/SD/FlashingSupport.html).
Our added setup partition and sanitized reference-image workflow require their
own physical SD validation.

## Additional boot experiment

The installed NVIDIA utmp override adds `/bin/sleep 2` before sysinit completes.
`configure_utmp_delay.py` removes that delay only when it is the sole expected
pre-start command. `--restore` removes our override. NVIDIA explains that it
corrects the timestamp printed by `last reboot -F`; removing it may make that
history timestamp less accurate ([NVIDIA staff explanation](https://forums.developer.nvidia.com/t/why-systemd-update-utmp-service-need-sleep-2s/363767)).
It does not remove driver/power ordering or inference-readiness checks. The
previous measured baseline is 16.485 s Linux startup, 31.084 s engine-ready and
39.626 s HUD-ready; firmware time is outside those clocks. New results must be
measured, not inferred by subtracting two seconds.

The first parked reboot with the override measured Linux **13.699 s**, engine
**25.938 s**, and first USB HUD refresh **34.190 s**, with no failed systemd
units. Kernel time also varied (8.152 → 6.861 s), so the entire observed gain
cannot be attributed solely to the removed two-second sleep. This is one warm
reboot, not a repeated ignition/cold-boot guarantee.

## C4-triggered updates: proposed protocol, not yet implemented

The recommended control flow is C4 request → Jetson download/stage → explicit
parked maintenance → activate → validate → commit or rollback. Jetson uses its
own network connection for bundles. C4 does not stream update archives over
the inference USB channel or require a shared SSH password.

1. `check/status`: expose host source version, compatibility, available release,
   download progress and last failure through a bounded protocol extension.
   Keep the existing USB owner in charge; a C4 CLI talks to its local management
   socket rather than opening a second competing USB session.
2. `prepare`: retrieve a signed release manifest from a pinned HTTPS origin;
   verify a pinned signing key, exact bundle hash, size and compatibility.
   SHA-256 sidecars alone prove integrity, not publisher authenticity. Stage a
   separate immutable release and versioned venv if dependencies change.
   Model download/engine preparation is maintenance work; do not run GPU builds
   alongside driving inference. An interrupted download can resume after
   validating ranges; no active files are overwritten.
3. `apply`: require fresh stopped and fully disengaged vehicle state, a C4
   native-model fallback acknowledgement and an explicit maintenance lease.
   The ordinary `stopped OR steeringPressed` model-join rule is **not** permission
   to update while moving or controlling the vehicle. Loss of the lease before
   activation cancels activation. Prepared files alone never trigger a switch.
4. Save a durable transaction record and previous release/venv, atomically
   switch `current`, restart the host, and keep C4 on the internal model while
   checking actual engine readiness, finite inference outputs and USB handshake.
   Systemd `active` alone is insufficient. Failed health checks or a power cut
   during an uncommitted transaction restore the previous complete release on
   the next boot. Rejoining still follows the normal validated join policy.
5. Updating JetPack, TensorRT, kernel or QSPI is a separate image/maintenance
   operation. Routine C4 updates should initially cover application sources
   with an unchanged validated OS/model contract.

The image's version-directory layout prepares for this design; it does not by
itself implement signed releases, a C4 update command, the maintenance lease,
engine health gating or automatic rollback. Those need protocol tests, failure
injection and parked device validation before being advertised as available.
