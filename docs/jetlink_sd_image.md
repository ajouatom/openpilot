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

Offline checks on the reference Jetson passed full server/HUD dependency
imports, the pinned ONNX SHA-256, and first-boot provisioning in isolated mount
and hostname namespaces. Synthetic Wi-Fi/SSH setup was removed afterwards and
free ext4 blocks zeroed again. A separate miniature loop disk verified mounted
APP growth. `parted --script` refused the in-use partition on this distribution;
the implemented GPT update preserves APP's start, type and UUID, then refreshes
the kernel partition size with `partx` and grows ext4. These checks do not
replace booting the physical spare SD.

An SD image does **not** update QSPI firmware. A fresh or differently configured
Jetson may need matching NVIDIA firmware prepared first. The image is not a
generic original Jetson Nano, Orin NX, NVMe or Mac image. NVIDIA describes SD
image creation and APP expansion in its [flashing documentation](https://docs.nvidia.com/jetson/archives/r36.4.4/DeveloperGuide/SD/FlashingSupport.html).
Our added setup partition and sanitized reference-image workflow require their
own physical SD validation.

## Windows readback and automatic volume changes

The raw image is 24 GiB (25,769,803,776 bytes); its current Zstandard archive is
8,249,576,581 bytes. Keeping both on the PC uses approximately 34 GB. The archive
contains the complete OS, NVIDIA runtime and model, rather than just Carrot.
Routine application updates should not require downloading this entire image.

Windows can modify a freshly written removable image when discovering its GPT
and mounting its FAT setup volume. On the reference 128 GB card, it relocated
the backup GPT to the actual card end, changed the primary header's backup LBA,
last usable LBA and CRC, and added `System Volume Information` to `CARROTSETUP`.
The partition-entry arrays and all three original setup files remained equal
to the image, and both live GPT CRCs and FAT copies were valid.

The Windows writer intentionally still requires exact whole-image readback.
Its `-VerifyOnly` mode avoids rewriting a card after a script failure, but a
Windows-modified card can fail this strict hash comparison. A mismatch is not
permission to skip validation or copy private setup automatically. Diagnose it
with a complete byte comparison, validate the primary and relocated backup GPT
against the original entries and actual disk size, and compare all original FAT
files before treating any differences as Windows metadata. Any difference in
firmware/Linux/model data remains a failure. This manual recovery case means
the Windows writer is not yet an unattended consumer installer. Do not claim
an exact raw-image hash match for a card verified through this separate method.

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

The C4 fault monitor observed approximately **58.7 s** without external-model
activation across shutdown, firmware boot and reconnection. It continued
internal inference, but the existing timeout transition still produced three
missing model frame IDs and a maximum observed receive gap of **205.9 ms**.
Do not describe reboot fallback as maintaining a hard 50 ms deadline. A separate
60 s steady run after recovery recorded 1,200 model frames, mean **37.912 ms**,
maximum **44.393 ms**, with no skipped frames or model/pose/CAN invalidity. DM
remained at the owner's original disabled setting; this is parked validation.

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
