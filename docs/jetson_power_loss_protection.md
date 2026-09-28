# Jetson storage protection candidate (2026-09-28)

This is a candidate implementation, **not an installed or released read-only
image**. The public v0.3.2 Windows installer still contains a writable APP
filesystem. Do not describe a source push or an isolated filesystem test as a
completed vehicle conversion.

## Required behavior

- Normal operation must not write the system filesystem. Logs and temporary
  OS state use bounded RAM mounts. The base OS and a factory runtime remain
  available independently of updateable storage.
- A known Wi-Fi network must reconnect without a comma, USB session, model,
  successful update, or Internet access. An unknown SSID still needs provisioning
  over USB or the local setup partition; software cannot discover its password.
- A failed credential change retains the previous profiles. Wi-Fi recovery is
  independent of the runtime activation service. No open recovery AP or default
  password is introduced.
- Power interruption during identity writes must retain a previous complete
  record. Runtime updates continue to verify signatures, hashes, runtime ABI and
  a candidate engine before activation; transaction recovery retains the old
  release. These checks do not upgrade the base OS or change Cinque v2.

## Storage layout

`build_protected_image.py` accepts only a hash-verified, pristine 24 GiB raw image
and a hash-verified committed bundle. It creates a **new 40 GiB file**, preserving
APP and firmware partition offsets and adding partition 17, CARROTDATA, in the
new space. Only a loop device backed by that exact output is written. It never
opens a physical SD for writing or resizes the running root.

APP mounts `ro` from the kernel command line and fstab. Early boot additionally
sets the APP block device read-only. RAM overlay upper layers cover `/etc`,
`/var`, `/home`, `/root`; `/tmp` and `/var/log` have separate bounded tmpfs mounts.
The helper never remounts APP writable. Firmware ESP is also mounted read-only.
Legacy APP expansion is disabled; the initial DATA capacity is fixed at about
16 GiB. Automatic DATA growth is deliberately not implemented yet.

DATA contains the complete updateable `/opt/carrot-jetlink`, including its cache,
release symlink and update transaction files. It is checked while unmounted,
then bound onto the runtime path only when its basic layout is complete. An
unrepairable DATA filesystem or incomplete runtime falls back to the immutable
factory runtime. This is structural recovery, not exhaustive model/venv bit-rot
detection. No partition is automatically formatted and no user state erased.

The stable network worker and storage helper reside in the base OS. They do not
import the model, CUDA, or candidate runtime. Runtime activation cannot prevent
the network worker starting. Updates are disabled in base-recovery mode rather
than appearing to install into volatile storage.

The signed update manifest must explicitly declare `storage_format: 1` on a
protected image. An older comma selecting a pre-protection release cannot
downgrade its runtime. Legacy images still accept their original signed releases.
The publisher derives this field from the committed bundle's compatibility file.

## Identity and credentials

Hostname, SSH host keys, authorized public keys and NetworkManager client
profiles are explicitly allowlisted. Each private record has a generation and
SHA-256, with two alternating slots on DATA and a second copy on CARROTSETUP.
File and directory fsync precede completion. A damaged/truncated newest record
falls back to another complete record. Unchanged state is not rewritten.
Checksums detect corruption, not a malicious writer with physical root access.

No identity records belong in a public image, Git, diagnostics, or NAS download.
First boot generates individual keys; logs and live status contain no passwords.
PID1's machine-id remains a per-boot volatile identity; the storage helper does
not replace it with a different ID after systemd has initialized. SSH host keys
and the configured hostname are restored separately.

## Existing installations

Existing cards have already expanded APP to the end of the SD. A safe separate
DATA partition cannot be created there merely by enabling an overlay or pulling
Git. This implementation **does not automatically repartition those cards**.
Wi-Fi recovery improvements can be delivered as a runtime patch; complete OS
storage isolation requires the new image and a physical boot test. Do not
promise read-only protection for the legacy layout.

The original `image_first_boot.py` remains byte-pinned for the already released
in-place USB-C hotfix. Protected images use `protected_first_boot.py`; changing
the old payload would invalidate its exact original-image hash/extent contract.

## Validation and release gate

Local Windows suite: 149 passed, 12 skipped. Linux CI on source `8d5b944e06`
passed 188 tests (1 skipped), 119 navigation/renderer checks, and 11 real-loop
storage tests; Windows disk-selection checks also passed. The host mirror's
114 tests passed (1 skipped). Identity fault-injection includes
interruption before/after replacement, corrupt newest slots, cross-partition
fallback and no writes for unchanged state. Wi-Fi tests cover no USB, empty
profile sets, rejected credentials and worker restart during replacement.

On the parked reference Jetson, an isolated mount namespace and a **new 64 MiB
RAM-backed loop image** exercised the real ext4/overlay/tmpfs implementation.
Two mount cycles discarded transient changes, rejected writes to the base OS,
and retained the exact base image hash. This did not reboot the Jetson, install
the candidate, cut vehicle power or test SD controller failure.

The reference legacy Jetson received the independent network worker and stable
updater while retaining runtime `f2b22dcf`. A deliberate NetworkManager disconnect
followed by `Worker.step(None, ...)` recovered a saved connection in 0.549 seconds.
No USB provisioning packet was supplied to that worker; the physical cable
remained connected. Inference and HUD were not restarted. This tests recovery of
a known available network, not availability of an absent access point.

A 120-second parked C4 observation during the image workload recorded 2,400
model/odometry/pose/road messages and 2,401 wide messages, no invalidity or frame
gaps, maximum model execution 42.622 ms and road/wide gaps 57.448/57.449 ms.
Temperature was 69.156–70.031 C, speed zero and assistance disengaged. This is
parked validation only. The monitor's reused scope label is historical; this run
does not establish a new USB role-change result.

Systemd dependency checks against the reference image passed. Still required
before publication: physical SD boot, persistence of Wi-Fi/SSH
across boots without USB, failed DATA recovery, signed update/rollback on DATA,
bounded log behavior, and repeated parked power interruption tests. Physical SD
controller failure and electrical damage can still require reflashing/replacing
the card even when the filesystem is read-only.

## Full image candidate audit

The final candidate is built from the pristine base and committed runtime
`8d5b944e0643566e15b1a0868443298859a642fb`, never from the provisioned live SD.
Raw size: 42,949,672,960 bytes; SHA256:
`0802e60fe1886d53aae3881828a422819fe611d14f11fbaaee8296adeee1d5b9`.
Compressed size: 10,054,161,819 bytes; SHA256:
`1014dc56a8f595c8133e13f477e6b0962487a27cd194354fa241d8334676060c`.

The actual candidate's helper was executed in separate mount/UTS/PID namespaces
and chroots for normal DATA and simulated unavailable DATA. Each loaded the
complete runtime imports, rejected `/usr` writes with EROFS and accepted RAM
writes; the normal path also accepted a temporary DATA write. No physical SD
device or GPU/USB device was exposed inside those chroots. Both factory and DATA
model hashes, unprovisioned identity checks, filesystem checks and GPT passed.
This does not execute PID1's full boot sequence, prove inference in recovery, or
simulate electrical failure. Physical candidate validation remains outstanding.

The user selected a complete replacement image release for this storage change.
The integrated Windows preparation mode verifies/extracts the finished image;
it does not apply a separate legacy USB-C patch. Future runtime/model updates
continue through the comma-selected signed channel.

The private Windows ZIP is 10,078,859,286 bytes, SHA256
`1dba6bacbaebdf8ce25bbabc10710a359c354e5c5a08733de9fec4b4140c637d`.
The complete ZIP was extracted with CRC checks, then its actual `01` CMD and
bundled Python prepared the full 40 GiB image in a Korean/space path. The final
raw SHA256 matched independently; the run took 140 seconds on the reference PC.
No legacy patch file is included or applied. Both compressed image and ZIP were
copied to the private NAS candidate directory and fully read back for SHA256.
Physical card writing and boot are still pending; the public v0.3.2 download and
the comma's existing signed runtime pin have not been replaced.

Docs-Not-Needed: Engineering candidate only; no new user setting or released
installation procedure. Public beginner instructions change only with a tested
installer artifact.

## Physical candidate failure and USB diagnostics (2026-09-28)

The owner card passed all 40 GiB of write/readback comparison before private
public-key provisioning. The subsequent physical trial did **not** establish a
working Jetson: comma reported USB not attached and the host model inactive,
while native model/pose messages remained valid at rest. SSH was unavailable.
The user has only a TURZX USB panel, not a DisplayPort boot console.

Read-only inspection after returning the card to the PC found completed
provisioning and checksum-valid identity generations 1 and 2. Generation 2
contains two Wi-Fi profiles; the provisioning worker saves this generation only
after NetworkManager reports a selected imported connection active. This shows
the first run reached provisioning/network setup; it does not establish which
later stage failed or why subsequent USB/SSH access disappeared. No raw profile
or identity content belongs in this document or a public artifact.

There is also a separate confirmed protection gap: NVIDIA's embedded initrd
mounts the non-overlay root writable despite the kernel `ro` token, before
systemd later remounts it. The card's formerly empty base machine-id was written.
The current image must **not** be advertised as protected from the first mount.
Fixing and physically validating the initrd path remains a release requirement;
this finding alone is not proof of the observed connection failure's cause.

The diagnostic candidate adds a CPU/Pillow JPEG screen independent of comma
snapshots, Xorg and inference. It shows storage/boot stage, Wi-Fi, IP, SSH and
service status, temperature and boot elapsed time. A live service is not labelled
model-ready. A shared process lock hands the panel to the regular HUD; a killed
HUD releases it, and stale requests cannot survive PID reuse. Diagnostics never
reset the USB device and run at nice19/one frame per second. The diagnostic code
is pinned outside updateable releases in new images, but still needs the supplied
Python environment, working kernel/USB and a supported panel. It cannot display
firmware, power or pre-userspace failures, nor take over a live wedged HUD's lock.

Storage boot also records its stage/error class in RAM and attempts one bounded
`BOOT-STATUS.json` write on CARROTSETUP at completion/failure. It excludes profiles,
keys and journals. Failure before this helper starts can leave an older result;
the boot ID distinguishes recorded runs. The first failing card predates this
recording, so its original transient logs are unavailable.

Windows tests and CPU rendering are not USB-panel or new-image boot validation.
An owner-card diagnostic patch is being prepared separately from public artifacts;
the NAS public image and signed automatic update channel remain unchanged.

### Recovered connection and full-boot findings

After the owner reconnected USB, the same candidate booted with fresh USB ready
state, active Cinque v2 inference, Wi-Fi and SSH. This does not prove the earlier
failure was a defective cable. Live root and root block device were read-only;
etc/var/home/root used RAM overlays, logs used tmpfs, and the runtime used DATA.
This verifies the post-helper state, not protection during the initial initrd mount.

The full boot exposed NVIDIA `nv.sh` attempting to recreate already-correct
Weston/Wayland symlinks on the immutable root. Its failure blocked nvpmodel,
performance setup, Xorg and the normal USB HUD. The candidate image installer
now makes those link operations idempotent: matching links need no write;
missing/incorrect links retain their write failure. NVIDIA runtime initialization
is retained. A RAM-only trial on the owner's Jetson made nv.service succeed.
Because inference had already initialized the GPU, retrying nvpmodel then asked
for a reboot and failed without rebooting; normal HUD recovery is not yet proved.
Do not bypass that dependency or claim the power-mode label verifies every clock
and gating setting. The owner card still needs the persistent image fix.

The CPU diagnostic worker also encountered Python3.10 TypeError from a thermal
sysfs read. It now skips that unavailable sensor, as host_health already does,
and still renders other sensors and boot errors. It reports NVIDIA initialization
failures too. The portrait JPEG correction and thermal fix were installed in RAM
on the running device; they have not been persisted to its pinned owner payload.
The old owner payload, public image and stable runtime channel remain unchanged.

### Second image candidate: first-mount protection

The owner subsequently confirmed that the USB diagnostic screen is visible.
The new offline installer patches only the SHA256-reviewed L4T36.4.7 initrd init
program, preserving other archive members. Its SD path disables the alternative
EFI overlay selection, requires mmcblk0p1, mounts APP `ro,noload`, skips the DNS
copy into immutable etc, and sets the root block device read-only before PID1.
Unknown init programs/layouts are rejected. The kernel and firmware are unchanged.
This is a candidate implementation; physical boot and power-cycle validation
are still required before claiming the original early-write gap is closed.

Normal systemd boot retains RAM machine identity instead of committing it to APP.
The existing storage helper gives NVIDIA's temporary /mnt directory a small
tmpfs and binds only the regenerated PVA authentication allowlist output to RAM.
It does not disable PVA authentication or make the firmware tree writable.
NVIDIA initialization services explicitly wait for these storage preparations.
Real-loop tests write the PVA output and /mnt while checking the entire base
filesystem remains byte-identical across two simulated helper boots. This is
not a substitute for a physical PID1/USB/model startup test.
