# One-time Windows patch for automatic Jetson updates

The user replaced the manual first-update wait workflow with removing the boot
storage, applying a Windows patch once, and reinstalling it. The package is
self-contained; it requires neither the original 10 GB installer nor SSH, WSL
or a separately installed Python. This is a preview pending physical patch/boot
validation, not evidence that every existing Jetson image is supported.

## Supported storage and PC operation

The publisher opens the exact public R2 40 GiB image read-only, verifies its full
SHA256, and resolves the existing protected image-setup file through ext4. It
also constructs a read-only view of the published SD/NVMe v2 patch. Each variant
receives a same-size compressed Python replacement for
`/usr/lib/carrot-jetlink-storage/protected_first_boot.py`, with independent ext4
readback. Its existing allocation covers 6,144 bytes. There is no inode, allocation,
GPT, bootloader, kernel, model, filesystem creation or full image rewrite.

The Windows launcher excludes system, boot, internal, source and offline/read-only
disks; the user selects and confirms a USB disk. Stable identity is checked again
before each phase. Existing volume locks and normalized full-APP hash verification
identify the exact supported variant and permit interrupted patch retries. Other
root modifications, unknown images and withdrawn NVMe v1 are refused.

After a read-only APP preflight, the launcher validates SETUP partition 16 by
offset, size, FAT32 and CARROTSETUP label. It copies only `carrot-boot-update.zip`
using a temporary file, flushes and verifies its hash, then applies and reads
back the APP sectors under volume locks. DATA and existing identity/network
files are not edited by Windows. A temporary drive letter is removed if one was
assigned. Windows format prompts must be canceled, and interrupted patches must
be rerun successfully before booting. PC completion confirms these writes, not
Jetson runtime activation.

## First Jetson boot

The original image-setup functions are preserved. Before their normal entry,
the patched helper installs RAM-only guards on inference and HUD services,
requires image setup completion, and verifies the small payload against the
SHA256 embedded in the protected APP code. Unknown archive members or altered
payloads are refused before execution. The verified payload is cached under
DATA for subsequent boots, without rewriting unchanged helper files.

Using the existing runtime virtual environment, it installs the same stable boot
updater helpers as signed host source `84087a5b78118acc40234bfd8ef64235421f1aab`.
The helper never overwrites an already installed newer updater. The legacy apply
service can now start the USB-only boot gate; the service guards also cover old
runtime entry points that lack Python guards. They have no start timeout while
waiting for Internet. The new gate receives C4's signed selection and existing
private Wi-Fi provisioning, verifies/downloads/applies via the existing candidate
probe and transaction path, then starts inference/HUD. It does not require the
legacy offroad timer or an extra activation power cycle. The normal signed pin,
model and runtime ABI are unchanged.

Missing/corrupt payload or incomplete migration cannot pass the service guards.
If protected DATA storage is unavailable, the original immutable recovery path
is retained; this patch is not a DATA repair tool. Runtime activation and new
boot-gate behavior still require actual parked-device validation.

## C4 first-update wait retirement

The Web card, elapsed timer, translations, poller and API are removed. The old
manual offroad gate and provisioning-only maintenance loop are removed too.
`JetsonLegacyUpdatePending` remains registered only as CLEAR_ON_MANAGER_START,
so saved holds are cleared when the updated manager starts; the old alert was
already cleared at manager start. The pending flag stays excluded from ordinary
settings and backup restoration. Normal USB boot-update status and host telemetry
continue through their existing paths. Removing the old card does not itself
upgrade an unpatched Jetson.

## Validation

Desktop tests cover exact variant selection, read-only preflight, unaffected
data, readback, interrupted retry, corrupt/unknown APP rejection, one-time helper
installation, later-updater preservation, boot guards, bad payload failure,
cache reuse and baseline recovery. Existing boot gate, updater, Windows disk
guards and Jetlink tests are also run: 100 Python tests passed, with 18 platform
skips; four Web tests, the Web build, new-module Ruff and 15 Windows disk guards
passed. Packaged embedded Python inspected the real patch without relying on
the PC Python installation. The guide rendered without horizontal overflow at
904px and a true 390px viewport. Both real image variants passed full APP
selection, 6,144-byte virtual application/readback and independent ext4 decoding;
their original apply-service ordering and executable test helper were checked.

The self-contained ZIP is 12,448,084 bytes, SHA256
`7ae15ab13011f1f3d84ef4a80a899d070841d52f7a8a30a17d60b43f375dc597`.
The boot payload is 21,941 bytes; bundled Windows Python is the same verified
3.14.7 embedded release used by the earlier installer. The release location is
[the Windows patch ZIP](https://upload.shind0.synology.me/downloads/jetson/v0.4.1-boot-update-patch-preview/carrot-jetson-auto-update-patch.zip).
Private publication/readback evidence is retained in the local investigation
archive, rather than committing disk captures or private device information.

Physical USB-media writes, actual systemd startup ordering with this APP hook,
first patched boot and vehicle operation are not established by desktop checks.
