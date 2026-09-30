# Jetson microSD / NVMe 공용 패치

**v2 수정 안내:** v1은 초기 부팅 이미지에 없는 `wc`·`readlink` 명령을 사용하여
부팅을 막는 결함이 있었습니다. v2는 해당 명령 없이 동작합니다. 이미 v1을 적용했다면
새 패치를 기존 CarrotJetson 폴더에 덮어 풀고 **03만 다시 실행**하세요.
이미지 재다운로드·재기록은 필요 없습니다. v1 배포는 중단합니다.

기존 **v0.4.0-boot-preview의 R2 이미지**를 그대로 사용합니다. 새 이미지를
제작하거나 다시 다운로드하지 않고, 약 32 MB 패치만 추가합니다.
대상은 Orin Nano 개발자 키트의 microSD와 **M.2 NVMe**입니다.
USB 외장 케이스는 PC에서 기록할 때 사용하며, Jetson에서는 NVMe 슬롯에 장착합니다.

**시험 패치:** PC 검사는 통과했지만 패치된 매체의 실제 SD/NVMe 부팅은 아직
검증 전입니다. 기존 정상 작동 SD를 복구용으로 보관하세요.

1. [패치파일 받기](https://upload.shind0.synology.me/downloads/jetson/v0.4.0-sd-nvme-patch-v2-preview/carrot-jetson-windows.zip)
   후 내용을 기존 **CarrotJetson 폴더 안에** 풀어 `support` 폴더를 합칩니다.
2. 처음 설치하는 매체라면 기존 **01 → 02**를 완료합니다. 이미 R2를 기록했다면
   다시 기록하지 않습니다. NVMe는 **NVMe용 USB 외장 케이스**로 PC에 연결합니다.
3. **03_SD_SSD공용패치.cmd** 실행 → 관리자 권한 **예** → 대상 **디스크 번호**
   → **PATCH** 입력. 검사·패치·기록 확인은 자동이며 약 5~20분 걸립니다.
4. 안전하게 제거하고, Jetson **정상 종료·전원 분리 후** SD 또는 NVMe를 장착합니다.
   복제된 SD와 NVMe를 동시에 연결하지 마세요.

02는 선택한 매체를 지웁니다. 먼저 백업하고, 기록·패치 중에는 분리하거나 PC 전원을
끄지 마세요. 중단되면 같은 패치를 다시 실행하여 검증이 끝나기 전에는 부팅하지 마세요.
나중에 02로 원본 이미지를 다시 기록하면 03도 다시 실행해야 합니다.
호환 QSPI 펌웨어는 여전히 필요하며 이 패치는 펌웨어를 변경하지 않습니다.
남는 SSD 용량의 자동 확장은 포함하지 않습니다.

## English

**v2 correction:** v1 called `wc` and `readlink`, which are absent from the R2
initrd, preventing early boot. v2 uses Bash builtins instead. If v1 is already
applied, extract the new patch into the same CarrotJetson folder and **rerun only
03**. No image download or rewrite is required. v1 is withdrawn.

Keep the existing **R2 image from v0.4.0-boot-preview**. No new image build or
download is needed. This approximately 32 MB add-on patch supports microSD and
M.2 NVMe on the Orin Nano developer kit. Use a USB NVMe enclosure for PC writing,
then install the SSD in Jetson's NVMe slot.

**Experimental:** desktop checks passed; physical boot of the patched SD/NVMe
is not yet verified. Keep your working SD as a recovery option.

1. [Download the patch](https://upload.shind0.synology.me/downloads/jetson/v0.4.0-sd-nvme-patch-v2-preview/carrot-jetson-windows.zip)
   and extract its contents **inside the existing CarrotJetson folder**. Merge
   `support` folders and replace matching files. Existing portable Python is reused.
2. For new media, run the original **01 → 02**. Already recorded R2 media does
   not need rewriting. Connect NVMe through a USB NVMe enclosure.
3. Run **03_SD_SSD공용패치.cmd** → administrator **Yes** → target **disk number**
   → type **PATCH**. Verification, patching and readback take about 5–20 minutes.
4. Safely eject. Shut down Jetson and **disconnect power before installation**.
   Install only the chosen medium, never both cloned SD and NVMe together.

Step 02 erases the selected medium: back up first. Do not disconnect or power
off during writing/patching. After interruption, rerun the same patch and wait
for successful verification before booting. Reapply after rewriting the original
image. Compatible QSPI firmware is still required and is not modified. Additional
SSD capacity is not automatically expanded. First test boot, Wi-Fi and model
connection while safely parked with driver assistance disengaged.

## Implementation and validation scope

The publisher opens the existing 40 GiB R2 image read-only and emits same-size
sector replacements for six APP files: extlinux.conf, initrd, fstab, the storage
marker, protected_storage.py and protected_first_boot.py. Inodes, GPT, filesystem
allocation, kernel, model, runtime, DATA and SETUP contents are unchanged.
Python replacements are compressed source wrappers; gzip initrd is recompressed
within its original allocation with zero padding. Every archive member except
`init` is preserved. No replacement image file is generated.

Boot uses APP PARTUUID `d3fb8cf2-60ea-44b1-bec1-63a7417719cf`. After loading NVIDIA's
existing PCIe/NVMe drivers, initrd rejects multiple currently visible matches and
accepts only SD/NVMe partition 1. APP's first mount remains `ro,noload`, and its
block device is made read-only before PID1. Keep only one image copy connected;
this check does not guarantee detection of hardware that enumerates later.

Storage format 2 resolves the mounted root's kernel major/minor identity and
derives DATA17, SETUP16 and EFI10 from that same disk. No global DATA/SETUP label
lookup can select a different disk. EFI is an optional read-only mount through
a volatile symlink created before local-fs-pre. DATA failure retains the immutable
baseline runtime; SETUP failure retains independent DATA identity recovery.
Legacy format 1 and its SD-only behavior are preserved.

The installer retains system/boot/source-disk exclusion, USB-only PC selection,
stable-ID rechecks and volume locks. The full normalized APP SHA256 and GPT guard
must match before any write. Readback and reruns are supported, including interrupted
patch extents; unrelated APP changes are rejected. Format 1 retains its 64 KiB
patch bound; format 2 allows at most 32 MiB for initrd. Compressed manifests are
checksum checked and have a bounded decompression size. This is release-specific
integrity checking, not an unattended signed OS updater.

Base image SHA256:
`b11f5601d3a713ad0de23315ee90daddf5452f8e548f2c87c8eeec28d321e55f`.
Virtual patched image SHA256 (no image file was created):
`2f97e66ea533c34750ba676a81df51e4485eb6ed742dcbe48a324ad605ade62b` (v2).

Checks cover the exact R2 base, full virtual-result hash, independent ext4 file
reads, full normalized APP validation, in-memory patch writes/readback and retry,
SD/NVMe device selection, wrong-root rejection, recovery branches, Linux shell
selection and Windows disk guards. Physical media patching, UEFI selection,
SD/NVMe boot, inference, Wi-Fi persistence and power-cut endurance remain untested.
The normal image download and signed automatic runtime channel are not changed.

### September 30: v1 boot failure investigation

An owner reported that SD displayed Jetson diagnostics while the patched NVMe
left the USB display on its own default screen. This establishes absence of
Jetson-rendered diagnostics, not the exact firmware/kernel stopping point.

Inspection of the actual shipped R2 initrd confirmed NVMe, NVMe-core, PCIe and
PHY modules are present, as is util-linux blkid with the requested options.
However, neither wc nor readlink is installed. The v1 selector called both before
mounting APP. Its failed readlink leaves an empty root path, triggering the rescue
shell. This is a reproducible patch defect consistent with the report; the
owner's exact stopping point still needs physical confirmation.

The previous desktop test incorrectly supplied a readlink mock and inherited
the desktop wc. It therefore could not detect this dependency failure. Tests
now clear PATH inside Bash (including on Git Bash, which otherwise adds its own
utilities), supply only the existing blkid/sleep operations and exercise SD,
NVMe, duplicate, missing and unsupported roots. The old two-command sequence
fails under these conditions; v2's newline detection and assignment use builtins.

The v2 package changes only initrd relative to v1's installed system. The patch
retains all original sector extents, so the full normalized APP check also accepts
v1 and interrupted patch writes. Independent virtual-disk validation applies v2
over v1, reads every changed file through ext4 and verifies a repeated run without
creating or writing a disk image. This does not substitute for physical SSD boot.
