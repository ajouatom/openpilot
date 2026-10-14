# eGPU USB failure and persistent rejection (2026-10-14)

## Findings

The reported vehicle ran `carrot-wip` at `504a902155`. Its uploaded minute
contains 1,200 valid model outputs with consecutive frame IDs, approximately
20 Hz, and model execution mean/p99/max of 35.06/36.50/37.47 ms. The actual
failure occurs about seven seconds after that recording ends; automatic error
reports provide the failure evidence, not the uploaded driving segment.

At 10:36:58 KST, the eGPU USB port stopped responding to address setup. The
kernel logged error -71, failed link-state/disable operations, and a SuperSpeed
USB reset. The worker failed during input upload with `AMD failed with -1`.
In the installed, SHA256-verified runtime, AMD.error_state points to the USB
link's first-error word. USB I/O failure and short transfers can produce -1.
This establishes a USB communication failure, not an invalid model prediction.
It does not distinguish cable, connector, power, bridge/firmware, or host causes.
The device report already has USB power/control=on; it does not support claiming
that merely disabling USB autosuspend will fix the incident.

The failure was incorrectly persisted as an artifact rejection. Later reports
after reboot/update to `8165f50b32` contain the identical earlier failure and
show USB present but model rejected. Those reports do not demonstrate repeated
new inference failures. The selected model was retrained Mountain Dew 8d736683,
compiled AMD 256f2178, not the navigation model.

## Correction

Recognize the exact terminal `AMD failed with -1` error only for runtime archive
SHA256 `90970e01f2ba70d553294c0f9d5c6fe1678827f750705cab679a7b19b7bcb0b2`.
Preserve the failure report and invalidate the boot smoke-test receipt, without
blacklisting the model artifact. Other AMD errors and unknown runtime versions
retain their existing rejection behavior.

Previously misclassified USB failures may recover after a different boot, with
matching artifact and runtime evidence. Normal download/hash verification and
boot smoke validation remain required. Explicit USB transport failures during
inference also qualify for this migration; PSP bootloader timeout migration
remains startup-only. Missing or mismatched evidence remains blocked.

The worker still fails immediately and closes on a runtime/USB failure; existing
internal-model fallback remains unchanged. There is no in-drive USB retry, stale
prediction substitution, timeout relaxation, runtime upgrade, or model change.
The separate one-frame numerical-output tolerance does not apply to USB faults.

## Validation and limits

On carrot-nav, 206 focused model delivery, runner, validation, and numerical
recovery tests pass using the Windows native-dependency test harness. Ruff
passes for both changed Python files. Tests cover exact runtime/error matching,
failure persistence, receipt invalidation, previous-boot migration, rejection of
ambiguous errors, and re-verification of corrupted cached artifacts.

Raw logs and diagnostic reports remain private. This change repairs persistent
blocking after a USB fault; it does not repair the underlying USB fault. No
installation on the affected vehicle, hardware reproduction, measured road
fallback gap, or successful post-fix drive is claimed.

## English reply for the reporter

The diagnostics show a USB communication failure while uploading inputs to the
eGPU. The software then incorrectly marked the model as rejected, preventing it
from starting again even after a reboot. This patch corrects that handling and
allows the affected model to go through normal verification after a reboot.
Immediate fallback to the internal model remains unchanged. The patch does not
fix the underlying USB failure; its hardware/firmware cause is still unconfirmed.
