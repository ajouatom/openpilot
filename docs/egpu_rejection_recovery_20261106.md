# eGPU quarantine after a device timeout or hang

The reported comma 4 / Tiny Chestnut RX 9060 XT failures are consistent with
the existing classification path: a terminal `AMD failed with -7` or
`Device hang detected` at boot validation creates `rejected`, and the old
previous-boot allowlist does not recognize it. A desktop reproduction using
the previously committed implementation keeps the model blocked on the next
boot; the corrected implementation permits verified preparation.

This reproduces the persistence bug, not the original GPU fault. The five
reported diagnostic JSON files and the affected hardware were not available
for this change. No vehicle, CAN, firmware, model selection or Jetson changes
were made.

## Source evidence

The four deployed runtime archives were read and their complete SHA256 hashes
checked against `egpu_usb_wait.VERIFIED_USB_RUNTIMES`: N16 `90970e01`, Flip/MD
`c7c58622`, Light speed `cb9bf2cf`, Light speed 2 `96c6ecc9`.
Each binds AMD `error_state` to the USB link error word. Its USB wait graph
writes `LIBUSB_ERROR_TIMEOUT` (-7) when completion misses the wait deadline.
The AMD hang handler raises `RuntimeError: Device hang detected`.
This does not distinguish power, cable, bridge, driver or GPU as the initiating
cause. In particular, a timeout is not evidence of corrupted model weights.

## Policy

- Recognize only these terminal messages in hash-pinned runtimes. A later
  checksum/contract exception, quoted text, unknown runtime or another numeric
  error is not automatically allowed by this change.
- Keep the rejection marker for the current boot and fall back to the internal
  model. Do not add these faults to the live background-recovery allowlist.
- On a different, known OS boot identity, allow normal preparation for matching
  saved failure/model identity. This also migrates old reports that were saved
  as `artifact_blocked`. Previously misclassified terminal PCIe link-not-up
  reports are eligible as well; current PCIe failure handling is unchanged.
- Invalidate the startup receipt, verify PKL/runtime hashes, retain contract and
  output checks, and require startup validation. A new failure blocks that new
  boot again. There is no repeated reset or attempt loop within a failed boot.
- Persist `device_restart_required` for new known hang/timeout reports. Retain
  the diagnostic file when retrying; a subsequent failure records its new cause.

## Carrot Web

The eGPU card shows whether the active artifact is blocked, its terminal cause
and phase, reboot guidance, and the existing diagnostic download link. It does
not attach an old model's rejection to a different model being downloaded.

“Recheck blocked model after reboot” is available only with matching saved
evidence, a known boot ID, offroad true, onroad/engaged/GPU-active/loading false,
and no installation or active recovery. The API rechecks these conditions and
requires the displayed active model SHA. The button does not reboot, start a
worker, delete diagnostics or remove the rejection marker.

It writes a request bound to the PKL hash, runtime hash, exact failure-file hash
and requesting boot ID. Normal preparation consumes it once, only on a later
boot, before attempting verification. A changed report/model/runtime invalidates
the request. Corrupt downloads still fail verification and remain blocked.
Missing/inconsistent evidence cannot be released through this button.

Apply with ordinary carrot-wip Git Pull and a full comma reboot while parked.
No manual renaming of `rejected` or `last_failure.json` is needed for matching
previous-boot device failures. Power-only ignition cycles that leave the comma
OS boot unchanged do not qualify as a new boot. Internet may be needed by the
existing installer to retrieve the catalog or repair cached files.

## Validation

- 416 focused Python tests pass: old-report migration, four pinned runtimes and
  three failure phases, repeated-boot quarantine, output/integrity checks,
  one-shot requests, corrupt downloads, API state/model guards and existing
  delivery/recovery/startup/receipt behavior.
- 19 web tests pass, including Korean/English reason and action states.
- Isolated production web build passes; unrelated ongoing web edits are excluded
  from the published bundle.
- Ruff passes for changed Python modules and tests.

No physical eGPU recovery or road behavior is claimed. Repeated GPU hangs and
the reporter's remote-start PCIe failure still need actual diagnostics.
