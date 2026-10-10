# Recurrent eGPU USB failures, 2026-10-10

The C4 on 7999cf05dd contains the previous PSP timeout fix. Device-resident
boot logs, four original rlog segments, recent qlogs, persisted application
logs, saved tmux and the kernel ring were inspected before intervention.
Automatic NAS captures corroborate the device evidence. Raw evidence remains
private; do not publish settings, identifiers or complete logs.

## Observed sequence

- The first morning session, on the previous boot, starts modeld at monotonic
  51065.151. No modelV2 is recorded in the two inspected full rlog segments.
  The kernel records USB submit error -19 and eGPU disconnect at 51092.080,
  followed by hub reset/re-enumeration, then another disconnect at 51125.255.
- After reboot, modeld loads in 27.8 seconds and first publishes at 90.485.
  At 98.859 the worker exceeds the existing one-second response deadline;
  saved progress identifies output_read, frame 144, age 977 ms. The internal
  model takes over. This stage alone does not distinguish a GPU completion
  wait from a USB read stall.
- The HUD starts after that fallback. Its USB2 reset is at 101.658; the first
  subsequent eGPU disconnect is at 105.312. These later events cannot be
  claimed as the cause of the earlier output stall.
- A later ignition session begins loading at 488.198. The kernel records
  eGPU disconnect at 508.058; the loader reports libusb_control_transfer:
  No such device at 508.492. That transport error is incorrectly persisted as
  a rejected artifact. Further model starts report compiled=false even though
  the saved download status still says compiled.
- The SuperSpeed hub/eGPU continues to disconnect while the eGPU worker is
  absent. Kernel records include hub U1/U2 failures, -71 protocol errors,
  failed enumeration and xHCI transfer-length errors. The user reports manually
  reconnecting cables after the initial fault; not all reconnects can therefore
  be attributed to an uncommanded hardware fault.

A later read at boot monotonic 1327.788 still shows the eGPU behind the same
SuperSpeed hub, bridge power 14490 mV / 1354 mA with fault=false, and the
existing controller link-error diagnostic at 8338. This is a point-in-time
power observation, not evidence excluding earlier voltage drops. The preserved
kernel ring contains 67 eGPU disconnect entries and 69 hub reset entries,
including normal controller re-enumeration and possible user cable operations.

## Controlled comparison

Fresh valid CAN state confirmed Park, standstill, zero speed and disabled/
inactive control. With eGPU inactive, runtime power management was temporarily
disabled on the observed hub, root hub and external USB controller. Three eGPU
disconnects still occurred in 50 seconds (trial start 1036.250). All three
power/control values were restored to auto. U1/U2 sysfs entries were read-only;
their attempted changes failed and they remained enabled on the external hub.
This excludes neither link-power-state behavior nor electrical/firmware faults.
No GPU reset, artifact deletion or timeout increase was performed for the trial.

## Code correction

The prior PSP-only repair did not cover libusb hot-unplug/transport exceptions.
Recognize terminal known libusb transfer/open errors at the worker/boot-validator
text boundary and preserve the model hash for a future normal start. Do not add
same-session retries or change inference deadlines/fallback. Historical load/
boot-validation rejection recovery still requires matching artifact and different
known boot identities, rechecks artifact hashes and invalidates old smoke receipts.
Checkpoint, output-contract, serialization and model-file errors remain rejected.

The diagnostic collector previously truncated raw dmesg before filtering, hiding
USB events behind camera traffic. It now filters the complete file-backed output
first while retaining bounded memory/output. This improves evidence only.

157 focused model/runtime/startup/build/diagnostic tests pass. Ruff passes after
explicit string-concatenation cleanup; the 95 artifact/runner tests pass again.
The actual captured
hot-unplug traceback matches the transport classifier. This repairs the persistent
software lockout; it does not establish a cure for the physical/driver link loss
or the initial output-read stall.

## Device verification

With Park/standstill/zero speed and disabled/inactive control checked again,
the device cleanly fast-forwarded to e394b17f07 and used the normal manager
reboot once. Normal startup migrated the old transport rejection, reverified
artifacts and passed the C4 smoke test (load 23.71 seconds). No manual rejection
deletion or integrity bypass was used. The original hub topology was retained.

Three separate 55-second observation windows (165.04 seconds total, covering
boot monotonic 165.75 through 383.19 with gaps between windows) received 3,302
modelV2 and 3,302 cameraOdometry messages, all valid, about 20 Hz. eGPU and HUD
were connected in every one-second flag sample; startup_failed remained absent.
Mean model execution in each window was 40.58-40.65 ms, maximum 43.51 ms.
The maximum subscriber receive interval was 89.61 ms, not a sensor-gap measure.
At monotonic 399.40, eGPU remained active with a valid model, no rejection,
a validation receipt and no new same-boot failure. The new boot's preserved
kernel evidence contains no eGPU/hub disconnect or reset. Park, zero speed and
disabled/inactive control were retained.

This confirms recovery from the software lockout and several minutes of parked
operation, not an intermittent-link cure or next-cold-start/driving reliability.
The user was asked to compare a direct eGPU connection without the hub; that
physical comparison remains pending. Do not attribute every historical disconnect
to a defective cable or claim that the earlier output stall is fully explained.
Evidence and reproduction scripts are retained locally under
`.analysis/archive/2026-10-10-egpu-recurrence/`.
