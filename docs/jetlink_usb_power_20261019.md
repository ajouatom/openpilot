# Jetlink auxiliary USB input-current protection

## Current policy (2026-10-23)

The owner explicitly requires the vehicle/OBD-C connector to supply the comma.
The auxiliary Jetson USB connection must not supply operating current. This
supersedes the previous empty-port-only protection: **there is no gear, speed,
voltage, transport-mode, cable-detach or separate enrollment condition**.

On every normal AGNOS launch, before repository locking, OS initialization/update work, dependency
installation, build and manager startup, the launcher runs a bounded root
helper. It sets the existing PMIC `USB_ICL/force_val` to `0`, enables
`USB_ICL/force_active=1`, and verifies both values. An already active nonzero
forced input limit is replaced with zero without temporarily disabling the
force. Missing or unrecognized interfaces and failed writes/readback are errors,
never a successful apply. Each apply reasserts `force_active=1`, including when
the vote already reads `1/0`: writing `force_val` alone does not run the PMIC
callback, and a previously failed callback can leave those same readback values.
The force is never disabled during reapplication.

Jetlink independently runs the same helper once before opening a USB session,
including manual daemon startup. If verification fails, only Jetlink is marked
blocked with an explicit power-protection error. The launcher/manager and native
local inference are not stopped. This avoids silently using an unprotected
host connection. The helper is not called per frame or repeatedly after failure.

The forced vote remains across cable reconnects, ignition/mode transitions and
daemon exit. The kernel resets it at reboot; the code reapplies it on every
boot. Pulling the change alone does not change a running process. Update and
restart normally while parked, retaining the independent main supply.

This policy intentionally does not fall back to Jetson power when main power
is lost. An installation powered only by auxiliary USB can shut down as soon
as the helper runs. Main power is an installation requirement, not inferred
from the USB host, CAN messages or a single voltage observation.

## Scope and physical limits

Only the two auxiliary USB input-current voter files are written. No USB data
role, USB-PD power role, UDC bind, VBUS regulator, CAN or main-input control is
changed. Existing USB transfer timing, HUD snapshots, inference and vehicle
control code are unchanged. This is a comma-only update; the selected Jetson
package and its validation/update path are unchanged.

The software policy starts when the launcher executes. It cannot guarantee no
current during boot ROM/kernel startup before this code runs, zero electrical
leakage, galvanic isolation, or sufficient main-supply capacity. A requirement
for physical exclusion from the instant power is connected needs hardware
support. Do not equate a verified voter with a connector-current measurement.

Upstream [Jetlink 491cf1ef](https://github.com/zoompilot/jetlink/commit/491cf1ef8d6786ed51cf995e18fc8721389066c6)
reports a 3X brownout when drawing from a USB host. The
[comma kernel implementation](https://github.com/commaai/agnos-kernel-sdm845/blob/4a6a2ab8e50fa4c6875d20b5f4392fef2afbe3a1/drivers/power/supply/qcom/smb-lib.c#L910)
suspends USB input for requested current at or below 25mA. This supports the
chosen input-current control, but does not establish the cause of this owner's
power-offs or prove the new branch safe under actual vehicle load.
The [voter callback implementation](https://github.com/commaai/agnos-kernel-sdm845/blob/4a6a2ab8e50fa4c6875d20b5f4392fef2afbe3a1/drivers/power/supply/qcom/pmic-voter.c#L488)
is why each application writes the active flag after the zero value.

## Read-only verification

From the comma web terminal after updating and restarting:

```sh
sudo -n python3 /data/openpilot/tools/jetlink/usb_power.py --status
```

Expected: `force_active` is `"1"`, `force_val` is `"0"`, and `input_blocked`
is `true`. This reports the kernel vote, not measured connector current or
independent confirmation of a previous PMIC callback. `--status` never changes
the vote. A permission or missing-file error is not evidence that protection
is applied.

## Validation

Fake-sysfs tests execute the actual helper for attached/empty/unknown role
states, existing inactive and nonzero active votes, repeat boot/daemon starts,
write failures, missing nodes, failed readback, PMIC callback reapplication
after failure and read-only status. They check
that no sibling role, VBUS, UDC or main-power files are changed. Daemon tests
exercise normal USB/mobile connections and verify that a failed current cut
cannot reach gadget setup, handshake, HUD or inference. Launcher ordering is
checked separately. No physical USB, power-off, vehicle or reboot test is
claimed for this change; those require separate measurements.

On 2026-10-23, 328 PC tests passed in a Linux temporary checkout with these
changes over `c1bec34b`. Coverage includes the power helper and launcher,
daemon ownership, USB priority, mobile/auto transport, adapter preparation,
boot updates, HUD, update permission, connection startup and timing. Bash
syntax validation of the launcher also passed. Windows checkout CRLF and
filesystem permission artifacts were excluded by using Linux files and modes.

## Standalone carrot-nav review (2026-10-23)

Ported only the auxiliary-input policy, launcher hook, daemon guard and their
tests from contributor commit `b18ddab2` onto upstream carrot-nav `819ce67c`.
No PV5 HUD, Wi-Fi, inference optimization or host-package change is included.
The existing upstream connection-startup and shared-input behavior is retained.

On this port, 258 isolated Linux PC checks passed across USB power, daemon
compatibility, automatic/mobile transport, timing and USB priority. The launcher
passed Bash syntax validation after normalizing the temporary export's line
endings; the Git index stores LF. These tests use fake sysfs and transport
objects, not attached hardware. Physical current, reboot and vehicle behavior
were not re-tested for this port. The independent main-supply requirement and
pre-launch electrical limitations above still apply.
