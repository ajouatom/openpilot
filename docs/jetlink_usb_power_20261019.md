# Jetlink USB input-current protection

The PV5 reboot report has a PMIC UVLO power-off record and a subsequent
Jetson Orin Jetlink session. Upstream
[491cf1ef8d6786ed51cf995e18fc8721389066c6](https://github.com/zoompilot/jetlink/commit/491cf1ef8d6786ed51cf995e18fc8721389066c6)
reports a comma 3X browning out when drawing current from a USB host. This is
a plausible matching mechanism, not a confirmed diagnosis of the PV5 event.
The saved USB available-current messages are not measured current consumption,
and occur after the reboot. Private captures remain outside Git.

## Port into Carrot

Carrot uses its own always-running Jetlink daemon, rather than upstream's
`jetlink/comma/owner.py`. Its gadget's self-powered descriptor does not itself
force the charger input current to zero. Port only the USB input-current
protection; preserve branch-specific models, transport arbitration, deadlines,
USB role policy, VM tuning and inference code.

The daemon observes `/sys/class/usbpd/usbpd0/current_pr` between sessions.
After it reports `none` for at least 1.5 seconds across observations, a bounded,
noninteractive root helper sets the existing `USB_ICL` voter's `force_val` to
zero before enabling `force_active`. The helper rechecks the port before
enabling the vote and verifies both values afterwards. This readback confirms
the requested kernel state, not electrical behavior at the connector.

Protection applies to USB, auto, Android and iOS configurations. It stays in
place across connection failures, protocol fallback, mode changes and daemon
exit. Reboot clears the kernel vote. Carrot has no equivalent live link-off
setting, so this port does not add upstream's explicit draw-restoration path.
There is no per-frame helper call or new background thread.

Already-attached or unknown ports are deferred. A current forced nonzero vote
is left alone, and missing nodes, rejected writes and helper failures are
reported without terminating inference. Failure attempts are bounded to one
per daemon lifetime; a helper observing a concurrent attachment defers for a
new empty-port interval instead.

Unlike upstream's explicit iOS mode, Carrot **does not cut an attached port
based on the configured transport alone**. Its transport selection is not proof
that the device has another power supply. This preserves devices powered only
through USB, at the cost of deferring protection until a later empty-port window.
Polling and sysfs writes are not an atomic hardware interlock.

## Activation and limits

A fresh daemon is needed to load the change. If the host is already connected
at startup, protection is deferred. With the vehicle parked and the comma
independently powered, leave the host cable disconnected until tmux reports
`Jetlink USB power: zero input-current vote verified; retained until reboot`,
then reconnect it. Merely pulling this change with the cable continuously
attached does not establish that protection has been applied. Do not disconnect
a device's only power feed to perform this check.

No vehicle installation, restart, live voter change or physical USB test was
performed for this change. The PV5's exact comma variant and pre-reboot current
trace remain unverified. This is not a demonstrated cure for that reboot or
for the separate next-day GPS/modem disconnect.

## Validation

New tests execute the helper against temporary sysfs-like files: initial cut,
inherited vote, attached/unknown/missing state, pre-existing vote conflict,
attachment during helper startup, write failure and failed readback. Policy
tests cover empty-port debounce, bounded failures, cold attached startup,
helper deferral and preservation across later plugs. Existing daemon tests
verify invocation before gadget setup across all transport modes and retries.

Desktop regression runs use Linux OS doubles on Windows. The real Linux
`sendmsg` socket case cannot execute there and is excluded; the existing Linux
Jetlink CI suite includes it and discovers the new tests automatically.
Fake sysfs and desktop tests do not validate PMIC electrical behavior, road
stability or native inference frequency.
