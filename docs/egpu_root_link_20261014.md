# eGPU root-link investigation — 2026-10-14

## Observed failure

The parked device was running carrot-nav `3808b5733a`, with eGPU present but
inactive, loading false and startup-failed true. The background-recovery
commits had not been installed. Its selected precompiled model failed during
inference with `AMD failed with -1`. The exact pinned runtime's error word
latches USB control/bulk failures or a short bulk transfer; the traceback at
warp dispatch does not establish a numerical-model error.

The failure marker maps to approximately boot +5204.55 s. The first subsequent
kernel eGPU disconnect in the retained ring is +5386.56 s, about 182 s later.
Do not misreport that later disconnect as the event preceding the inference
failure. The initial lower-level transfer cause was not separately recorded.

USB topology at inspection was:

- comma SuperSpeed root port -> Genesys Logic `05e3:0625`, 5 Gbit/s hub.
- Hub port 2 -> eGPU bridge `3801:0001`, firmware `ed4e39b7-CLEAN`, 5 Gbit/s.
- The companion USB2 hub also carried the USB display.

Repeated eGPU disconnect/re-enumeration and SuperSpeed hub resets continued
long after the worker exited. No process held the bus-4 USB device nodes at the
ownership snapshot. This is not repeated inference startup/recovery activity.

## Parked read-only measurement

A continuous 60-second observation required fresh, valid, alive Park/standstill,
zero speed, disengaged selfdrive and inactive lateral/longitudinal control.
The eGPU remained inactive and not loading. No model worker was started and no
USB power query ran during this interval.

| Observation | Result |
| --- | --- |
| eGPU USB addresses | 15 -> 16 -> 17, two disconnect/re-enumeration cycles |
| Host root-port link errors | Mean 679.49/s, range 614.35–741.68/s |
| Internal model output | 1,201 messages, zero invalid, 20.007 Hz |
| Largest internal model publication gap | 65.22 ms |
| Driver model output | 1,200 messages, zero invalid, 20.010 Hz |
| Largest driver model publication gap | 70.50 ms |

Error rates use 56 adjacent one-second samples with the same nonmissing USB
address and a nondecreasing counter; intervals spanning disconnect/reset were
excluded, not interpreted as negative errors or counter rollover. The official
[AGNOS kernel implementation](https://github.com/commaai/agnos-kernel-sdm845/blob/master/drivers/usb/dwc3/dwc3-msm.c)
exposes the controller's USB3 PORTLI register. The low 16-bit link-error field
and topology implicate the comma-to-hub SuperSpeed link, not a model tensor.
This does not identify which endpoint, cable/contact, power source or firmware
caused those link errors.

A separate earlier bridge power snapshot read 13,388 mV, 3,850 mA, fault=false.
It is neither a transient-voltage capture nor evidence that the power path is
healthy under every condition. No inference worker running also does not mean
the powered GPU/bridge is electrically idle.

## Power-management hypothesis and limits

The eGPU device already had power/control=on. The parent hub had control=auto,
was active, and reported hardware U1/U2 link power management enabled. The
[Linux USB power-management documentation](https://cdn.kernel.org/doc/html/latest/driver-api/usb/power-management.html)
distinguishes link power management from runtime autosuspend.

A planned temporary U1/U2-disabled comparison could not run: the installed
kernel denied writes to those attributes, including through sudo. Both final
readbacks remained enabled. Only the original-state observation completed;
there is no successful A/B comparison and no evidence that disabling LPM fixes
this device. No unbind, USB reset, firmware flash, kernel change or permanent
power-management change was performed.

## Next discriminating test

While parked and inactive, compare the same root-port error rate and USB
address stability after replacing the comma-to-hub SuperSpeed cable, then
bypassing/replacing a separate hub if the physical topology permits. Change
one component at a time. If the hub is integral to the eGPU adapter, compare a
known-good adapter/cable path instead of assuming it can be bypassed. If errors
remain, inspect connector/host PHY, hub/bridge firmware and power transients.

Only after the link is stable should full eGPU inference and the background
recovery path be tested. Current available RAM was approximately 635–666 MiB,
below the recovery candidate's conservative memory guard; that guard was not
weakened to force a test. Neither physical-root-cause repair nor recovered-eGPU
road validation is claimed. Vehicle code/settings were not updated. Raw logs,
USB ownership snapshots and probe samples remain in the private analysis tree.

## Follow-up after the user power-cycled the comma

The eGPU resumed operation with greatly reduced link errors and no observed
re-enumeration, but a separate checkpoint overhead regression limited inference
to 16 Hz. See [the correction and parked retest](egpu_checkpoint_latency_20261014.md):
20 Hz was restored without another power cycle. The earlier investigation
above describes the pre-cycle state; it does not establish a faulty cable.
