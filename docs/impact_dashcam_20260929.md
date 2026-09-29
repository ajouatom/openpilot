# Suspected-impact Dashcam transition

The user selected accelerometer detection, a 1.5g threshold, an on-screen alarm
with a ten-second touch cancellation window, then saving OpenpilotEnabledToggle
OFF and rebooting. This is an experimental trigger, not a collision classifier.

## Detection

Selfdrived processes the existing accelerometer subscription at its 100Hz loop.
The current LSM6DS3/TRC stream is 104Hz with a per-axis +/-2g range. Samples can
saturate; short pulses and subscription conflation can be missed. Sensor range,
sensord configuration, locationd checks and control validity thresholds are not
changed. The old deviceFalling event definition is not wired to a producer in
this tree and is not used as proof of a functioning drop detector.

The detector uses locationd's sensor-to-device axis mapping, subtracts gravity
using fresh valid livePose orientation, and rotates into calibrated vehicle
axes using liveCalibration.rpyCalib. It does not use filtered pose acceleration.
The norm of longitudinal and lateral acceleration must reach 1.5 * 9.81 m/s²
on two distinct increasing sensor timestamps no more than 30ms apart. A sample
must be at most 100ms old; pose must be at most 200ms old, healthy and valid,
with completed valid calibration. NaN, malformed, future, stale and duplicate
samples cannot count as a second hit. Pure vertical shocks and resting gravity
do not trigger. Mount motion/drops can still look like horizontal impacts.

Detection is enabled only after initialization while onroad and non-passive;
REPLAY, SIMULATION and notCar are excluded. aEgo is diagnostic context only;
its wheel-speed Kalman filter resets acceleration when speed disagreement
exceeds 2m/s and is unsuitable as the required corroborating signal here.

## Countdown and reboot

Typed JSON Params in /dev/shm carry a unique notice token and UI feedback.
Both C3 and C4 render a global three-line notice over the current screen.
Any touch cancels and is consumed before underlying widgets handle it. The
existing prompt sound uses normal soundd volume/mute and alert priority.
Higher-priority driving warnings retain precedence. The notice yields to full
or critical alerts and stale selfdriveState, so it cannot cover takeover text.

The ten seconds start from acknowledged UI presentation, not impact time.
The final UI frame must acknowledge at least ten seconds of presentation;
touch cancellation wins over expiry. An unseen notice expires after two seconds;
a visibility/heartbeat interruption exceeding 0.5 seconds cancels the transition.
A fresh quiet interval below 10m/s² for one second re-arms after cancellation.
Missing/invalid input cannot earn this quiet interval. Once detected, loss of
sensor data alone does not erase an otherwise visible countdown.

Expiry synchronously saves OpenpilotEnabledToggle=false and verifies its typed
readback, sets/verifies ImpactDashcamReboot, then requests manager DoReboot.
The existing manager cleanup and hardware reboot helper handle workers and the
bounded reboot chime; no reboot syscall or new audio child is added here.
Selfdrived immediately disables/blocks entry, controlsd gates AlwaysLateral,
and card neutralizes any queued active control until shutdown. After reboot,
existing card startup configures passive/noOutput from the saved OFF toggle.
Re-enabling requires the normal manual enable/restart path. Recording has a
reboot gap; no incident file protection or automatic upload is added.

## Validation

Focused synthetic tests cover direction, gravity/tilt compensation, vertical
bumps, AEB-sized acceleration, one-sample spikes, invalid/future/stale/duplicate
input, UI cancellation including expiry, hidden/frozen UI, settings write order,
engagement lockout and neutralizing queued control. Windows adapters replace
native IPC/Params/hardware, while cereal schemas, math, events and the state
machine are real. Physical detection/false-positive rates, mount behavior,
C3/C4 display and audio, control release, log completion and reboot remain
unvalidated. No collision or drop was performed for validation.

The focused desktop run passed 123 tests, with one native IPC sound-timeout
test excluded by the Windows runner. C3/C4 Korean and English notices were
rendered using the production prompt/label code and inspected for fit. Python
parsing and new-file Ruff checks pass; modified existing modules retain their
33 pre-existing Ruff findings without additions. The desktop NumPy version
emits existing orientation-helper deprecation warnings.

The bilingual settings summary and Dashcam guide explain persistent OFF and
recording interruption. OpenpilotEnabledToggle is a native device setting,
absent from carrot_settings.json, so there is no corresponding generated
per-setting Wiki page to modify; the existing guide publication path carries
the localized documentation.
