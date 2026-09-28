# DM2 implementation and validation — 2026-09-28

The user requested a new default-off experimental monitoring mode, then explicitly
requested automatic interaction monitoring for both absent and failed cameras,
strong-forward-attention recovery, and pedal/steering/vehicle/BT-button credit even
with a working camera. This supersedes the initial proposal to require a manual
camera-installation setting. There is no such setting in the final implementation.

## Configuration and stock boundary

- `DriverMonitoringMode=0` is the boot-latched default; only an explicit value of
  `1` selects experimental behavior. Existing `DisableDM` values never select 1.
- Stock `monitoring/policy.py` and `monitoring/dmonitoringd.py` are unchanged.
  The managed process retains its name and scheduling but runs `dm2d`, a bounded
  20 Hz dispatcher. `DriverMonitoring2` inherits the original policy; when a
  healthy camera is used in mode 0, the original criteria and state transitions
  are used. Models, model selection and inference artifacts are unchanged.
- `DisableDM` remains registered only for migration. Old value 2 preserves road
  streaming through `CarrotVisionEnabled`; monitoring is enabled independently.
  The web's individual setting writer asks for experimental-use acknowledgement.
  This acknowledgement is not a certification or a legal exemption.

## Policy

Camera mode 1 multiplies only head-pose tolerances by 1.2. Sleep, eye closure,
phone, confidence gates, terminal counters and lockout use the stock criteria.
A fresh control edge restores at most two seconds of ordinary attention allowance,
at most once per second and never above full awareness. No extra credit applies
to orange/red alerts or current or outstanding eye/sleep/phone distraction.

Forward-attention evidence combines face/eye confidence, open-eye, sleep, phone,
sunglasses and centered head-pose signals. A minimum component score of 0.9 for
two continuous seconds permits 1.5x recovery of ordinary distraction, capped at
full awareness. This score is not a calibrated gaze or wakefulness probability.
The same protected-distraction and orange/red exclusions apply. The relaxation
is revoked immediately during the traffic override or unhealthy road context.

Missing, invalid, malformed or stale camera DM data selects interaction monitoring;
healthy data must persist for two seconds to recover. There is no attempt to
distinguish absent hardware from a fault. A driver sensor node missing at startup
no longer asserts the whole camera daemon; road/wide failure handling remains.
This does not make a shared camerad crash harmless or provide hardware hotplug
reinitialization. Shared road-camera failures still use their normal handling.

Interaction alert budgets are 5/15/25 seconds in mode 0 and 10/30/50 in mode 1.
A pedal/eligible speed-button response supplies a temporary +0.2 multiplier for
15 seconds. Another +0.2 requires ten seconds of stable, observed empty straight
road. The maximum factor is 2.4. Corner coverage requires enabled, supported
Hyundai corner radar and healthy radar/model data; no coverage or stale data is
not evidence of an empty road. Unhealthy road context forces factor 1.

New moving observations ahead or in adjacent lanes restore standard criteria for
ten seconds. Positional association, relative-motion projection, duplicate removal
and a two-second dropout hold prevent continuously visible cars from perpetually
retriggering the timer. Radar selection and detection algorithms are not modified.
Context-budget changes retain elapsed seconds, and orange/red cannot be cleared by
budget expansion. Camera-source transitions retain awareness progress and alert
stage instead of restoring an old stock-policy saved budget; the fraction is
preserved across the different vision/wheel budgets, not their absolute seconds.

CarState is read without conflation for interaction edges so a 100 Hz button press
is not lost in the 20 Hz policy. Held inputs do not repeatedly reset monitoring;
automatic vEgo/vCruise changes do not count. Vehicle speed buttons are excluded
when stock ACC uses automatic speed-button injection. BT has a separate bounded
`attention` journal emitted from the existing exclusive evdev reader. Unmapped
buttons can count, but learning, stale/startup/cancelled events and automatic
long-press repeats cannot. DM never consumes a driving-command journal.

## Health, alerts and diagnostics

Both modes retain DM warnings, lockout and the existing terminal-alert forceDecel
connection. A dedicated, low-priority camera-unavailable notice appears only while
a fresh, valid DM fallback is running. A dead camera-model process is exempted
from processNotRunning only under that same healthy fallback condition. DM output
itself is now required for normal cars. Road camera, model, CAN and other control
validity checks remain separate. No new steering timeout or guaranteed emergency
stop is introduced; stock ACC cannot be assumed to execute the same deceleration.

Appended driverMonitoringState fields record camera availability, experimental
selection, remaining traffic override, wheel factor, forward-attention evidence,
extra recovery and granted interaction seconds. They do not change radar schemas
or require a radar replay algorithm deployment.

## Validation and remaining limits

- 96 policy, daemon-adapter and Bluetooth tests: stock regression, mode-0 camera
  equivalence, sleep/eye/phone preservation, bounded input credit, forward recovery,
  camera loss/recovery, actual fallback packet production, Bluetooth event journals
  and pipe-fed daemon repeat handling.
- 70 context, camera-health selection, settings-schema and video-service tests.
- 41 web acknowledgement, typed-boolean and Carrot Vision regression tests.
- 25 Wiki generator/validator tests and candidate Wiki validation; Korean/English
  catalog and detailed guides are synchronized.
- New DM code passes Python lint and the change passes whitespace checks. Lint of
  all touched Python files reports the same 40 pre-existing diagnostics as HEAD,
  with no new diagnostics. The unchanged stock DM files are checked against Git.

Windows policy tests use real cereal schemas and policy calculations with adapters
for native IPC, Params, hardware and process-title imports. Bluetooth pipe tests
replace the OS flock call; they do not validate Linux exclusive-input ownership.
Reproduction adapters and reports are retained in the local analysis archive.
This is not a target-device native build, real camera-disconnection trial, hardware
timing test or driving validation. C3/C3X/C4 camera faults, recovery, sustained load,
actual input provenance, warning audibility, radar coverage and vehicle deceleration
still require controlled target testing. Neither setting is worldwide legal approval.

Public guides: [한국어](user/ko/driver-monitoring.md),
[English](user/en/driver-monitoring.md).
