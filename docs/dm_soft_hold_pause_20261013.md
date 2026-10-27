# Pause driver monitoring during stationary soft hold

The active dm2d dispatcher now pauses attention assessment and warnings while
softHoldActive is positive and the car is confirmed stationary. Both standard
and experimental DM, AlwaysOnDM, and the camera-unavailable interaction fallback
use the same pause. Driver View retains its diagnostic behavior.

Eligibility requires valid/fresh carState and selfdriveState (under 250 ms),
valid CAN, D/low gear, standstill, no gas pedal, and finite raw/filtered speed
with absolute values below 0.1 m/s. Releasing soft hold, pressing gas, moving,
or losing this evidence removes the pause on the next dispatcher iteration.

During the pause, neither camera attention assessment nor interaction-fallback
assessment runs. Ordinary 20 Hz driverMonitoringState publication continues,
using the existing dm2Disabled output contract to suppress warnings, HUD alerts
and DM forced-deceleration requests. No enable/session Params are written.
The monitor object, calibration, awareness, accumulated warning counts and
lockout history survive; soft hold is not a parking-reset or lockout-clear action.
On release, the retained monitor resumes the applicable camera/fallback path.

The camera and dmonitoringmodeld inference process keep running. This avoids
process/camera startup delay on departure; the change does not claim GPU savings
or pause neural inference itself. There are no driving-model, signal-control,
braking or process-manager changes.

Validation: the monitoring suite passes 269 tests, with four native Params
migration cases skipped on Windows. The new 27 cases include real Cap'n Proto
car-state validation and eight daemon-loop combinations of standard/experimental,
camera/fallback and AlwaysOnDM. Each holds for 120 iterations with no assessment
or warning accumulation, then resumes on the first release iteration. Existing
lockout survives the pause. Native IPC/hardware/Params access is substituted in
the desktop runner; monitoring policy, messaging Python code and schemas are real.

No vehicle installation, native device timing or road validation was performed.
Private test runner: `.analysis/scratch/2026-10-13-dm-soft-hold/run_tests.py`.


## Signal and stopped-lead extension, 2026-10-27

The same temporary assessment pause now also covers active longitudinal control
holding a full stop at a red signal (planner e2eStopped) or behind a stationary
primary lead (planner lead/hasLead, valid radar, finite lead speed below0.1m/s,
distance within40m). Car/control/plan/state packets must be valid and younger
than250ms; both raw/filtered ego speeds below0.1m/s, standstill, D/low and no gas.
The longitudinal controller must be stopping and target speed below0.1m/s.
Missing/stale data, departure target/controller, green/released signal or moving/
missing lead resumes assessment on the next dispatcher cycle, even before ego
motion. Existing SoftHold eligibility is retained. Camera inference continues;
no enable/session Params, longitudinal command or accumulated lockout state is
changed. Frozen assessment resumes; this is not a lockout reset.

Desktop:253 focused policy/dispatcher/catalog checks (overlapping earlier220),
including actual six-second dispatcher pause and next-cycle resume with camera
and fallback policies, experimental on/off, always-on on/off and all three stop
reasons. Web dropdown3 checks. Linux CI and vehicle installation follow separately.


Deployment: runtime cd80fae432 published only on carrot-nav-blink. CI40412207582
passes monitoring232, broad1634/56skip, codec62, updater94, NAV119, storage11,
network11 and Windows installer. Counts overlap desktop evidence.
Vehicle was actually offroad: fresh device startedfalse, Panda ignitionLine/Can
false, controlsAllowedfalse/noOutput. Normal Git Pull/reboot installed cd80fae432;
boot ID changed and actual web API lists only N16 Nav/Flip Flop with retired
MD selection rejected409. N16 preference/effective unchanged, DM enabled and
session disablefalse, snapshotfalse/offroad preserved. USB2CAN selection remains
SPASfalse/test0/mode2. MD PKL/cache was already absent; Flip cached files remain.
No Jetson runtime/model update or physical signal-stop/lead-stop trial was done.
Actual road assessment pause/resume acceptance is pending; synthetic dispatcher
coverage must not be described as a vehicle stop test.
