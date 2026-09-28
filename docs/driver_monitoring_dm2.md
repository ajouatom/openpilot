# DM2 implementation and validation - 2026-09-28

## Revised user contract

The user's follow-up supersedes the initial implementation's 5/15/25 fallback,
2x-to-2.4x wheel timing, two-second camera input credit, protected distraction
debt exclusion, 1.5x forward recovery and ten-second traffic window. The user
explicitly selected delaying camera warnings for the unavailable-camera mode-1
allowance after input, rather than immediately resuming camera evaluation.

- Camera mode 0: stock comma policy, including stock internal face-loss fallback.
- Unavailable-camera mode 0: 15/30/45-second interaction alerts.
- Unavailable-camera mode 1: same timing, doubled only with verified empty-road
  conditions; eligible input resets the entire allowance before terminal alert.
- Camera mode 1: 10/16/26-second vision alerts; 20/32/52 with empty-road conditions.
  Head-pose tolerance is 20% wider before orange. Sleep, blink and phone detection
  thresholds are unchanged, but warning timing for those detections is extended.
- Fresh driver/BT input in camera mode 1 resets monitoring and starts 45 seconds
  of camera-warning grace, or 90 seconds with empty-road conditions. Detections
  continue during grace, including sleep/eye/phone, but the warning budget remains
  full. After grace, the camera clock starts. Under sustained distraction, this
  can place the first warning around 55/110 seconds and terminal around 71/142
  seconds after input. These are requested experimental choices, not legal limits.
- Confident forward attention for two seconds resets camera mode 1 without
  renewing the interaction grace. Existing terminal alerts and lockout cannot be
  reset by input, forward attention, camera recovery or empty-road expansion.

## Stock boundary and state transitions

Stock monitoring/policy.py and monitoring/dmonitoringd.py remain unchanged. The
manager runs the separate dm2d dispatcher at the existing scheduling placement.
DriverMonitoring2 modifies its own settings instance, retaining normal camera
mode-0 equivalence. Models, artifacts and inference behavior do not change.

Camera absence, failure, malformed probabilities/vectors and stale output select
automatic interaction fallback. Two continuous seconds of healthy samples restore
camera monitoring. Source transitions preserve fractional monitoring progress and
strong alerts; they do not preserve identical absolute seconds across different
source budgets. No manual camera-installed setting is required. Road/wide camera,
CAN, process and other control health checks remain independent. A shared camerad
failure is not rendered harmless, and this does not add hardware hotplug recovery.

Within one source, changing traffic context preserves elapsed attention seconds.
Expansion cannot erase orange/red. A shortening that crosses terminal counts the
terminal transition once. Fresh input may reset before terminal; once terminal
is reached the stock disengagement/lockout behavior remains. A grace that expires
cannot reopen merely because the road clears later. Mode 0 camera inputs stay
stock; mode 1 uses fresh edges rather than a continuously held steering/gas state.

## Traffic and input evidence

DM only consumes existing radarState leads and lists. No radar detection, selection,
shared replay adapter or radar model is changed, so this task does not deploy a new
NAS radar replay algorithm. Appended fields describe DM only.

Moving observations use absolute ground speed >=2 m/s, longitudinal position
-10..150 m and path lateral distance <=6 m. Equal-speed lead traffic counts.
Stationary/slow observations are intentionally excluded; this is not proof of an
obstacle-free road. A candidate immediately removes the empty-road bonus; 0.2 s
of continuous confirmation starts a 20-second hold. Position association, relative
velocity projection, deduplication and two-second dropout retention prevent each
frame of one car from retriggering. A single spike cannot start the full hold.
These gates reduce some noise; they do not establish physical target identity.

Empty-road doubling additionally requires healthy radar/model, supported enabled
Hyundai/Kia corner coverage and ten seconds of stable straight-road conditions.
A continuously present moving object blocks this condition after the hold ends.
Unknown coverage, stale input, curves or invalid observations cannot earn it.

CarState is drained without conflation to retain short control edges. Stock-ACC
speed-button injection configurations exclude ambiguous vehicle speed buttons.
Other eligible controls and the existing independent BT attention journal remain.
Learning/test, stale/startup/cancelled events and long-press automatic repeats do
not count. Automatic ego/set-speed changes are never driver interactions.

## Configuration, diagnostics and documentation

### Original steering touch input (2026-09-28)

Ioniq 5 PE alone enables the received `STEER_TOUCH_2AF` profile. `CarState.steeringTouch`
records availability, validity, contact, original CAN timestamp and raw status /
TOUCH1 / TOUCH2. It reads the existing ECAN parser's raw bytes, never the mutable
forwarding cache, CAM input or Panda transmit receipts. Existing ADAS transmit
code, safety rules, message registration and global torque-based steeringPressed
remain unchanged. This bus distinction does not authenticate a sensor against
another device injecting frames onto ECAN.

Six one-minute historical segments yielded 3,597 original 10 Hz frames. All fit
the receive checksum profile (CRC polynomial 0x1D, initial register zero, xor
residual 0x32 across bytes 1..7) and a high-nibble counter 0..14. Three newer
segments established the profile and three older segments independently checked
it. This is empirical compatibility evidence, not an OEM protocol specification.
The existing transmit checksum function differs and was not changed.

Status 0 had raw TOUCH1 12..17 / TOUCH2 14..18; contact status 1 already included
TOUCH1 15 and TOUCH2 14 on different samples. Raw ranges overlap, so a baseline
plus one threshold is not established by these logs. The user confirmed that
zero means released and a rising value represents touch. The decoder accepts
the lowest reported contact status 1, through observed status 4. All 313 samples
with logged steeringPressed were in nonzero status, including turns; this is
corroboration, not hand-labeled physical-contact ground truth.

The decoder requires the observed eight-byte layout, checksum, known status,
successive counters and sample age <=250 ms. Startup/recovery requires two
consecutive valid frames; malformed, repeated-counter, unknown-layout or stale
input grants no contact. Replay accepted 3,591 frames after the six initial
counter baselines, including 1,015 contacts. No new mandatory CAN checks are added.

Without a usable camera, fresh continuous contact maintains wheel awareness
before terminal alert in both modes. In camera mode 1 only a valid release-to-
contact transition grants the existing interaction grace. Held contact and
recovery while already held cannot renew camera grace. Camera mode 0 ignores
this added signal, and terminal/lockout handling remains unchanged. No claim
of gaze, sleep detection, legal certification or new-vehicle validation follows
from capacitive contact or these desktop/log checks.

### Hidden legacy override

The local legacy-override restoration retains DM2 driver-camera fallback: only
road/wide-road camera packets participate in selfdrived's camera-fault checks.
With monitoring enabled, driverMonitoringState health remains required. An
unavailable driver camera therefore uses interaction monitoring while a failed
DM dispatcher, road camera or vehicle communication retains its error handling.

DisableDM is marked search_only in the settings catalog. Ordinary groups and
substring searches omit it; a full case-insensitive parameter-name query reveals
the existing control. Public Wiki generation excludes search-only settings.
Explicitly saved favorites/profiles and parameter persistence retain their
existing behavior.

The DisableDM=2 follow-up applies the saved override once at manager startup to
the internal, non-default DisableDMActive parameter. It is cleared on manager
startup and excluded from normal parameter backups. Manager, selfdrived and
controlsd use this snapshot, including when a child process restarts. Saving a
different DisableDM value cannot stop/start DM or change warning handling during
the current session; reboot applies it consistently. The legacy forceDecel
condition is otherwise unchanged.

Carrot Vision eligibility is CarrotVisionEnabled OR applied DisableDM=2, subject
to the existing ClusterHud exclusion. The web runtime and resource diagnostics
read the same applied value, including the externally launched web server; they
do not depend on inheriting manager's environment. Old values still migrate the
independent streaming preference once. All three introductory presets explicitly
save DisableDM=0 and DriverMonitoringMode=0 without changing CarrotVisionEnabled;
the reset takes effect on reboot.

DriverMonitoringMode is latched at startup: only value 1 is experimental. Old
DisableDM never opts users into mode 1; its old video choice still migrates once to
CarrotVisionEnabled. The experimental-use confirmation remains.

New dm2VisionTimeoutFactor and dm2InteractionGraceRemaining fields complement the
existing wheel factor and traffic hold. dm2ForwardRecovery now identifies a full
forward-attention reset, and dm2InteractionCredit reports the full seconds restored
by an input, not a fixed two-second credit. Historical packets retain their earlier
semantics. Camera-unavailable notices and terminal forceDecel connections remain;
this is not guaranteed emergency stopping or equivalent stock-ACC deceleration.

Korean/English guides and the localized catalog explain all four cases, grace
composition, delayed sleep warnings, moving-target exclusions and terminal limits.
Wiki MANUAL explanations are updated separately through the existing generator.

## Validation and limits

After rebasing onto 3e2ea2ec, the DisableDM=2 fixes passed 119 focused Python
tests and 30 web tests. These
include all nine saved/applied mode transitions, restarting either control child
before reboot, twelve DM/video/cluster combinations, all three preset reset
paths from modes 1/2, the externally served video status and hidden-setting search.
The control/manager contracts execute extracted source predicates without native
imports; fake Params and HTTP responses stand in for device storage and IPC.
The web build, user-docs check and 26 Wiki tests also passed (175 distinct tests
in total). Python syntax and changed-line whitespace checks passed. Focused lint
found no new diagnostics; 18 existing findings in controlsd, vision_test and the
intro preset module were compared with HEAD and left outside this change.
Both DM integer writes use put_int, and the desktop Params doubles reject
incorrect value types. Four native Params tests cover the applied snapshot and
pending values, but were skipped on Windows because params_pyx is not built.

The visibility/fallback follow-up passed 18 camera/context tests, 47 settings
schema tests, 229 web settings tests and 26 Wiki generator/validator tests.
The web build and user-docs validator passed. A smoke check using the real server
catalog and generated browser bundle confirmed ordinary/partial-search exclusion
and full-name lookup. These desktop checks do not validate native IPC or a device
camera-loss event.

- 103 stock/DM2 policy, daemon-adapter and Bluetooth tests passed. Coverage includes
  all four modes, exact timing stages, sustained sleep/eye/phone detection, actual
  dispatcher BT grace, reset/expiry, context shortening, source recovery, terminal
  counters, lockout and stock camera mode-0 equivalence.
- 60 traffic/context/config and settings-schema tests passed, including noise
  persistence, twenty-second entry hold, dropout association and empty-road gates.
- Focused Python lint passed. Stock policy and daemon content remain unchanged.
- User-docs and Wiki validation accompany publication; generator tests are run.

The Windows adapter uses real cereal schemas, policies and numpy, but substitutes
native messaging, Params, hardware, process-title and flock interfaces. It does not
validate Linux IPC or exclusive evdev ownership. No device native build, hardware
camera-loss trial, C3/C4 timing, physical input provenance or driving validation was
performed. Neither mode is legal certification. Reproduction scripts and reports
are retained locally under .analysis/archive/2026-09-28/dm2-revision/.

Public guides: [Korean](user/ko/driver-monitoring.md),
[English](user/en/driver-monitoring.md).
