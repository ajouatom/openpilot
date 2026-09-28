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

DriverMonitoringMode is latched at startup: only value 1 is experimental. Old
DisableDM never opts users into mode 1; only its old video choice migrates once to
independent CarrotVisionEnabled. The experimental-use confirmation remains.

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
