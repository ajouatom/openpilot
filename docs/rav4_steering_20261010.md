# RAV4 steering activation and event-cadence regression — 2026-10-10

## Interpretation

The report distinguishes an enabled lateral-selection switch from actual
steering assistance. `carState.latEnabled` selects whether lateral control may
operate; it does not independently engage control. With AlwaysLateral OFF,
the existing policy also requires active selfdrive engagement. With
AlwaysLateral ON, eligible driving gear and the existing speed/fault conditions
allow lateral control without cruise engagement. This distinction applies to
Toyota too. No new independent engagement mode is introduced here.

The submitted logs also establish a separate software regression: the
continuous readiness gate in incident revision `e7987ed0fd` rejects healthy
`onroadEvents` bursts. Pedal and steering-override transitions can therefore
interrupt an already active steering request. This is not evidence of a radar
collision or Panda rejecting the Toyota steering messages.

## Evidence

- Full NAS rlogs for RAV4 TSS2 2022 route `0000000d--18c3f226d3`, segments 5–9;
  full cereal schema and Toyota DBC decoding. Raw logs/settings stay local.
- Incident revision `e7987ed0fd07748eda8d0826f4bab2fe3c829ce2`, clean tree;
  Tizi device. Startup wall clock is 2026-10-10 12:36:57 KST. Segment-relative
  timestamps below use first carState, not the repeated startup initData time.
- CP: Toyota torque/PID steering, pcmCruise=true,
  openpilotLongitudinalControl=true, radarUnavailable=true, flags=1372,
  Toyota safetyParam=73. The prior experimental radar-disable configuration
  remains selected; stock-longitudinal mode was not being compared here.
- All 30,002 carState samples are CAN-valid, in Drive, latEnabled=true,
  cruiseState.enabled=false, with no decoded ACC or temporary/permanent steering
  fault. No physical buttonEvents are published. All 30,001 carControl samples
  have enabled=false and longActive=false. Thus these five minutes cannot
  establish behavior while longitudinal cruise is actually engaged.
- InitData records AlwaysLateral=1, AutoEngage=0, LfaButtonMode=2,
  LatSuspendAngleDeg=300. The nearest pre-start W: backup (12:35:31) and newest
  available backup (12:38:01) also have AlwaysLateral=1/LfaButtonMode=2; an older
  12:31:29 backup had LfaButtonMode=0. These snapshots do not capture every live
  setting change. The long inactive window across segments 6–7 cannot be
  attributed wholly to the readiness regression; an AlwaysLateral OFF interval
  is consistent with the code and report but not directly logged.
- Panda controlsAllowed=false throughout, yet bus-128 TX echoes contain nonzero
  steering requests. This fork's common torque checker separately permits its
  existing always-lateral path. All 2,807 Panda reports have safetyTxBlocked=0
  and safetyRxInvalid=0. CAN error/reset/bus-off counters do not increase over
  these segments. Historical nonzero counters are not new failures here.

| Segment | carControl latActive samples / total | Nonzero Toyota steering TX echoes |
|---|---:|---:|
| 5 | 3,677 / 6,002 | 3,666 |
| 6 | 111 / 5,999 | 112 |
| 7 | 0 / 6,002 | 0 |
| 8 | 5,614 / 6,000 | 5,597 |
| 9 | 3,138 / 5,998 | 3,130 |

TX/control counts differ slightly at segment boundaries and by publication
latency. They are not estimates of physical assistance or steering feel.

## Cause and correction

`selfdrived.publish()` sends onroadEvents once per second **and whenever event
membership changes**. Its registered frequency is 1 Hz. The ordinary
FrequencyTracker accepts approximately 0.8–1.2 Hz and can reject the faster
event-driven stream. The incident's shared `service_ready()` called
`all_checks()` for that stream, so the same false rejection affected both
controlsd and card. A later 1-second heartbeat could restore readiness, producing
repeated steering interruptions after ordinary input changes.

For example, segment 8 has carControl.latActive=false from 28.854 to 31.856 s,
while travelling about 42 km/h. The event-rate test alone rejects readiness;
model, pose, live geometry, CAN and steering-fault checks remain healthy. The
outgoing Toyota steering request goes inactive and its TX echo confirms it.

`service_ready()` now checks onroadEvents receipt, Event validity and liveness
without enforcing fixed publication frequency. Initialization events still
block first readiness. Other periodic input checks are unchanged.

During this investigation the separately requested startup-only revision
`7ceee11642` was committed, including this event-cadence correction. Its
controlsd/card readiness latches run until first readiness only; later input
changes or disengagement do not rearm startup protection. This investigation
preserves that policy. The event exception also prevents legitimate bursts
from unnecessarily delaying the initial latch. Steering gains, Toyota CAN
packing, Panda firmware, longitudinal behavior and engagement policy are
unchanged by this investigation.

## Validation and limits

- Timestamp-ordered replay uses the incident readiness code and production
  FrequencyTracker against 29,950 evaluable carControl frames. Logging begins
  mid-route, so unavailable initial history is excluded. Of these, 6,227 frames
  fail only the event frequency check; the corrected input predicate accepts
  those healthy inputs. This is not a count of frames that should necessarily
  steer: other engagement/speed conditions still apply.
- After excluding the first four seconds of segments 5 and 8, incident
  readiness plus the existing speed/standstill conditions matches all 11,201
  logged latActive decisions. Across their evaluable full windows, 2,644 frames
  (about 26.44 seconds at 100 Hz) fail the event check. Publication timestamps
  approximate receipt times; this is not a native scheduler/wire-timing replay.
- 166 focused desktop tests pass on the integrated startup-only code: event
  bursts in both subscriber modes, missing/invalid/stale inputs, initialization,
  first-readiness latches, startup controller behavior, steering ratio and
  lateral engagement gates. Toyota cases explicitly preserve the requirement
  for cruise engagement or AlwaysLateral.
- Separately, 116 tests passed with the event exception applied to the incident
  baseline before the independent startup-only revision. This included the
  continuous controlsd/card guard tests and startup controller tests.
- Focused Ruff checks pass. Test fixtures substitute Windows Params/hardware
  and execute production Python calculations without native IPC.
- No physical driving, EPS response or steering-feel validation is claimed.
  The logs do not prove the precise physical LKAS-button operation described
  in the report, nor do they contain an engaged-cruise comparison.

Private reproduction scripts, summaries and validation output are indexed in
`.analysis/archive/2026-10-10-rav4-steering/INDEX.md`.
