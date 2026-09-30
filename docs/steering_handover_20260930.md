# Experimental Hyundai angle-steering handover

The user requested selectable experiments after observing prolonged reduced steering
authority in a curve, then explicitly requested their combination and live setting
changes. `SteerHandoverMode` is persistent, defaults to **0**, and is polled every
50 controller frames (approximately 0.5 seconds). No restart is needed. Unsupported
values select 0. An actual change clears experimental evidence, while an unchanged
read preserves it. The existing recovery state continues independently in every mode.

## Modes and scope

| Mode | Method | Selection |
|---|---|---|
| 0 | Existing recovery | Exact legacy authority output |
| 1 | Limited authority offer | Joint error/effort convergence with error tolerance and gradual withdrawal |
| 2 | Abrupt-release recovery | Early limited capture, then error-dependent ceiling rise after low-force confirmation |
| 3 | Combined | Mode 2 takes priority over mode 1; increases are never added |

Only Hyundai/Kia/Genesis `ANGLE_CONTROL` uses the helper. Both camera-SCC and
non-camera-SCC steering transmitters receive the selected ceiling; `torqueOutputCan`
reports the selected pre-quantization value. Torque-control platforms, angle-command
limits, CAN safety checks, `steeringPressed`, touch reception and DM are unchanged.
The ceiling affects potential physical assistance; it is **not** a calibrated
physical torque command or a guaranteed tactile notification.

## Current signal and transition contract (September 30 follow-up)

The user explicitly selected torque-ceiling-only recovery in the follow-up.
Do not add a captured-angle reference, offset, or target blend. The existing
model target, angle rate/acceleration limits and steering CAN angle fields remain
unchanged. This supersedes the offset-blending design review in
`steering_handover_ff6_ff7_20260930.md`.

- Driver effort remains abs(steeringTorque) / STEER_THRESHOLD with a three-sample
  magnitude median and 0.12 s low-pass filter; it is not clipped at 2. Raw force
  remains available for prompt yielding, and signed force for offer opposition.
- Error is the greater magnitude of target-minus-actual and limited-command-minus-
  actual angle, with a 0.08 s filter. The full-offer tolerance is min(3 degrees,
  the bicycle-model 0.8 m/s^2 angle-error equivalent at current speed). Above it,
  offer quality rolls off linearly to zero at twice that tolerance. These constants
  are experimental calibrations, not a validated vehicle operating envelope.
- Mode 1 requires 0.25 s of qualifying evidence after a stable force direction and
  about 0.2 s of history. Both levels and trends matter: error must be inside the
  tolerance or decrease by more than 0.05 degrees over the history window; effort
  may rise by at most 0.05. Stable large error does not qualify. Raw and filtered
  effort limit the offer to 80 through strength 1.3, 45 at 2, and 25 at 3.
- During an offer, an unclear convergence trend pauses increases; a lower
  error/force-dependent ceiling reduces authority at 110 CAN units/s. There is
  no derivative-only error veto. Opposite raw force above 0.7, raw force above
  3.2 or initial effort +0.4, filtered effort above initial +0.15, or a history
  effort increase above 0.2 triggers fast yielding at 2,000 units/s. These are
  force-based heuristics, not proof of driver intent.
- No effort reduction of 0.1 after 0.6 s, or a 1.5 s offer duration, starts gradual
  withdrawal at 110 units/s. Renewed strong effort still interrupts withdrawal
  quickly. Repeated offers are blocked until low unpressed force is confirmed.
  A mode-1 release held low for 0.1 s holds the offered ceiling while the legacy
  recovery catches up, then joins it with at most 110 units/s of increase.
- Modes 2/3 arm after 0.15 s of sustained, same-direction pressed effort above 1.
  Within 0.5 s of that evidence, unpressed raw/median force at or below 1, a
  0.25 decrease from a recent peak of at least 1.05, and a decline rate of at
  least 0.5/s must persist for 30 ms. A 0.4 s force history supplies this test.
  Capture then moves toward a ceiling of 45 at 110 units/s; an existing mode-1
  offer above 45 also approaches it gradually. Angle error does not gate entry.
- After raw/median effort stays at or below 0.6, unpressed, for 60 ms, recovery
  rises toward maximum. Its rate is (maximum-minimum)/0.5 multiplied by
  clip(tolerance / max(tolerance, raw error, filtered error), 0.25, 1).
  With 25/250 bounds this is 112.5 to 450 CAN units/s. Small error permits faster
  recovery, large error slows it, and changing error updates the rate every tick.
  Error alone neither prevents recovery nor suddenly reduces the current ceiling.
- Capture without confirmed low force expires after 0.3 s and withdraws gradually.
  Pressed status or renewed raw force cancels capture/recovery promptly. Capture
  uses max(0.8, initial strength +0.15); confirmed recovery uses 0.8. Mode 3
  selects capture ahead of an offer, without adding the two ramp increments.
- While an experimental transition is active, its selected total ceiling replaces
  max(legacy, extra), so a larger legacy ceiling cannot bypass the chosen ramp.
  Legacy override/repetition state remains untouched. After yielding, 0.2 s of
  low unpressed force permits bounded handback to legacy; an accepted mode-1
  offer does not drop solely because legacy is still release-latched.
- Healthy CAN, no steering fault, fresh changing model frame ID within 150 ms,
  finite yStd[10] in [0, 0.3], finite inputs and consecutive timestamps within
  30 ms remain required. Invalidity clears evidence and returns at most both the
  legacy and previously owned ceilings; it cannot jump a reduced ceiling upward.
  Deactivation and actual mode changes reset experimental state immediately.
  Switching to mode 0 selects legacy behavior immediately, so a mode change is
  not itself a rate-blended handover.
- Diagnostics log state transitions as well as one sample per second. CAN and
  carOutput retain the actual output record; logging is not physical torque sensing.

## Follow-up validation

- 88 focused helper/controller/steering-mode tests pass, with only native Params
  storage substituted on Windows. Coverage includes small-error tolerance, both
  force/error trends, gradual versus fast withdrawal, release noise, error- and
  speed-dependent recovery, large-error entry, legacy-cap bypass prevention,
  invalidity, mode polling, all directed mode changes, and both CAN-FD paths.
- Identical inputs produce identical angle outputs in all four modes, including
  release with substantial angle error and a changing model target. A separate
  6,000-frame full-controller replay preserves mode-0 angle, authority and steering
  CAN bytes exactly against pre-feature 0c492cd14c. The legacy block is unchanged.
- On the supplied ff6/ff7 input traces, the updated helper begins limited capture
  32/32/95 ms after the 5.488/13.001/50.714 s pressed falling edges. Recorded legacy
  recovery began after 350/320/520 ms. Several other releases are captured, while
  two ff6 releases still do not qualify. These are output scheduling comparisons
  on unchanged feedback, not measured steering response improvements.
- Full-trace mode-1 timing differs from the already-started offer study: ff7 now
  offers at 48.096 s and withdraws at 48.629 s when filtered effort increases from
  the initial 1.934 to 2.085. It remains blocked at the old 50.078 s error-only
  withdrawal point, then captures release at 50.809 s. Do not claim the earlier
  isolated 47-versus-25 comparison is the final full-trace output.
- A sustained zero crossing can still look like release before force returns;
  the regression test retains that counterexample and checks renewed-force yield.
  Faster force detection does not establish hand removal or consent. No physical
  vehicle response, closed-loop stability, steering feel or road benefit is proven.
- The 25 focused Wiki tests and user-documentation checks pass. Generation
  against the existing Wiki validates 187 settings / 564 generated files with
  zero structural errors; 28 unrelated pre-existing description warnings remain.
  Korean/English guides and all three catalog descriptions match the revision.

Private reproduction scripts and outputs are indexed in
`.analysis/archive/2026-09-30/handover-update/`.

## Initial implementation evidence and limits (historical)

The incident's final curve had continued driver effort with the authority ceiling
at 25. At some instants actual steering and target were close despite continued
pressed status. Neither close angles nor pressed status identifies the driver's
intent: low assistance can itself require continued driver effort.

Desktop validation:

- 62 new helper/controller tests cover noise, strong grip, reversal, growing error,
  release timing, invalid/stale inputs, mode priority, bounded ramps, CAN ceiling
  packing, torque-platform equivalence, disengagement, all 12 directed runtime
  mode transitions, unchanged polls and preservation of legacy state.
- 12 existing steering-mode selection tests also pass. On Windows only native
  Params storage is substituted with an in-memory implementation; no physical
  Params filesystem/device timing is validated by this harness.
- A 6,000-frame recorded-input controller comparison against `0c492cd14c` gives
  identical mode-0 steering angles, authority values and generated steering CAN
  bytes. The complete legacy authority block is unchanged. Replay supplies model
  freshness at the recorded stream's verified continuous 20 Hz cadence and uses
  the exported vehicle-speed trace; it is not a full process replay.
- The replay's modes 1/3 offer from approximately 57.058 s, reaching a maximum
  ceiling of 42.283 in the final curve before withdrawing as error grows at
  57.878 s. Mode 2 alone leaves that held-force curve at 25. Modes 2/3 start rapid
  recovery at 52.151 s, about 67 ms before the legacy recovery start at 52.218 s.
  These are **counterfactual output calculations on unchanged recorded inputs**,
  not proof of faster physical steering or a driver's reaction to added assistance.
- 25 settings-Wiki tests pass; generation/validation covers 187 settings, three
  languages and 563 generated files. Korean/English setting guides and catalog
  descriptions include all four modes and runtime behavior.

A long zero-force interval during a steering reversal can be indistinguishable
from release until force returns. The tests explicitly retain this counterexample:
a 160 ms interval can trigger recovery, followed by withdrawal on renewed force.
No force-only algorithm can establish future driver intent. Sensor noise in this
recording is not the full vehicle population. Physical steering feel, conflict with
the driver, closed-loop lateral dynamics and actual road performance remain
unvalidated. Modes 1–3 are for controlled experiments; mode 0 remains the default.

Private input traces and reproduction scripts are indexed under
`.analysis/archive/2026-09-30/steering-handover-implementation/`; they are not shipped
or committed. Earlier investigation/prototype evidence remains in the neighboring
`ioniq-curve-handover` and `handover-validation` archives.
