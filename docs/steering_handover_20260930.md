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
| 1 | Limited authority offer | Small/converging angle error and stable/decreasing driver effort |
| 2 | Abrupt-release recovery | Sustained override followed by confirmed rapid force release |
| 3 | Combined | Mode 2 takes priority over mode 1; increases are never added |

Only Hyundai/Kia/Genesis `ANGLE_CONTROL` uses the helper. Both camera-SCC and
non-camera-SCC steering transmitters receive the selected ceiling; `torqueOutputCan`
reports the selected pre-quantization value. Torque-control platforms, angle-command
limits, CAN safety checks, `steeringPressed`, touch reception and DM are unchanged.
The ceiling affects potential physical assistance; it is **not** a calibrated
physical torque command or a guaranteed tactile notification.

## Signal and transition contract

- Driver effort is `abs(steeringTorque) / STEER_THRESHOLD`, with a three-sample
  magnitude median and 0.12-second low-pass filter. It is deliberately not clipped
  at 2: the observed late-curve effort often exceeded 2. Signed/raw force remains
  available for reversal and rising-force vetoes. Alternating force cannot cancel
  itself into a false release through a signed low-pass filter.
- Error uses the greater magnitude of target-minus-actual and limited-command-minus-
  actual steering angle, filtered at 0.08 seconds. Both raw and filtered error must
  fit a two-degree ceiling, tightened at speed by a bicycle-model 0.5 m/s² error
  equivalent. This is an experimental gate, not a validated vehicle envelope.
- Mode 1 requires 0.25 seconds of qualifying evidence, a stable force direction,
  and either a small/stable error or a decreasing bounded error. It ramps the
  offered CAN ceiling at 110 units/s. The upper ceiling is 80 through effort 1.3,
  falling linearly to 45 at 2 and 25 at 3. Both current raw and filtered effort
  constrain it. Effort above the threshold can therefore receive a limited offer,
  without changing the existing boolean pressed signal.
- Opposing/increasing force or growing/out-of-bounds error withdraws the offer.
  An effort decrease of at least 0.1 is needed after 0.6 seconds; the offer lasts
  no more than 1.5 seconds. Rejection blocks repeated offers until 0.2 seconds of
  confirmed low, unpressed force. Invalidity interrupting an offer preserves this
  block. With a confirmed release, mode 1 holds only the offered ceiling while
  legacy recovery catches up.
- Mode 2 arms after 0.3 seconds of sustained pressed effort above 1. It requires
  raw and median effort at or below 0.3, unpressed, reached within 0.25 seconds of
  strong effort and maintained for 0.1 seconds. Only bounded angle error permits
  faster recovery. Its ramp is 450 CAN units/s (25→250 in 0.5 seconds), matching
  the fastest legacy ramp but starting without the legacy filtered-release wait.
  It can also bypass the legacy repeated-override delay while its own conditions
  hold. Renewed pressed force, raw effort above 0.6 or growing/unbounded error
  withdraws the additional authority.
- Mode 3 selects the rapid branch before processing the offer branch. It never
  applies both ramp increments in one update. Withdrawal reduces the experimental
  ceiling at 2,000 units/s; mode change, disengagement or invalidity discards it
  immediately. The selected output never falls below the independently calculated
  legacy ceiling. The helper cannot erase the legacy override or repetition latch.
- Experimental recovery requires valid CAN, no reported steering fault, a present
  model with a changing frame ID within 150 ms, finite one-second position yStd in
  [0, 0.3], finite inputs and consecutive controller timestamps within 30 ms.
  Failure grants no additional authority. These checks do not replace existing
  model/control validity gates.
- Diagnostic logging records mode changes and one sample per second of state,
  effort, error, legacy ceiling and selected ceiling. It is sampled diagnostic
  output, not a complete transition trace. CAN/carOutput remain the output record.

## Evidence and limits

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
