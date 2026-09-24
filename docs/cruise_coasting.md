# Cruise coasting margin

`CruiseCoastingPercent` defaults to 0, ranges from 0 to 10%, and uses one-percent
steps. Zero preserves the existing longitudinal output. The setting does not
raise cruise/MPC speed targets, change SCC modes, or add propulsion commands.

## Control policy

The longitudinal planner publishes `cruiseCoastingTarget` (physical m/s; zero
means ineligible) and `cruiseCoastingPercent`. It retains a fixed reference only
after the unmodified cruise target and eligibility have remained stable for one
second. Display-speed calibration is applied consistently with the existing MPC
target. A set-speed change resets permission immediately; small calibration
noise cannot ratchet the reference upward.

Permission requires openpilot longitudinal control, ACC rather than blended
mode, a solved cruise-source plan, valid current inputs, and a reference above
10 km/h. Both lead slots and the cut-in candidate must be absent. The cruise
source enum represents the first MPC obstacle node and is not sufficient proof
that a lead cannot request braking later in the horizon. This initial policy
deliberately leaves all detected-lead scenarios on the original controller.

Camera, speed-bump, section/school-zone and other navigation caps are checked
independently of the MPC source label: navigation targets are folded into the
cruise target upstream. Any external cap at or below the full coasting ceiling
blocks relief, including a cap above the original set speed. A previously seen
but stale navigation publisher also blocks permission. Signal-stop states block
relief before the low-speed `shouldStop` flag becomes true. Force-deceleration,
FCW, pedals, turn limiting, ATC, lane changes, reset/engagement, soft hold, economy
target increases, and the existing CarrotCruise mode also block permission.

`longcontrol` rechecks plan age (at most 0.2 s), source, pedals, set speed, both
leads/cut-in candidate, stopping, FCW and acceleration limits. The permitted band
is strictly above the fixed reference and below reference times (1 + percent/100).
Brake relief uses smoothstep ramps over the first 10% and final 40% of the band.
In the middle, a negative PID output can reach zero. The final 40% progressively
restores the ordinary negative output. Positive PID outputs retain their normal
path and are never increased by this feature.

Added relief increases at no more than 0.5 m/s^3. The speed envelope can restore
braking sooner, and a safety/target/input veto discards relief immediately; no
coasting comfort ramp delays requested camera, bump, lead or signal braking.
During suppressed negative output the PID integral is frozen at its previous
value. It is neither accumulated against a blocked actuator nor initialized to
an artificial positive value. P/feedforward continue to reflect the normal plan.
This does not promise a bounded jerk on a safety veto or a hard actual-speed
ceiling. Existing downstream vehicle jerk handling remains in effect.

The approach was informed by the historical
[Twilsonco GM output-stage coasting implementation](https://github.com/prodigz/openpilot-twilsonco-archive/blob/f1660891c8e364b11f9518f1a907b33f4c615885/selfdrive/car/gm/carcontroller.py#L238-L258).
The implementation here is separate: GM ZERO_GAS cannot be treated as equivalent
to a Hyundai SCC zero-acceleration request. Actual regeneration at zero request
and ride quality require vehicle validation.

## Validation and scope

Windows focused tests exercise production longcontrol/PID, the production
planner permission method (without importing native MPC), Cap'n Proto defaults
and round trips, and the speed-envelope helper. Cases cover:

- 600-cycle exact legacy-output/integral comparisons at 0% on Hyundai, GM and
  Toyota tuning paths, including parameter refreshes;
- continuous speed-band boundaries, limited brake release, no added positive
  acceleration, integral freeze, stale/missing metadata, live disable and target
  changes;
- camera/bump/HDA/section/school/ATC/curve/route caps while the MPC source still
  reads cruise; distant signal stopping before `shouldStop`; current leads and
  cut-in candidates; input and solver vetoes;
- existing stopping and lead-preview regression tests.

On 2026-09-24, the focused control suite passed 436 tests, the settings-schema
suite passed 44, the Wiki generator/validator suite passed 25, and the focused
web catalog/risk suite passed 12. Ruff, Python compilation, the user-docs check
and generated Wiki Markdown validation passed. The schema suite's pre-existing
radar-menu expectation was updated to include the already-shipped RadarTrackFlip
setting; radar behavior was not changed.

The full native MPC and vehicle CAN/SCC response have not been exercised by these
desktop tests. No driving validation or elimination of regenerative braking is
claimed. The MPC solver, radar detection/selection, navigation speed computation,
stopping thresholds and existing CarrotCruiseDecel controller remain unchanged.
The added longitudinal metadata does not change radar replay inputs or results.

User guides: [Korean](user/ko/cruise-gap.md#carrot-cruise),
[English](user/en/cruise-gap.md#carrot-cruise). The generated settings Wiki uses
the same catalog and separately reviewed manual descriptions.
