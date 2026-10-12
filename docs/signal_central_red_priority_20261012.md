# Central red priority during signal preparation — 2026-10-12

The user requested spatial priority instead of cancelling moving preparation for
every red/green mixture. A central red, or multiple agreeing reds near the ego
lane center, may now take priority over clearly lateral greens. This only affects
experimental pre-deceleration. It never grants departure or changes the separate
stopped-red departure guard.

## Geometry and decision

`housing_alignment` transforms both horizontal ends of each measured housing with
camera calibration. Across that housing's entire accepted distance interval it
compares lateral position with the unexpanded inner lane boundaries. Values are
signed fractions of ego-lane half-width: zero is lane center and ±1 are its edges.
It uses every road/lane x knot and the interval endpoints, including bends between
uniformly sampled distances. It uses housing position independent of lamp color.

Confident ego-lane lines are required for this exception. Road edges may constrain
the search and reject contradictory geometry, but road center alone cannot name
the ego lane. Clipping against road edges must not shift the lane center. No
screen-center fallback, curve extrapolation, guessed range or relaxing the existing
25–140 m full-range requirement is introduced.

The experimental bands are explicit heuristics, not learned probabilities:

* A single red can supply priority when its complete projected housing lies within
  ±0.75 half-lane-widths over its entire plausible range.
* Without such a central red, at least two distinct eligible red housings may supply
  priority if each fits within ±1.25 half-lane-widths. Both must independently earn
  the existing red confirmation; the existing overlap, row, scale, range and
  maximum-four-red checks still apply.
* Every eligible green must lie wholly outside ±1.75 half-lane-widths and at least
  0.5 half-lane-width farther out than the red support. An unknown alignment, a
  central/nearby green, overlapping boxes or conflicting identities cancels the
  exception. Counting more reds cannot defeat an equally central green.

The same support IDs and competing green IDs must maintain this separation for
at least four distinct captures spanning 0.6 s. A new competing green or changed
support starts that confirmation again. Each red also retains its independent
confirmation requirement. Loss of a two-red support member removes that exception
unless a new valid support earns confirmation. Conflicting geometry cancels
immediately; repeated frames do not accumulate time evidence.

Red-only operation retains the previous multi-housing behavior. Without alignment
information, mixed colors retain the previous veto. Mode 1 records/notifies without
applying the cap; mode 2 may apply the existing gradual cap toward −0.5 m/s².
Lead presence, turning/blinkers, gas/brake, speed eligibility, freshness, existing
model-stop handover and stronger braking remain unchanged. The original model
continues to determine the stop trajectory; this is not a new stop-line estimator.

Diagnostics add `priority` (`red_only`, `central_red`, `central_red_pair`),
`priority_ids` and `side_green_ids`. Existing `track_ids` still includes all
eligible reds. The runtime supplies calibration/lanes to the alignment callback,
and the planner passes it to the preparation gate. Display persistence does not
grant these control permissions.

## Verification

428 focused tests pass on Windows with only native Params storage substituted.
New tests cover single-center and paired-near-center red priority, central green
veto, all competing greens, uncertain/invalid bounds, overlapping detections,
identity changes, repeated captures, source loss, lane center versus image center,
full range and intermediate curve knots, absence of lanes, actual road projection,
runtime transport, modes 1/2, pedal/lead/turn rejection and original model handover.
Ruff passes for the preparation module and the new test file. Existing NumPy
shape deprecation warnings remain.

The previous committed gate and revised gate were compared using stored detector
outputs and context extracted from native rlogs: 3,348 earlier pose-observer inputs
and 981 corrected recorded-cadence observer inputs, totaling 4,329 gate updates.
Some footage overlaps between observer variants; these are not independent labeled
examples. Both versions have zero active preparation updates and zero entries.
No qualified red/green mixture reaches the exception: only six earlier and 89
corrected red track observations pass the appearance/history screen, and none
passes the full geometry screen. No new camera inference was run for this replay.

Replay reconstructs context eligibility, not the whole controller: `entry_allowed`
is set true because internal planner counters are not reconstructed; segment
boundaries start fresh state. It uses the original recorded delivery times and
vehicle/model context, not simulated responses to altered braking. Synthetic tests
exercise the positive branch; these logs do not establish real-world effectiveness,
false-positive rate, earlier braking or correct signal-to-movement assignment.

The remaining ambiguity is material: spatially centered signals can control a
different movement, and several housings may repeat the same irrelevant phase.
This revision supplies the requested bounded priority hypothesis, not proof that
the selected red controls this vehicle. Distance/boundary eligibility remains a
separate limitation and is not silently weakened to create replay entries.

Private script, pinned previous gate and per-segment results are retained under
`.analysis/archive/2026-10-12-signal-priority/`. Code is committed on `carrot-wip`
with a single parent. No vehicle installation or restart is performed here.
