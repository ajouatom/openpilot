# Niro tau-recovery candidate: recorded-input rejection screening

Follow-up alternatives and stopping-decision screening:
[Niro stop alternatives](niro_stop_alternatives_20261004.md).

## Result

The proposed causal tau recovery is implementable, but the tested candidates
do not establish a useful improvement on these logs. Some moments improve;
others become more optimistic before the lead brakes again. Do not promote
these candidates or weaken production braking based on the earlier hindsight
examples. This rejects the tested simple rules as ready-to-deploy fixes, not
every possible tau recovery design.

No production code, settings, model, vehicle state, or NAS service changed.
The user authorized local advance verification and previously asked not to
force a speculative fix.

## What was actually replayed

- Full cereal rlogs for Niro `000002c3--4363018d77` segments 0–1 and
  `000002c5--e88ae7f308` segments 15–16, four observed stop approaches total.
- 4,508 published radarState samples were processed chronologically. Original
  distance, velocity, acceleration, jerk, identity and baseline tau remain
  fixed. Only a causal candidate recovery floor changes tau.
- The production `LongitudinalMpc.extrapolate_lead` function was executed in
  isolation with its actual 13-node, 10-second horizon and discrete integration.
  Lead position and stopped-equivalent obstacle changes were calculated.
- Observed future lead displacement is reconstructed from integrated measured
  ego speed plus change in range. Interpolation handles time alignment. Require
  continuous valid measured radar identity through the comparison horizon,
  no >150 ms radar gap, active control and no pedal input at the evaluated
  instant. Comparisons use moving, braking lead snapshots in predeclared windows.
- Main windows: 2c3 t=48–62; 2c5 t=9–18, 49–57 and 89–100 seconds from each
  route's first analyzed carState. Evaluated horizons are 2, 4 and 6 seconds.

This is a **prediction-layer recorded-input replay**, not a full radard,
fast-overlay or native MPC replay. The local checkout lacks the generated
longitudinal acados module and WSL has no usable installation. No substitute
solver is presented as native MPC. Ego acceleration commands, alternate ego
speed, jerk, stopping position, clearance and ride comfort were not simulated.
The replay does not establish vehicle safety or improvement.

## Candidates, including the unsuccessful first attempt

All decisions use current and past input only. Future logged motion is used
solely for scoring. The baseline is the **recorded tau**, so this isolates an
additional recovery floor rather than silently reconstructing a different
baseline or modifying the driving model.

Initial design, fixed before its first run:

- Remember peak braking magnitude within a selected-lead episode, requiring
  at least 0.8 m/s² peak braking.
- Qualify easing of at least max(0.4 m/s², 30% of that peak), with nonnegative
  reported jerk and no >0.05 m/s² one-sample strengthening, for 0.30 seconds.
- Then approach `cap × fraction_of_braking_released` with a 0.50-second rise
  time constant. Output is max(recorded tau, recovery floor).
- Cancel the floor on jerk below -0.3 m/s³ or >0.05 m/s² strengthening.
  A rebraking cancellation starts a new episode peak from current braking.
- Invalid/nonfinite input, identity changes and >150 ms sample gaps reset
  history. Duplicate timestamps do not advance confirmation. Near-stopped
  leads <=0.3 m/s and nonbraking a>=-0.1 m/s² receive no recovery floor.
- Caps 0.05, 0.10 and 0.20 are sensitivity variants, not per-event tuning.

The first result showed repeated cancellations on small acceleration changes.
A second, explicitly post-result diagnostic variant increased the one-sample
tolerance to 0.15 m/s², allowed reported jerk down to -0.1 while qualifying,
and used -0.5 m/s³ for jerk cancellation. Other rules and all three caps stayed
the same. This is sensitivity exploration on the same data, not an independent
holdout validation or a chosen production calibration.

Synthetic checks cover duplicates, identity changes, stream gaps, invalid and
nonfinite input, stopped/accelerating leads, renewed braking, sustained braking,
already-high baseline tau, and brief alternating easing. Both variants pass.
Recorded replay checks baseline preservation on declared renewed braking and
the bounds `baseline <= candidate <= max(baseline, cap)` on every sample.
These checks verify the prototype mechanics, not physical safety.

## Outcomes

Four-second lead-travel mean absolute error, metres, for cap=0.10:

| Route / variant | Baseline | Candidate | Changed snapshots in approach windows |
| --- | --- | --- | --- |
| 2c3 / initial | 3.5086 | 3.5153 | 17 |
| 2c3 / tolerant | 3.5086 | 3.5211 | 72 |
| 2c5 / initial | 3.9246 | 3.9276 | 58 |
| 2c5 / tolerant | 3.9246 | 3.9312 | 73 |

There are 247 eligible four-second comparisons for 2c3 and 468 for 2c5 per
candidate. Initial and tolerant variants, at all three caps, provide no
aggregate four- or six-second error improvement. Tolerant recovery slightly
improves two-second error on 2c3 (0.9547→0.9521 m at cap=0.10), but not on
2c5. Aggregate differences are small and not evidence of statistically
significant degradation; the important result is the absence of demonstrated
benefit and the individual optimistic counterexamples.

Tolerant cap=0.10 examples, 4-second future travel in metres:

| Snapshot time | Baseline prediction | Candidate | Observed reconstruction | Interpretation |
| --- | --- | --- | --- | --- |
| 2c3 51.989 s | 38.59 | 39.54 | 39.63 | Improved after observed easing. |
| 2c3 54.992 s | 14.67 | 14.67 | 23.75 | No recovery yet: current evidence is not durable easing. |
| 2c3 55.591 s | 16.13 | 16.86 | 21.12 | Partial improvement after easing is confirmed. |
| 2c3 56.491 s | 21.94 | 22.47 | 16.89 | More optimistic just before renewed braking. |
| 2c5 12.812 s | 14.45 | 15.11 | 12.83 | More optimistic before later braking. |

The 55-second hindsight example from the prior analysis does not by itself
justify recovery at that instant: the driver has not yet supplied enough
observable easing. Once easing is established, current aLeadK itself has
already relaxed the baseline prediction. Extra tau recovery can therefore
arrive when baseline prediction is already adequate or optimistic.

Both candidates cancel the correction when their observed rebraking condition
is reached. This cannot undo a more optimistic prediction made before that
observable condition. Abrupt floor cancellation also changes the future
obstacle back toward baseline; its command-level jerk effect remains untested.

No tau changes occur at cap=0.10 in the late creep windows (2c3 64–73 s;
2c5 18–24, 57–64 and 100–108 s). Baseline tau has already recovered or the
lead is nearly stationary. Accordingly this replay cannot demonstrate that
tau recovery fixes the final crawl.

## Limits and decision

The two uploaded drives are not independent large-scale evidence. Adjacent
frames and horizons overlap, so frame counts are not independent trials.
Range reconstruction has bias, filtering and geometry error, and is not lead
ground truth. Published selected-lead samples differ from exact planner
fast-radar overlays and do not retain every production per-physical-track
state. Any integration would need shared radard/fast-overlay identity handling;
this prototype is intentionally not wired into either.

Do not select a cap merely because it reduces braking at one selected instant.
The current recorded-input results support keeping production unchanged. A
future attempt needs better evidence that easing will persist and separate
attention to low-speed request tracking/StopReq. Native solver and closed-loop
checks would be needed before claiming smoother ego braking or shorter crawl.

Private reproducibility archive: `.analysis/archive/2026-10-04/niro-tau-trial/`.
It includes both failed/sensitivity designs, scripts, summaries, and complete
prediction comparison tables. Original uploads remain on NAS.
