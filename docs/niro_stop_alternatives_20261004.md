# Niro follow-stop: alternatives after tau-recovery screening

Completed follow-up: [obstacle cost 5 versus 3](niro_mpc_obstacle_cost_20261004.md)
using independent OCP reconstruction and fixed-measurement planner-state rollout.
That report supersedes the earlier statement below that no cost sweep has run;
native acados and vehicle-response validation remain unperformed.

## Follow-up direction from the user

The subsequent clarification is more specific: permit a temporary shortfall from
the desired following gap, distribute braking over the approach, and finish at an
appropriate stopped gap instead of concentrating braking early and crawling later.
This prioritizes the tradeoff between distance tracking, acceleration magnitude
and jerk over a release-only change to previous-acceleration cost.

Source inspection confirms ACC weights include obstacle distance=5,
acceleration magnitude=0, jerk=5 times mode/response factors, and previous-plan
acceleration deviation=200 when the normal previous-acceleration constraint is
active. Zero acceleration magnitude cost does not mean deceleration is unconstrained:
acceleration limits, distance costs, jerk and previous-plan costs still act.
However neither jerk nor previous-plan deviation directly penalizes a large
sustained deceleration. A nonzero acceleration-magnitude weight is therefore a
relevant existing MPC tuning axis, alongside distance and jerk weights. Its
quadratic form also penalizes positive acceleration and can affect departures.

The intended comparison should first keep lead inputs, TF, stopping distance,
acceleration limits and danger-zone penalty unchanged, and sweep those comfort
weights separately and together. The danger-zone restriction is a soft constraint
with finite penalty in this implementation, not a guaranteed hard clearance bound;
preserving its settings still requires checking actual predicted minimum gaps.
If static weights cannot balance approach comfort with settled gap/departure,
bounded near-horizon distance tolerance or time-varying weights are later design
options. They are not existing menu settings or a completed validation. Lead
prediction variants should follow this isolated baseline comparison, so their
effects are not conflated with the comfort objective. No solver sweep has run.

The user prefers investigating lead prediction and MPC parameter tuning before
adding a conditional stopping path. The research order below is the earlier
proposal, not an approved implementation plan. No conditional handoff is selected.
The next priority is a baseline-matching MPC comparison: previous-acceleration
cost, jerk cost and obstacle-distance cost separately, then justified lead-model
combinations. The failed tau-only forecast candidates do not establish that MPC
tuning cannot improve the trajectory. Recorded A_CHANGE_COST=200 penalizes deviation
from the previous predicted acceleration trajectory; it is not a command to hold
measured deceleration constant. Lowering this cost and lowering jerk cost are
different experiments. Lower jerk cost can increase acceleration changes and must
not be described as inherently smoother. Native MPC comparisons remain unperformed.

## Outcome

The most directly supported next experiment is a bounded terminal-stop handoff,
not a general reduction of lead braking prediction. A recorded-plan screening
advances the stopping request in all four investigated approaches while avoiding
two restart delays introduced by a global stopping-speed increase. This is a
candidate for further validation, **not demonstrated smoother vehicle motion**.
No production code, settings, vehicle or NAS service changed.

Previous evidence: [2c3](niro_2c3_stop_tail_20261004.md),
[2c5](niro_2c5_follow_stop_20261004.md),
[tau candidates](niro_tau_recovery_trial_20261004.md).

## Reproduction and baseline check

Full cereal rlogs from the same two routes/four segments were decoded again.
Every scored decision uses the current plan and preceding carState/radarState/
carControl, without future input. The production `get_accel_from_plan` function
was extracted and executed with the recorded 17-node speed/acceleration arrays,
0.25 s action time for 2c3 and 0.35 s for 2c5. Its 0.10 m/s stopping predicate
matches all 4,495 aligned recorded decisions (2,097 + 2,398); the first 2c5 plan
is excluded because a preceding carControl is unavailable.

The relevant drive_helpers, longcontrol, longitudinal MPC, preview and Hyundai
controller source files have no diff against reported revision 39157c1c. Device
initData nevertheless reports dirty=true, so unknown device edits remain possible.

## 1. Global stopping-speed increase has a restart tradeoff

Current VEgoStopping is 0.10 m/s in both drives; catalog default is 0.50 m/s.
The predicate checks planned speed at action time and action time plus one second,
not measured ego speed. Increasing it advances the stopping request, but also
keeps that request true longer as a stopped car's plan begins to accelerate.

Two observed departures in 2c5 demonstrate the latter on unchanged inputs:

| Threshold | Extra true interval after original stop request releases | Lead speed during interval |
| --- | --- | --- |
| 0.30 m/s | 36.869–37.429 s; 70.688–70.938 s | up to 0.70 / 0.67 m/s |
| 0.50 m/s | 36.869–37.828 s; 70.688–71.538 s | up to 0.92 / 1.10 m/s |

At 36.927 s the recorded controller is already in PID with a positive request,
whereas both raised-threshold predicates still request stopping. This establishes
an undesired decision difference; actual extra departure delay is not simulated.
It argues against treating a global threshold increase as a complete solution.

## 2. Confirmed stationary-lead terminal handoff

An analysis-only candidate retains the original predicate and ORs in the existing
0.50 m/s predicate only when all of these conditions hold:

- Valid fresh plan/car/radar inputs, valid CAN, active longitudinal control,
  no gas or brake input; radar age <=150 ms and carState age <=50 ms.
- Same measured radar lead, nonnegative track identity, |vLead| <0.30 m/s for
  0.50 continuous seconds. Identity changes, invalid data or radar gaps >150 ms
  reset confirmation. Confirmation advances on radar messages only.
- Ego speed below 5 km/h.
- Range <= 6 m + vEgo²/(2 × 0.50 m/s²) + 0.30 s × vEgo.

The 6 m term is this vehicle's recorded stop-distance setting. The deceleration
and delay terms are explicit screening assumptions, not a calibrated vehicle
stopping model. Constants were fixed before this screening run, but this is the
same development dataset, not independent validation. Only leadOne is modeled;
production integration would need selected-source, competing-lead and mode
handling, full finite-value checks, release behavior and controller validation.

| Approach | Original request time | Candidate time | Advance | Range at original / candidate request |
| --- | ---: | ---: | ---: | ---: |
| 2c3 stop | 69.146 s | 65.596 s | 3.55 s | 4.32 / 6.62 m |
| 2c5 first | 21.467 s | 17.868 s | 3.60 s | 4.52 / 6.63 m |
| 2c5 second | 60.280 s | 56.588 s | 3.69 s | 4.42 / 6.54 m |
| 2c5 third | 104.038 s | 101.340 s | 2.70 s | 4.02 / 6.14 m |

Across all aligned plans, the candidate adds requests only in these four approach
intervals. Neither of the two departure intervals above receives an additional
request. The second stop illustrates the confirmation benefit: unrestricted 0.50
requests stopping at 55.779 s while the lead is still moving at 0.58 m/s; the
guard waits until 56.588 s. There are no false-positive additional episodes in
these logs, but their absence in four stops is not evidence of general safety.

At each candidate onset, recorded aEgo is greater than configured StoppingAccel,
so the existing PID-to-stopping acceleration gate would not block entry with the
same state. Full controller state evolution was not replayed. Earlier StopReq
changes the vehicle trajectory; subsequent recorded speed/range/plan cease to be
counterfactual ground truth. Request advance is therefore **not** time saved,
and the table's ranges are not predicted final stopping clearances.

Existing stopping control retains a stronger prior negative command rather than
unwinding it to StoppingAccel. Earlier handoff can therefore also brake more firmly
or stop farther away. A smooth bounded transition and prompt cancellation on lead
departure must be evaluated, not assumed from the earlier trigger alone.

## 3. Low-speed command tracking is a separate suspect

Median recorded values before and after the original request switch are below.
Before = switch−1.5 to switch−0.2 s; after = switch+0.35 to switch+0.8 s.
Requests are carControl accelerations; prior reports independently decoded their
SCC12 transmission and StopReq.

| Approach | Request before / after, m/s² | Actual aEgo before / after, m/s² |
| --- | ---: | ---: |
| 2c3 | −0.552 / −0.997 | −0.066 / −0.414 |
| 2c5 first | −0.443 / −0.502 | −0.044 / −0.490 |
| 2c5 second | −0.417 / −0.505 | −0.038 / −0.483 |
| 2c5 third | −0.696 / −0.652 | −0.142 / −0.441 |

The third event has stronger measured deceleration after switching despite a
slightly weaker requested deceleration. Together with the first two, this supports
investigating stopping mode/low-speed ECU response. It does not isolate StopReq as
the cause: speed, brake state, jerk request, grade and other conditions vary.

This non-CAN-FD path sends comfort bands derived from requested acceleration.
The CAN signal names alone do not establish how the ECU applies them. Reducing
bands, adding integral action or low-speed brake compensation cannot be validated
with a fixed-input replay; all can produce stronger braking or a stop jerk. They
remain lower-priority physical characterization candidates, not proposed blanket
changes to Hyundai control.

## 4. Release-only MPC response merits study, but is not proved necessary

The existing lead-response boost requires positive lead acceleration above its
deadband. Merely easing negative acceleration does not activate it. Recorded
aChangeCost stays 200 at the 2c3 easing samples. An alternative research design
would adjust the penalty on releasing previous braking after sustained measured
easing, with a stable lead, adequate gap and bounded closing speed, while retaining
the original lead prediction and physical constraints. A direction-specific cost
would avoid globally lowering jerk cost on both brake application and release.

However the recorded target already releases: at 55, 56.5 and 58 s its values are
−2.03, −1.52 and −0.77 m/s²; actual aEgo is −2.14, −1.79 and −1.03. At 56.5 s ego
is still 29.3 km/h versus lead 24.5 km/h, so faster release cannot be justified by
lead easing alone. The lead also brakes more strongly again by 58 s. This is a
candidate to compare in MPC replay, not a conclusion that cost 200 is excessive.

The separate anticipatory output correction is zero at these three samples
(aTarget equals aTargetBase); changing its release filter does not explain or fix
that observed interval directly. It is also absent from the representative final
crawl samples. Previously analyzed opening-gap recovery is not established as the
cause of these tails either.

Native longitudinal acados bindings remain unavailable in the local environment.
No alternate solver, vehicle plant or closed-loop replay was used. Consequently
release-only cost changes have not been scored for command jerk, clearance or
vehicle motion. The completed experiment here is the stopping-decision screening.

## Recommended research order

1. Validate confirmed stationary-lead handoff and separate stop-entry/departure
   criteria, including continuous transitions, restarts, moving crawl, lead loss
   and a second lead. Do not raise the global threshold blindly.
2. Characterize low-speed vehicle response to the existing request and stopping
   flag before introducing stronger braking compensation or changing CAN bands.
3. Compare release-only MPC cost variants using a baseline-matching native replay;
   retain the original prediction and braking constraints. Rebraking cases are
   necessary counterexamples, not optional comfort tests.

Private reproduction evidence is under
`.analysis/archive/2026-10-04/niro-stop-research/`. Source rlogs stay on NAS;
decoded caches are reproducible and removed after analysis.
