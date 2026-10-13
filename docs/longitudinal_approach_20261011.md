# Following deceleration and moving-front range disagreement

## October 13 correction: remove braking-onset distance memory

The user reported enlarged following gaps at gap 1 with Urgent follow and
stopping too far from the lead, then explicitly requested removing distance
memory instead of making its recovery response-dependent. The October 11
distance-memory mechanism described below is historical and is now removed.
There is no captured distance floor, margin ramp, stopped-pair hold, or forecast
of that memory at any response level. Current and future references use the
existing opening-gap TF policy and bounded comfort relief. The qualified radar
matching change and physical obstacle/acceleration/stop constraints remain.

Regression coverage runs the production MPC update with fixed shared-braking
inputs through standstill for both lead slots and all six response levels.
Starting without opening-gap headroom, the current and forecast margins stay
zero and the final reference returns to the configured 6 m stop distance.
This is reference/wiring verification, not native MPC or vehicle-response
validation. The reported physical stopping outcome still requires road logs.

Validation: 332 desktop tests pass across braking comfort, gap recovery, CPU
cost caching, cutout/MPC integration, acceleration preview and Wiki tooling.
The new 12-case regression fails against the parent implementation and passes
after removal. Changed Python files parse, the user-doc mapping check passes,
and generated Wiki pages have no structural validation errors. Native ACADOS
and vehicle tests were not run; broader native-dependent test collection is
unavailable in this Windows environment (missing capnp/setproctitle).

## Evidence and recent-change audit

The user requested corrections for a bus approach, shrinking following clearance
while both vehicles brake, and excessive braking with spare headway. SCC commands
are comparison evidence only; this change does not copy or blend them.
Original recordings, decoded messages, pictures and replay scripts remain private
under `.analysis/scratch/2026-10-11-ioniq5-approach` and the preceding G80 directory.

The two Ioniq recordings contain initData commit `64151b60ff`, not the upload
page's `c4702e42`. Both use front-radar mode 1. In the 1097 approach the effective
mode is Normal until 57.642 seconds, then Safe. Normal uses response level 5;
selected gap 1 is 0.45 seconds and the recorded applied TF during braking is
about 0.579 seconds before the later Safe increase. Timestamps below are segment
relative, aligned to recorded services, not the boot-time initData timestamp.

| Change | Finding on these recordings |
| --- | --- |
| Oct 1 SCC policy, `b3354534ac` | Mode 1 does not select SCC. Replaying the pre-change and recorded selectors produces identical lead ID/range/relative speed/acceleration/tau on all 1,200 and 1,199 frames. |
| Oct 2 native kernels, `bc0aa059cc` | Intended equivalent computation; prior native investigation establishes same-input parity. This investigation replays recorded tracks, not raw CAN through old/new kernels. |
| Oct 4 jerk cost 5 to 20, `7432ac9b5c` | A plausible contributor: reconstructed initial braking is weaker and later braking lasts longer. Not proof of the entire recurrence. |
| Oct 5 headroom hold band, `be1a9442f0` | Opening-gap memory is bypassed at response 5 during the main Normal approach; it cannot directly explain that window. |
| Oct 5 stopped-lead corroboration, `e6a6284437` | Pre-change and recorded selectors are identical on both recordings. |
| Oct 9/10 StopReq handling | All 700 examined CAN command frames at 46–60 seconds have StopReq=0; the stopped-state changes do not explain the moving approach. Toyota-only PID changes do not apply. |

An older ApproachDistanceController captured approach distance and relaxed it
gradually; it was removed in `8fa6313533` on September 1. This establishes the
history of the remembered feature, **not the cause of a recent-days recurrence**.
The current opening-gap TF reservoir does not reproduce that behavior at level 5.

The independent sequential planner reconstruction uses the same recorded ego and
lead measurements, but each J5/J20 variant retains its own plan/filter history.
All 580 solves converge. At 48/49/50 seconds the base target is respectively
-0.208/-0.452/-0.678 with J5 versus -0.115/-0.348/-0.596 m/s² with J20.
At 55/56 seconds J20 is stronger: -1.746/-1.449 versus -1.562/-1.286.
This is a full-convergence SciPy reconstruction, not native acados or a vehicle
counterfactual. The previously authorized J20 calibration is retained.

## Incident observations

In 1098 the bus-like front track is continuous from 33.822 seconds, initially at
203.65 m. The recorded selector uses it briefly at 40.250 seconds, then
continuously from 41.953. At 40.99 seconds radar reports 84.38 m and 31.56 km/h,
whereas vision reports 70.34 m and about 81.62 km/h; ego is about 89 km/h.
Normal range/path matching and a separate farther-radar veto delay selection.
The planner is already weakly braking, around -0.44 m/s², rather than strongly
accelerating. Later target braking reaches about -4.05 m/s². Gas override starts
at 43.932 seconds, so the log cannot establish an autonomous stopping outcome.
Pictures support the approach interpretation; exact radar/vision object identity
and range error do not have independent ground truth.

In 1097, measured distance falls from 19.35 m at 47 seconds to 13.80 m at 51,
9.40 m at 53 and 5.25 m at 59. The same front ID remains selected and no pedal
override occurs. The lead's filtered acceleration reaches -2.01 and -2.82 m/s²;
one-second measured speed trends near those dips are -1.93 and -2.42 m/s².
These events are not merely acceleration noise. An additional 0.2-second RC
filter delays braking thresholds by about 0.15–0.30 seconds in this example.
No such global filter is added.

At 47.997 seconds the planner command is -0.163 m/s² but positive speed feedback
P=0.213 produces control output +0.050, with nearby CAN requests +0.07/+0.09.
The distance-memory correction does not repair this earlier planner/PID timing.
The full desired distance also contains relative braking-distance terms; it is
incorrect to interpret every large target as extra TF.

G80's second event includes aLead about -5.18 m/s² and a one-second speed trend
about -3.48 m/s². Preview contributes only about -0.08 m/s² to a roughly -3.79
target. Extra opening headroom is at most about 2.83 m in that event, not tens of
metres. Sustained-deceleration prediction at small tau explains much of the
forecast, but the forecast also reaches negative danger reserve within 0.625 s.
A fixed acceleration cap or TF-only permission to delay braking is unsupported.

## Implementation

### Qualified moving-front range exception

Use stopped-equivalent distance `d + vLead²/(2*2.5)` to compare urgency; `d*v`
is not a stopping metric. Keep measured radar range and speed unchanged.
The exception requires measured front radar, established track state, 40–150 m
range, a moving lead above 4 m/s and relative speed below -1 m/s. Vision must
have probability >=0.6, acceleration >=-0.5 m/s², and be 2–20 m/s faster.
Require path/lateral agreement, low yaw, finite inputs and a bounded farther
range discrepancy: min(35 m, 40% of visual range, max(15 m, 2.5 visual xStd)).
The radar stopped-equivalent obstacle must be at least 5 m more restrictive.
Confirm the same ID for 0.25 s with range, lateral and velocity continuity.
Only that currently qualified track may bypass the distance-only veto. Existing
regular matches retain priority; observed braking vision is not overridden.

Bus replay advances continuous selection to 41.404 seconds, about 0.55 seconds
and 8 m earlier; it changes 11 of 1,199 frames. It does not recover the entire
early visibility interval. In 1097 it changes seven earlier approach frames near
33.8–35.1 seconds, none at 45–60. Reviewed images show a nearer SUV to the left
and a farther vehicle along the ego path; these images alone do not prove exact
sensor association. Broad false-association testing remains necessary.

### Distance memory while both vehicles brake

For an eligible stable radar lead, capture one distance per braking episode
after 0.3 seconds of track age, measured ego and lead acceleration below -0.2
m/s² and closing speed below -0.2 m/s. Bound capture by actual distance,
base TF distance plus 10 m and stop distance plus 2.5 seconds of ego speed.
Relax the captured distance toward the current base distance with an 8-second
time constant; departure evidence uses 2 seconds. Freeze a stopped pair to avoid
time-driven creep. Apply only the positive difference from the **full** existing
desired distance, with a 1 m/s reference-margin rate limit.

This applies at every response level, including 5. Combine it with opening-gap
memory using max, not addition. Track replacement, distance discontinuity,
invalid input or disengagement resets it. Predict through a copy of the same
distance state. No permanent user TF or stopping-distance setting is rewritten.

### Restricted comfort shortfall

Allow up to 0.4 seconds of comfort-reference TF shortfall, with a 0.7-second
reference floor, only for a stable moving lead with ample current and predicted
clearance, moderate closing speed and no current held positive headroom.
Require positive original danger reserve and physical clearance throughout the
first three seconds of the prediction. Entry ramps over 0.5 seconds; a hazard
withdraws the allowance immediately. Recheck the resulting trajectory and solve
once more without relief if it consumes that reserve; include both solves in
reported timing. A solver reset clears the allowance and prevents this retry.

These are comfort-reference conditions, **not a safety guarantee or brake cap**.
Physical obstacle arrays, original danger cost/constraint parameters, lead tau,
other leads, cruise/stop restrictions and acceleration limits remain intact.
The existing MPC danger constraints are soft penalties; unchanged parameters do
not mathematically guarantee collision avoidance. Neither SCC acceleration nor
the lead's instantaneous acceleration is used as a maximum permitted ego brake.

## Verification and limitations

- 682 focused tests pass for the helper, MPC reference integration, cutout
  compatibility and radar selection. Tests include invalid predictions, stopped
  hold/departure, lead replacement and withdrawal/re-solve of unsafe trial slack.
- Another 345 preview/fast-radar/SCC/stopped-lead tests and 36 route-server tests
  pass; four platform/integration-dependent server tests skip on Windows.
- 1,936 paired recorded-input planner steps (3,872 SciPy solves) all converge;
  physical constraint parameter arrays match exactly before/after on every step.
- In 1097 reconstructed displayed targets at 55/57/59 seconds increase from
  9.14/7.94/5.95 to 10.94/11.74/11.75 m. These are reference distances, **not
  measured post-change clearance**. Peak braking is essentially unchanged.
- G80 reconstructed peaks remain -3.903 and -2.010 m/s² in the examined 118/145
  windows. The severe peak gets no relief. This does not fix the reported G80
  harsh-braking peak and is not presented as such.
- Python AST MPC harness and independent full-convergence solver are used on
  Windows. Native acados timing/convergence, actual brake response, missed/false
  sensor associations and closed-loop minimum clearance remain unvalidated.
  Fast-track overlay reconstruction retains published tau, so it is not an
  exact native replay. No vehicle installation is included.

Retained work: early planner/PID feedback timing and native closed-loop tests
are still needed before declaring the shrinking-gap complaint resolved.
