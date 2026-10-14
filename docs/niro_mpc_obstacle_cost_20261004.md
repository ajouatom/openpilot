# Niro MPC obstacle cost: 5 versus 3

## Finding

Reducing X_EGO_OBSTACLE_COST from 5 to 3 is a direct way to relax desired-gap
tracking relative to the other costs. It is a reasonable sensitivity experiment.
The completed independent numerical reconstruction, however, finds only small
peak-deceleration changes on these four approaches, not a substantial redistribution
of braking. Some release intervals brake more and some peak jerks increase.
This does not establish that 3 improves the vehicle or that all obstacle-cost
tuning is ineffective. No production code or settings changed.

The requested target is to allow a temporary shortfall from the comfort gap,
spread deceleration over the approach and finish at an appropriate stopped gap.
Conditional stopping remains unselected; this experiment changes only the obstacle
cost and holds lead prediction, TF and the other modeled cost weights fixed.

## What the coefficient changes

The ACC distance residual is

`e = (x_obstacle - x_ego - [v²/(2*comfort_brake) + TF*v + stop_distance])/(v+10)`.

Its squared-error coefficient falls by 40%. For a fixed error, the associated
cost and gradient fall by 40%; deceleration need not fall by 40%. Relative to this
term, unchanged jerk and previous-plan costs become 5/3 times as influential.
The cost is symmetric: both too-close and too-far comfort-gap deviations become
less expensive. It does not exclusively permit closer approach. The shared
obstacle is also the minimum across leads, cruise and traffic-stop obstacles, so
a global change is not confined to following a braking lead.

The additional danger residual uses 0.8 times desired stopping-equivalent distance,
with a one-sided weight of 100. The factor does not mean 80% of raw radar range.
This is a finite soft constraint, not a guaranteed collision-clearance bound.
Its weight and boundary remain unchanged in the comparison. At a fixed speed
inside this penalty region, local position-error curvature from the two terms
changes in proportion to 105 -> 103 (about 1.9%), rather than 5 -> 3. This is a
local curvature observation, not the complete nonlinear optimizer's response.

The reconstructed forecasts enter that danger region during several strong-braking
snapshots. This is a model forecast, not evidence of a real collision risk or
driver danger at the time. It helps explain why changing the comfort-gap weight
alone has limited influence in this reconstruction. Previous-plan acceleration
cost 200 also resists an immediate change of trajectory.

## Independent numerical method and important exclusions

The native acados Python bindings remain unavailable. An independent SciPy
least-squares solver reconstructs the 12-control/13-node, 10-second OCP:

- Production lead extrapolation is extracted and executed unchanged, including
  right-endpoint discrete velocity/position integration and stopped equivalence.
- Ego dynamics integrate a constant jerk exactly over each nonuniform interval.
- Distance, previous-plan acceleration and jerk residuals use the production
  formulas and stage-time scaling. Acceleration magnitude weight stays zero.
- One-sided speed, acceleration and danger penalties retain weights 1e6, 1e6,
  and 100; acceleration limits are -4 and +2.5 m/s² in this reconstruction.
- Recorded TF is 0.85 / 0.90, stopping distance 6 m and comfort brake 2.4 m/s².
  Recorded aggressive personality corresponds to jerk factor 0.5, hence weight
  2.5 in the examined braking intervals. Recorded aChangeCost is used.
- The first seven MPC nodes through 2.5 seconds can be reconstructed by inverting
  the interpolation used to publish the 17 plan nodes. That covers all nodes
  carrying a nonzero previous-acceleration weight, which fades to zero at 2 s.

Differences from the actual planner remain:

- Published radarState is used, not the planner's fast liveTracks overlay.
- Physical leadOne/leadTwo obstacles are included. Cruise/traffic obstacles,
  gap-recovery reference offsets and other optional obstacle adjustments are
  omitted. The current displayed-reference discrepancy reaches 9.96 m on the
  sampled snapshots; despite good target agreement, input identity is not exact.
- The +2.5 acceleration bound is a reconstruction assumption rather than the
  logged dynamic cruise/turn bound. Positive-response jerk-factor changes,
  reset/standstill transitions and feature state are not fully replayed.
- This solver converges its nonlinear problem rather than executing native
  warm-started SQP_RTI. Native timing and local-iteration behavior differ.
- Output is base MPC target acceleration, before anticipatory output correction,
  PID, CAN actuation and vehicle response. It is not actuator acceleration.

Analytic-Jacobian checks against finite differences, constant-jerk integration
checks and plan-node reconstruction checks pass. These verify numerical mechanics,
not the omitted vehicle/planner behavior.

## Snapshot comparison

Thirty-nine snapshots span four braking/creep approaches. Each weight uses the
same recorded initial state and previous plan. All 78 selected solves converge;
two independent starts agree in objective to within 5.6e-11.

Weight-5 base-target reconstruction versus the recorded aTargetBase has mean
absolute error 0.00648 m/s² and maximum error 0.06703 m/s². Weight 3 minus weight 5
changes the immediate base target between -0.00572 and +0.00647 m/s². These are
one-solve sensitivities, not the accumulated effect of changing the coefficient.

## Carrying each candidate's plan forward

To avoid incorrectly concluding from a fixed previous plan, each candidate also
gets its own evolving previous-acceleration trajectory, initial acceleration and
desired-speed filter. The recorded 0.05-second planner update equations are used;
measured ego speed and radar inputs remain fixed to the log. Each window begins
with recorded planner state, then weight 5 or 3 is applied for that window.
There are 1,259 paired updates, 2,518 converged solves in total.

| Approach / analyzed time | Recorded minimum base target | Reconstructed weight 5 | Reconstructed weight 3 | Weight-5 target MAE / maximum error |
| --- | ---: | ---: | ---: | ---: |
| 2c3, 49–70 s | -3.0399 | -3.0417 | -3.0213 | 0.0131 / 0.2225 |
| 2c5 first, 10–23 s | -3.3608 | -3.3608 | -3.3527 | 0.0157 / 0.1624 |
| 2c5 second, 49–62 s | -3.2006 | -3.1781 | -3.1704 | 0.0214 / 0.1763 |
| 2c5 third, 90–106 s | -2.7844 | -2.7615 | -2.7501 | 0.0169 / 0.1392 |

All acceleration values are m/s². The small peak changes are comparable to or
smaller than reconstruction error, so they are not a reliable estimate of actual
vehicle improvement. They do show that this reconstruction provides no strong
support for a large braking reduction from this single weight change.

Maximum absolute planned initial jerk for weight 5 -> 3, m/s³:

- 2c3: 3.231 -> 3.302.
- 2c5 first: 2.857 -> 2.883.
- 2c5 second: 2.557 -> 2.556.
- 2c5 third: 2.125 -> 2.126.

In 2c3 at approximately 57.35 s, weight 3 requests about 0.093 m/s² more braking
than weight 5. Relaxed distance tracking does not guarantee a smoother release.
These are reconstructed planner outputs, not logged alternate vehicle jerk.

This is **fixed-measurement planner-state rollout**, not a closed-loop vehicle
simulation. Alternate braking does not change the recorded relative range,
measured velocity or lead observations. Final clearance, creep duration, stopping
time, stopping-mode transitions and comfort improvement therefore remain unproved.
Changing the weight from drive startup can also produce a different initial state
than these windows' common recorded initialization.

## Interpretation

The user's proposed tuning direction is correct for reducing comfort-gap tracking
pressure. On these logs it is not supported as the sole explanation or a demonstrated
fix for front-loaded braking. In the reconstruction, unchanged danger penalties
and previous-plan consistency remain influential, and reducing distance weight
can also weaken gap recovery. Do not lower danger cost merely to force the desired
result; whether conservative lead forecasts create unnecessary predicted proximity
is a separate hypothesis requiring baseline-matching native analysis.

The appropriate status is a completed, limited sensitivity analysis with no
production promotion. Preserve the 5 baseline. A native paired replay and a vehicle
response model or controlled validation would be needed to establish a useful
change in the complete stopping profile.

Private evidence and scripts:
`.analysis/archive/2026-10-04/niro-mpc-weight/`.
See also [earlier alternatives](niro_stop_alternatives_20261004.md) and
[tau screening](niro_tau_recovery_trial_20261004.md).
