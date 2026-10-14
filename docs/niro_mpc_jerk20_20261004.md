# Niro: J_EGO_COST 5 to 20

## Subsequent user-authorized trial

After reviewing this analysis, the user explicitly requested applying J_EGO_COST
20 for several days of driving. The ACC constant is now 20; obstacle cost remains
5 and the other costs, lead prediction, limits and stopping logic are unchanged.
Existing personality/response multipliers still apply. This is the shared ACC
calibration, not a Niro-only switch; blended mode keeps its separate jerk weight.
The analysis-only statements below describe the preceding experiment. Promotion
does not convert that experiment into native or physical vehicle validation.
Implementation validation: 261 focused tests pass; recording-solver checks confirm
ACC effective weights 10/14/20, retained response scaling and blended jerk weight 1.

## Result and scope

The user requested an offline trial of J_EGO_COST=20 after the obstacle-cost
comparison. This experiment changes the base jerk coefficient from 5 to 20;
the recorded aggressive-personality factor 0.5 makes the effective weight
2.5 -> 10. A value of 20 is a cost weight, not a jerk limit of 20 m/s³.

In the independent planner reconstruction, jerk cost 20 reduces maximum base
target deceleration by about 4–10% over four stop approaches. The response is
larger than obstacle cost 5 -> 3 alone. However, some later braking persists
longer, the fixed-input stop predicate occurs about 0.4–0.5 seconds later, and
predicted gaps do not improve consistently. This supports further study of
initial braking comfort, not a claim that the final crawl is solved.

No production constants, vehicle settings, stopping logic or NAS services changed.

## Comparison matrix

Each case uses the same recorded input sequence and owns its subsequent previous
acceleration trajectory, desired acceleration and desired-speed filter state.
Lead prediction/tau, TF, stopping distance, previous-acceleration cost and modeled
limits/penalties stay the same. Each window starts from recorded planner state.

| Case | X_EGO_OBSTACLE_COST | J_EGO_COST | Effective jerk weight |
| --- | ---: | ---: | ---: |
| Baseline | 5 | 5 | 2.5 |
| Previous obstacle-only candidate | 3 | 5 | 2.5 |
| Requested jerk-only candidate | 5 | 20 | 10 |
| Combined candidate | 3 | 20 | 10 |

A total of 1,259 timestamps across four approaches yields 5,036 converged solves.
The baseline and previous obstacle-only candidate reproduce the prior experiment
exactly (maximum target difference 0). Constant-jerk integration, interpolation
inversion and analytic-Jacobian checks pass. Twenty-six additional J20 comparisons
from two independent initial guesses agree in objective within 1.94e-11.

## Peak braking

These are reconstructed base MPC target accelerations, in m/s², before preview
correction, PID, CAN and vehicle response. Negative values indicate braking.

| Approach | X5/J5 | X3/J5 | X5/J20 | X3/J20 | J20-only peak magnitude reduction |
| --- | ---: | ---: | ---: | ---: | ---: |
| 2c3, 49–70 s | -3.0417 | -3.0213 | -2.7417 | -2.7141 | 9.86% |
| 2c5 first, 10–23 s | -3.3608 | -3.3527 | -3.2110 | -3.2051 | 4.46% |
| 2c5 second, 49–62 s | -3.1781 | -3.1704 | -3.0427 | -3.0406 | 4.26% |
| 2c5 third, 90–106 s | -2.7615 | -2.7501 | -2.6347 | -2.6296 | 4.59% |

The additional benefit of obstacle cost 3 with J20 is small: another 0.002–0.028
m/s² at the respective peak. It is not evidence that both coefficients should be
changed together. J20-only peak times occur 0.10–0.21 seconds earlier in this
reconstruction, so the result is not simply a later peak of the same depth.

## Jerk and release shape

Maximum absolute initial planned jerk, in m/s³:

| Approach | X5/J5 | X5/J20 | Reduction |
| --- | ---: | ---: | ---: |
| 2c3 | 3.231 | 2.466 | 23.68% |
| 2c5 first | 2.857 | 2.744 | 3.95% |
| 2c5 second | 2.557 | 2.517 | 1.55% |
| 2c5 third | 2.125 | 2.103 | 1.03% |

These are the first jerk values from successive optimized plans, not measured
vehicle jerk or the finite-difference derivative of the downstream command.
The 2c3 initial dip changes visibly; the other three initial jerk peaks change
less despite a fourfold cost increase. Other objective terms and predicted
proximity still influence the solution.

The jerk penalty applies both to applying braking and releasing it. Relative to
baseline, J20-only has later intervals with 0.09–0.17 m/s² stronger braking; the
largest is 2c3 at t=58.047 s. Raising jerk cost therefore does not uniformly weaken
braking or ensure prompt recovery when the lead eases off.

## Terminal behavior and range cautions

Using the same stopping predicate on each candidate's trajectory gives:

| Approach | Baseline predicate time | J20-only time | Difference |
| --- | ---: | ---: | ---: |
| 2c3 | 69.195 s | 69.648 s | +0.453 s |
| 2c5 first | 21.528 s | 21.918 s | +0.390 s |
| 2c5 second | 60.338 s | 60.789 s | +0.450 s |
| 2c5 third | 104.100 s | 104.589 s | +0.489 s |

These times are compared within the reconstruction, not against exact logged
StopReq times. They do not mean actual stopping is delayed by those amounts:
measured vehicle speed/range remain fixed, and candidate controller/vehicle
responses are not simulated. They provide no support for claiming reduced crawl.

Minimum predicted physical lead gap over all nodes/updates is respectively
3.886/3.990/3.981/3.292 m in baseline and 3.868/3.990/3.851/3.292 m with J20 only.
The combined X3/J20 case reaches 3.866/3.895/3.784/3.292 m. Some minima are set by
the shared recorded present range, so unchanged minima are especially unsuitable
as evidence of unchanged real safety. These are neither new final stopping
clearances nor a closed-loop clearance guarantee. Danger-margin changes also
differ across events; there is no uniform improvement.

## What is and is not validated

This uses the independent nonlinear OCP and fixed-measurement planner-state
rollout documented in [the obstacle-cost experiment](niro_mpc_obstacle_cost_20261004.md).
It is not native acados SQP_RTI and is not a vehicle plant simulation. Published
radar inputs substitute for fast radar; optional gap-reference/obstacle adjustments,
dynamic bounds, mode transitions and downstream controls are not fully reproduced.
The previous report lists these exclusions explicitly.

Baseline mean absolute target error against the log remains 0.013–0.021 m/s²;
maximum errors are 0.139–0.222 m/s². Candidate peak differences are larger than mean
baseline error but not consistently larger than worst local error. The numerical
trend is worth investigating; the percentages are not vehicle-validated benefits.
No independent holdout routes were evaluated.

The result supports retaining J20 as an analysis candidate for reducing the
initial deceleration peak. It does not support production promotion or a claim
that J20 solves both initial braking and the terminal crawl. A lower intermediate
weight or separate application/release treatment could be compared later; neither
was tested or selected here.

## Reproduction

Private archive: `.analysis/archive/2026-10-04/niro-jerk20/`.
It contains extraction, solver, math/multistart checks, all candidate trajectories,
summary metrics and `jerk20_comparison.png`. Run scripts from repository root.
SciPy 1.18.1, NumPy 2.5.3 and Matplotlib 3.11.2 were used in a temporary analysis
dependency directory. Decoded logs and dependencies are reproducible and removed;
the original rlogs remain on NAS. No raw logs/settings are tracked in Git.
