# ACC jerk-cost driving trial

On 2026-10-04 the user explicitly selected `J_EGO_COST = 20.0` for a multi-day
driving trial, following offline comparisons against the previous value 5.0.
The change applies to the shared ACC MPC, not a vehicle-specific override.

Existing personality and lead-response multipliers remain active. With no
lead-response reduction, personality factors 0.5, 0.7 and 1.0 produce effective
jerk weights 10, 14 and 20. The coefficient is a cost weight, not a jerk limit.
Blended MPC retains its separately specified jerk weight of 1.0.

Obstacle cost stays 5, acceleration-change cost stays 200, and acceleration
magnitude cost stays zero. Lead prediction/tau, TF, stopping distance, danger
penalties, actuator limits and stopping logic are unchanged.

The preceding independent fixed-measurement planner reconstruction suggested
lower initial peak deceleration, but also slower release in some intervals and
later stopping predicates. It was not native acados or vehicle-response validation;
physical comfort, clearance and terminal-crawl improvement remain unvalidated.

Validation: 261 focused gap-recovery, cutout/MPC integration, preview and lead-tau
tests pass on Windows with the root device-dependent conftest excluded. Recording
solver checks confirm effective ACC weights 10/14/20, retained response scaling,
and blended weight 1. The two AST-based test harnesses now load J_EGO_COST from
production source rather than retaining a stale test-local value of 5.

This is a fixed calibration change; no Params setting, menu or user workflow was
added. A comparison rollback of this trial consists of restoring J_EGO_COST to 5.0
while preserving all other longitudinal tuning.
