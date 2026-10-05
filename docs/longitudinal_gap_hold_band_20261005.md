# Distance range for additional following-headroom hold

On 2026-10-05 the user approved limiting the distance range in which opening-gap
and low-lead-speed conditions can retain extra following headroom. This applies
to the existing response-level 0–4 comfort mechanism in ACC. Effective level 5
still bypasses it. No new setting or response-level change is introduced.

Define D as the MPC's current ego speed times base TF plus its configured stop
distance. Extra TF and braking-distance-equivalence terms are excluded from D.
The selected TF already includes existing speed and driving-mode adjustments.
Use the corresponding predicted ego speed and anchored lead distance in the
future rollout, with the same configured stop distance.

- At or below 1.2D, retain the previous capture/hold/recovery rules.
- Between 1.2D and 1.5D, use a smoothstep distance weight w from 0 to 1.
- At or above 1.5D, stop replenishing/holding and release at the existing
  response-level rate, including for a stopped or still-opening lead.

Within the transition band, scale the opening envelope and its maximum rise
rate by 1-w. Blend the lead-speed recovery strength toward 1 by w. An envelope
below the reservoir is approached through the existing two filter stages;
an envelope above it can still replenish at the reduced bounded rate. This
limits replenishment as well as easing the recovery restriction. It does not
erase a state or directly add acceleration. Smoothstep has zero slope at both
boundaries, avoiding a binary distance switch or a separate hysteresis latch.

The two-stage solution with fixed lower envelope q is
`r_next=q+(r-q)*exp(-k*dt)` and
`e_next=q+(e-q+k*dt*(r-q))*exp(-k*dt)`. At full distance release q=0;
at zero distance weight the original dynamics remain. As before, output can
briefly rise if its reservoir was already larger when recovery began.
New-lead acquisition still initializes the existing candidate with its entry
ramp; loss, invalid inputs, level changes and eligibility resets stay unchanged.

Physical lead obstacles, base TF, stop distance, danger penalties, acceleration
limits, stopping predicates and the J20 trial are unchanged. The current and
future comfort references both use this rule. Stop distance is passed explicitly
from MPC to live state and prediction; invalid stop-distance inputs reset state.

299 focused gap, preview, cutout/MPC integration and lead-tau tests pass on
Windows with the device-dependent root conftest excluded. Validation covers
unchanged dynamics within the band, smooth boundaries,
progressively weaker hold, full recovery across stopped/slow/moving leads and
opening/steady/closing gaps, no far-gap replenishment, configured stop-distance
propagation, and live/prediction agreement without prediction mutating live
state. Recording-solver integration checks retain physical obstacles/constraints
and effective-level-5 parity. Such checks do not solve the native MPC.

A private recorded-input reconstruction matches its previous baseline exactly
for 1,334 frames. The final rule reduces the additional reference at one stop
approach from 1.916 m to 0.142 m at the same recorded input. This is neither a
changed physical following distance nor a measured reduction in stopping time.
Full closed-loop vehicle response, final clearance and comfort remain unvalidated.
Private input/replay artifacts are excluded from commits.

An additional independent full-convergence OCP with an illustrative 0.25-second
first-order vehicle lag compares three synthetic scenarios. A brake/crawl/stop
case has nearly unchanged peak deceleration and stop time, with final gap about
0.032 m smaller. Renewed lead braking stops the ego 0.6 s earlier, with final gap
about 0.064 m smaller and maximum late command change per 0.1 s increasing from
0.078 to 0.123 m/s^2. Steady 1 m/s lead following closes the initial excess sooner,
but minimum gap decreases from 7.03 to 6.61 m before returning to 6.91 m; minimum
ego acceleration changes from -0.068 to -0.129 m/s^2. Thus faster recovery can
increase subsequent deceleration and slightly undershoot the preferred gap.

These synthetic results are sensitivity evidence only: the independent solver
is not native SQP-RTI, the plant is not identified from a vehicle, and the model
omits ECU stopping modes, sensing noise and actual actuator behavior. Unchanged
danger constraints do not prove a safe physical stopping distance. The change
retains the existing full recovery rate, rather than adding an acceleration
boost or increasing that rate beyond the selected response level. User-facing
documentation retains the existing limits; physical validation remains necessary.
