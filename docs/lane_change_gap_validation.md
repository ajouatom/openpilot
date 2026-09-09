# Lane-change following-distance control

The lane-change percentage no longer multiplies common TF or jerk cost.
Baseline TF and dynamic TF have separate ramp histories, preventing repeated
reduction in consecutive planner cycles. Lane-change starting/finishing also
blocks the lead-acceleration response override and ordinary dynamic TF.

## Departure and destination constraints

`radar_motion/lane_change_gap.py` is shared by the vehicle planner and the NAS
radar replay. Radar yRel is left-positive; model and device-pose y are
right-positive. Integration starts in a fixed frame at lane-change entry.
Measured speed and valid livePose yaw establish actual lateral displacement.
The reconstructed old lead must stay laterally stable in that frame. This
prevents a rotating radar azimuth or a shifted model path alone from granting
departure credit.

The model must predict sustained body clearance, with a 2.5 m combined width
and margin plus an expanding 0.2 m/s uncertainty envelope. Its prediction may
not advance clearance faster than measured lateral motion. Track continuity,
actual movement and destination tracks must be confirmed. Missing pose,
invalid samples, large turns, changed primary identity, a path returning to
the old lane, blindspot warnings and unconfirmed destination traffic revoke
credit. An apparently empty destination is not treated as verified free space.
The confidence field is a confirmation ramp, not a calibrated probability.

Before allowing credit, a 50 ms grid checks the transition for three seconds,
using the allowed maximum ego acceleration and at least 1 m/s² target braking
(or a stronger measured deceleration). The old lead must retain a stopping
buffer until clearance plus 0.35 s. Every destination front target must retain
the full TF and braking-distance margin throughout the transition. Close side
or rear targets and blindspot warnings deny extra credit; existing lateral
permission checks remain responsible for the lane-change decision.

MPC keeps its unmodified common TF. It receives bounded credit only on old
lead obstacle samples after clearance plus 0.35 s, ramping over 0.5 s. The cap
is the smallest of 20% of base TF, 0.25 s and 4 m. Destination front vehicles
compete independently with full normal spacing from the start of the maneuver.
LeadTwo, traffic-stop and cruise obstacles remain in the minimum. Old CUT-OUT
and lane-change credits never stack. Original radarState and FCW trajectories
are not rewritten. The implementation does not change the MPC solver ABI.

## Settings

`DynamicTFollowLC=100` remains the default and disables extra departure credit;
destination constraints remain enabled. Invalid/zero values disable credit.
Values below 80 have the same cap as 80, so legacy aggressive settings cannot
restore global TF reduction. Credit is disabled for TF below 0.8 s and speeds
outside 5–35 m/s. These bounds are conservative implementation choices, not
experimentally established optimal driver preferences.

At 15 m/s and base TF 1.3 s, fully confirmed, otherwise eligible forecasts have
these maximum credits (actual credit may be zero):

| Setting | Time equivalent | Distance |
|---|---:|---:|
| 100 | 0 s | 0 m |
| 95 | 0.065 s | 0.975 m |
| 90 | 0.130 s | 1.950 m |
| 80 or below | 0.250 s | 3.750 m |

Use 100 for the initial road baseline. A single-step decrease is an optional
later comfort adjustment after comparing recorded maneuvers. Synthetic tests
cannot establish that 90 or another value is best for a particular driver.

## Verification and limits

Focused tests cover mirrored maneuvers, path-only movement, yaw-only azimuth,
aborted/re-entering paths, sensor loss, changed tracks, stopping/closing leads,
multiple destination vehicles, invalid settings, TF recurrence and production
MPC parameter assembly with a recording solver. The NAS adapter recalculates
the same tracker and exports `selection.lane_change_gap`; source fingerprints
include this module and invalidate old visual/web replay caches. It preserves
raw model and device-pose coordinates independently of radar display frames.

Windows TF tests substitute only unavailable Params storage, platform hardware
and UI event services. MPC assembly tests execute production update code with
a recording solver; they do not establish the closed-loop vehicle response or
represent an execution of compiled acados. No road test or safety guarantee is
claimed. Replay verifies sensor interpretation on recorded trajectories; ego
and other road users can react differently under changed control.
