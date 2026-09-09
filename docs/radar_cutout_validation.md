# Predicted lead departure: bounded ACC headway relief

The change predicts a previously vision-confirmed moving front lead leaving
the ego path. It publishes `cutOutTime` and `cutOutConfidence` alongside the
unchanged measured lead. ACC uses these fields to reduce future headway demand;
the primary matcher does not drop the track on this signal.

## Admission and control

- Applies to a measured, confirmed-quality front primary in radar modes 1–3.
  A tightly associated corner can supply lateral motion. A changed lateral
  identity or a measurement gap above 0.15 seconds resets the evidence.
- Requires 0.30 seconds of earlier central vision agreement, no more than
  2 seconds ago. Current visual probability must be at least 0.40, and its
  farther range must exceed radar by more than 3 m, 25% of radar range, and
  the visual range standard deviation.
- Both recent motion windows must move outward, with at least 0.20 m net
  progress and 75% net-to-total lateral travel. Raw lateral movement must
  support the path-relative movement. Strong turns, path shifts, stopped
  leads and lead braking below -2.5 m/s² do not receive relief.
- Predicts body clearance at 2.15 m from path center, within 2.5 seconds.
  The projected gap through clearance plus 0.30 seconds must exceed 6 m;
  the planner also applies the configured stopping distance. The projection
  assumes ego acceleration of 0.5 m/s² and at least 0.5 m/s² lead deceleration,
  or the measured stronger lead deceleration.
- Confirmation takes 0.10 seconds, followed by a 0.30-second strength ramp.
  Loss of evidence revokes relief. There is no stale-signal hold.
- Every MPC obstacle sample before clearance plus 0.30 seconds stays intact.
  Subsequent samples ramp the headway credit over 0.50 seconds. Credit is
  capped at half the configured time gap, 0.50 seconds and 8 m.
- LeadTwo, cruise and traffic-stop obstacles still compete independently.
  Original lead kinematics, FCW trajectory and predicted-danger diagnostics
  remain unchanged. Pedal override, stopping, forced slowing and blended
  mode disable relief. Missing fields in older messages default to inactive.

The existing probabilistic CUT-OUT predictor remains responsible for its
stationary-shadow use. Its far-horizon occupancy score alone is not sufficient
evidence of lateral body clearance for this control change.

## Validation on 2026-09-08

- 633 focused tests passed, including mirrored departures, aborted departures,
  re-entry, changing IDs, missing measurements, path-only movement, noisy radar,
  vision disagreement, close-range risk, schema compatibility, fast radar,
  stopping and the production MPC update with a recording solver.
- Replayed all 88 maintained route segments / 477 labels. An A/B comparison
  against the controller at `6f4c00e625` found **zero differences in existing
  output fields across 105,300 frames**; only new CUT-OUT metadata differed.
- The existing label set reported 11 detection expectation failures and 208
  unverified labels, identically in the comparison run. Pre-deceleration had
  zero expectation failures. These pre-existing misses/unverified labels are
  not presented as passing tests.
- New metadata was active for 8 frames in 4 maintained segments. Camera
  frames and subsequent same-track positions were inspected for each. All
  four targets subsequently moved outside the path corridor. One forecast
  was early relative to observed full clearance, reinforcing the need for
  bounded credit and immediate cancellation rather than dropping the lead.

| Segment | First signal (s) | Front ID |
|---|---:|---:|
| 000004f4--e1ae2223e7--14 | 10.807 | 46 |
| 000001d6--b3030f2f2e--28 | 35.345 | 36 |
| 000002d3--74e5ed8754--7 | 37.713 | 33 |
| 00000cfb--69588de3d7--6 | 36.904 | 39 |

The reported Ioniq 5 example is included as a numerical fixture with no vehicle
identifier. Its first nonzero signal is at approximately **11.403 s**, predicting
clearance in 0.688 seconds at strength 0.164. The driver had already pressed the
accelerator; actual longitudinal control was off. The change does **not** claim
to prevent that earlier pedal intervention. At the screenshot time, 11.254 s,
the range was already correctly retained by radar at approximately 11.0 m.

A separate local SciPy reference optimization of the MPC objective confirmed
the intended direction of headway relaxation and identical results with a
closer second lead or traffic stop. This is not an execution of the device's
compiled acados solver. No closed-loop vehicle test or device build was run on
this Windows host; replayed sensor inputs cannot establish the new closed-loop
vehicle trajectory.

Run the focused tests with pytest in the normal openpilot build environment.
On Windows without `params_pyx`, use an empty pytest configuration and
`--confcutdir=openpilot/selfdrive` for the pure-Python focused tests. The existing
route validator is
`openpilot/selfdrive/carrot/radar/tools/validate_radar_lead_model.py --shadow-only`.
