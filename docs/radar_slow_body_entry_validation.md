# Slow near-body cut-in with moving model path (2026-09-11)

K8 segment `00000306--8d2e9a9a21--4` misses the left white SUV at the
user-reported video time 52.27 s. Full-cereal rlog decoding and qcamera review
identify corner 1516 and front 52. At video 53.00 s, aligned vision range is
7.77 m with probability 0.992, but recorded L1 still follows front 37 at
15.71 m. Both radar sensors see the closer vehicle.

The detector's path-relative history classifies this as lateral jitter:
at 52.25 s its 0.9 s path displacement is 0.123 m out of 0.634 m total
travel. The raw body positions are substantially more coherent. Meanwhile,
the last outside observation has left the ordinary 1.5 s history. At 53 s,
even direct visual support and present path overlap cannot pass those gates.

## Correction

Keep the timestamp and side of a measured outside observation for up to 3 s,
clearing them with physical track discontinuity. This memory is usable only
by a new independently corroborated near-body entry path. Ordinary prediction
thresholds and the 1.5 s motion history are unchanged.

The exception requires measured corner and front returns, both overlapping
the path corridor, a corner within 8 m, ego speed at most 12 m/s, closing
speed at most 5 m/s and at least 1.2 s until longitudinal passage. Corner
history must span 0.75 s and the same paired front 0.5 s. Recent yaw must
remain below 0.020 rad/s. Vision probability must be at least 0.90 and its
range/speed must either directly support the body or lie between that body
and the old farther primary.

Both sensors must independently show inward raw lateral displacement and
directional consistency. Rotation-corrected measured inward speed must be
at least 0.10 m/s on each. This resolves model-path jitter without treating
ego yaw or two sensors merely seeing a parallel car as entry evidence.
The same-frame confirmation already used for corroborated near-body entry
applies once this history is established; existing withdrawal, side-pass,
curve, existence and lead-competition vetoes still apply.

The diagnostic includes `bodyEntry=1`. This is a detector/lead-role change,
not a new setting or an altered sensitivity scale. L1 association is unchanged:
the entering vehicle reaches longitudinal control as L2 while the older L1
is still farther away.

## Verification

At video **52.2538 s** (replay 52.1526 s), corner 1516 becomes CUT-IN and L2
with front-backed range **8.9186 m**; L1 remains at 16.4410 m. Modes 2 and 3
and sensitivities 1 through 5 all detect at that frame. The exact corner ID
is used for the new regression: front 52 is an ambiguous association earlier
in the segment and must not allow an unrelated corner to satisfy the deadline.
L2 is continuous throughout replay 52.16-53.45 s. Through video 54.11 s,
there is no sampled gap where only the old >10 m primary remains.

The two new actual-route expectations fail on baseline `e3b8ac8c7c` and pass
on the correction. Synthetic left/right path-jitter positives likewise fail
on baseline and pass after the change. Negative tests cover weak/missing
vision, absent or unreliable front support, missing physical inward speed,
turning, discontinuity, absent/expired outside history, parallel movement,
raw front jitter, and incompatible visual range/speed.

Local focused detector/controller/replay/planner tests: 683 passed.
Additional image-workflow cut-out/lane-change tests: 94 passed.
Radar lead-filter tests: 280 passed. Route-vault tests: 28 passed.

The complete corpus contains 94 logs / 491 labels per mode, including the
two new K8 expectations. Relative to the 93-log / 489-label baseline, no
existing label gains a new failure. Existing unresolved expectation counts
remain **10 in mode 2** and **13 in mode 3**; mode 3 retains one existing
pre-deceleration failure. There are 208 unverified labels with absent target
input, and no missing logs. These are not an all-pass corpus claim.

See [the comparison summary](radar_slow_body_entry_validation.json). Recorded
acceleration traces are not recomputed by this offline replay. Detection and
lead delivery are verified; closed-loop vehicle braking is not measured here.
