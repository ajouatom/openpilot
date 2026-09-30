# ID.4 same-road model and learned-parameter comparison

Follow-up implementation: the user subsequently requested applying manual SR
and live scaling to MEB. The MEB bypass described below applies to the recorded
commits, not the updated controller. The new controller honors CustomSR and
SteerRatioRate; defaults 0/100 retain unscaled learned SR. See
[the settings guide](user/en/settings.md). This change is not a validated fix
for the inward-curve complaint analyzed here.

History: the bypass originated in
[tjddyd0130's PR #427](https://github.com/ajouatom/openpilot/pull/427), merged
2026-07-04 as `f24623f3950f49aa289077baf38235be6c223542` (Git author sodoii3).
`99319aa7b0fc76f109f4688e02078cbec2f0356d` later extracted the existing bypass
into steer_ratio.py while adding invalid-rate validation; it did not originate
the MEB exception. The original rationale was preserving infiniteCable2's
learned-ratio convention for the curvature frame correction.

## Scope and conclusion

The user reported inward curve positioning around 13.1 s and 28.6 s of
`00000112--2d04be7e18--57` and supplied `000001b2--6f0b8f69ff--3` as a better
drive with another model. Full rlogs and qcamera video were inspected.

The newer log records `DrivingModelName=RDFv4`. GPS and video identify the same
road; the nominated positions correspond approximately to 19.5 s and 35.7 s of
the newer segment. The comparison does not isolate a single cause. Learned SR
is very similar at these positions, while learned stiffness, steering offset,
speed, initial position, driver input, camera trim, lighting and wet-road
conditions differ. Target-curvature timing also differs before the curve apex.
Neither an SR-only explanation nor a LatSmoothSec-only explanation is established.

Good instantaneous target tracking does not exclude an earlier accumulated
position/heading error. Conversely, lane position reported by two different
models is not independent physical ground truth.

## Provenance and method

Earlier source: `69b2c22e3ea874f5308b60690c3381b8430b48cd`, ajouatom/openpilot,
carrot-wip, dirty=true. Its bundled default is CD210CombinedSupercombo, but the
runtime model hash is not recorded; a local replacement cannot be excluded.
Later source: `4448c79c0782c2073c676156d05c7b1c77f66bb2`, **helico717/openpilot**,
carrot-wip-model_selector-ha, dirty=true. RDFv4 is the recorded selected name,
not a measured hash of the running artifact.

The first carState corresponds approximately to 2026-09-01 16:11:59.948 KST
and 2026-09-30 19:10:56.794 KST, respectively. Segment time uses first carState,
not the boot-time initData event. Later GPS positions were projected onto the
earlier GPS polyline to align road distance. Alignment has metre-scale uncertainty;
do not use GPS differences here to establish sub-metre lateral displacement.
Numeric point comparisons below average a one-second window about each position.

Full-schema decoding finds 1,200/1,199 modelV2 messages and 6,005/5,999 carState
messages in the earlier/later segments. All inspected modelV2, cameraOdometry,
livePose, liveParameters and carState message-valid flags are true. No steering
fault is recorded. These checks do not establish absence of all timing issues.

## Matched-position observations

| Quantity | Earlier first curve | RDFv4 first curve | Earlier second curve | RDFv4 second curve |
|---|---:|---:|---:|---:|
| Segment time, approximately | 13.1 s | 19.5 s | 28.6 s | 35.7 s |
| Speed | 47.80 km/h | 44.22 km/h | 47.45 km/h | 48.36 km/h |
| Learned SR | 14.9807 | 14.9742 | 14.9024 | 14.9680 |
| Learned stiffness factor | 0.6895 | 0.9995 | 0.6903 | 0.9995 |
| Learned steering offset | -3.345 deg | -3.048 deg | -4.391 deg | -2.945 deg |
| Curvature tracking error times v², RMS | 0.0265 m/s² | 0.0264 m/s² | 0.0475 m/s² | 0.0913 m/s² |
| Detected lane center relative to ego, right positive | -0.502 m | -0.784 m | +0.718 m | -0.037 m |

All four windows are laterally active without steeringPressed. The second-curve
lane estimate is much nearer center in RDFv4; the first-curve estimates do not
establish a consistent improvement. Lane probabilities are not uniformly high:
the second RDFv4 window averages approximately 0.47 left / 0.28 right. Exact
physical offsets cannot be inferred from this table. Older video is rainy
daylight; newer video is dark nighttime with droplets visible.

The older drive has steeringPressed from 29.43–33.76 s after the second nominated
point. RDFv4 has no corresponding driver intervention there, but has intervention
at 9.87–11.71 s before the first curve. This changes the entry conditions.

## What the earlier entry shows

Over the geographical interval corresponding to old 6–10 s, integrating target
curvature along GPS distance gives approximately 14.60 deg in the old drive and
11.91 deg in RDFv4. The corresponding pose-based integrals are 11.43 and 8.90 deg.
Both lag their own requested turn; the old drive already requests more turning
over this early interval. From old 10–13.1 s, target integrals are 24.69 / 25.68 deg
and pose integrals 25.71 / 26.51 deg. Thus a comparison at the apex alone misses
the different timing and previous position. These are descriptive integrals,
not a closed-loop replay or an absolute road-heading reference.

At left-curve entry (old 17–20 s), target integrals are -10.46 / -9.64 deg,
and pose integrals -8.41 / -7.38 deg. The following interval partly compensates
this difference. Initial lane-relative position also differs, so these observations
do not independently assign causality to the model or to learned parameters.

## SR, stiffness and offset

The MEB correction remains:

```text
output = curvature_PID + EPS_curvature - VM_curvature_without_roll
```

VM curvature depends on learned SR, stiffness and offset. Above 5 m/s with a
calibrated pose, PID feedback uses pose yaw rate / speed, not VM curvature.
CustomSR and SteerRatioRate are intentionally bypassed for MEB in both commits.
The saved rate changes from 100 to 30, but does not multiply MEB's SR by 0.3.

Using new-drive parameters at corresponding positions in **frozen old states**,
the SR-only change alters mean turnward pre-limit command times v² by about
-0.0031, -0.0014, +0.0026 and +0.0072 m/s² over old 5–10, 10–13.6, 17–23 and
23–29.1 s. Stiffness-only changes are approximately -0.0296, -0.0485, -0.0605 and
-0.0699 m/s². Offset-only changes are -0.0400, -0.0190, +0.0483 and +0.0871 m/s².
Positive means more turnward command. These calculations include the stiffness
effect on roll feedforward (Kf=1), retain PID state/yaw/angle/roll, and do not
simulate changed vehicle response, model inference or limiter state.

The actual SR difference is a much smaller immediate influence in this calculation
than stiffness/offset. That does not prove stiffness is wrong or the root cause.
paramsd explicitly resets stiffness to 1.0 at each normal startup, while retrieving
saved SR and angle offset. This older segment is about 57 minutes into its route;
the RDFv4 segment is about 3 minutes in. Long-term adaptation and wet conditions
are confounds. Do not attribute 0.69 versus 1.00 solely to changing model.

## Model, smoothing and estimator dependence

Both logs use SAD=9 (0.09 s), LatSmoothSec=13 (0.13 s) and no lane-based steering.
The later 19:08:58 settings snapshot corroborates these values. CameraYawTrimDeg
changes from -20 to -10, meaning -0.2 to -0.1 deg. Learned lateral delay is
0.2864 / 0.3037 s, but positive manual SAD selects 0.09 s in both code paths.

Computed adaptive smoothing is essentially 0.13 s through the RDFv4 curves.
The old model has modest additional smoothing and short spikes: for example old
10–13.1 s averages about 0.1566 s versus 0.13 s in RDFv4. Equal saved LatSmoothSec
does not imply equal effective smoothing. This difference is a candidate, not an
isolated cause. The earlier fixed-plan zero-smoothing calculation did not establish
a fix or predict a driven trajectory.

Public RDFv4 metadata has an action output; the bundled CD210 has none. The
postprocessor uses direct action/v² for the former and yaw-plan conversion for
the latter. Both are smoothed. Runtime hashes are unavailable in these captures.

The feedback called "actual curvature" here is a livePose estimate. It combines
IMU and model camera odometry. SR learning consumes this same pose estimate plus
CAN angle/speed. Model-dependent pose bias therefore cannot be excluded simply
because target and estimated actual curvature agree. Raw gyro comparison also
requires bias, mounting and time alignment; raw uncalibrated yaw is not ground truth.

## Code checks, limits and retained evidence

Exact-commit comparisons show identical steer_ratio.py, paramsd.py, locationd.py,
vehicle_model.py and Volkswagen carcontroller.py/values.py. The model smoothing
and action-conversion functions have identical ASTs. Other software changed,
including longitudinal tuning and camera runtime; both trees are dirty. This
is not a controlled model-only A/B test, and does not establish vehicle safety
or validate any proposed steering tuning.

Private reproducible evidence is archived locally under
`.analysis/archive/2026-09-30/id4-model-compare/`: comparison script, extracted
timelines, selected metadata, numeric report, plot, video stills and source checks.
Original full logs/video remain on NAS. Captures, settings and temporary packages
are not committed. No vehicle settings or steering runtime code were changed.
