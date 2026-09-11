# Noise-aware lead acceleration observer

`RadarLeadFilter` estimates lead velocity and acceleration together. Unlike the
historical two-filter derivative, it predicts the next velocity and corrects the
two motion states using the measured speed error. Its gain increases smoothly
when a persistent error is supported by another measurement in the same direction.
An estimate of measurement jitter suppresses that increase on noisy tracks.

The historical RC=0.10/0.15 s filter is the noise and timing reference. The current-
frame publication correction remains; comparison with the restored filter excludes
the older 50 ms publication bug, so that bug fix is not counted as this observer's
improvement.

## Scalar update

The essential operations, before the existing bounds and near-standstill handling:

```python
v_predicted = v_estimated + a_estimated * dt
error = v_measured - v_predicted
sample = clip(error, -0.5, 0.5)
evidence = max(0, previous_mean_error * sample)
noise_variance += noise_alpha * (0.5 * (sample - previous_sample)**2 - noise_variance)
mean_error += mean_alpha * (sample - mean_error)
weight = (evidence / (uncertainty_floor + 8 * noise_variance + evidence))**2
alpha = alpha_slow + weight * (alpha_fast - alpha_slow)
beta = beta_slow + weight * (beta_fast - beta_slow)
v_estimated = v_predicted + alpha * error
a_estimated += beta * error / dt
```

The first isolated error has no previous signed evidence and cannot immediately
select a faster response. Opposite-signed errors have zero evidence. The mean
residual uses RC=0.15 s and its short-term variation uses RC=0.50 s. The uncertainty
floor is `(0.10 + 0.8*dt)**2`, in squared speed units. These are calibrated constants,
not guarantees that coherent sensor error can always be distinguished from motion.

Slow gains are derived from the historical filter's poles. At 20 Hz they are
alpha=0.5 and beta=1/12. Fast gains correspond to RC=0.05/0.075 s: alpha=0.7 and
beta=0.2. This is interpolation between observer gains, not a switch on the sign
or magnitude of `aLead`, and not a drive-mode or response-level setting.

Acceleration correction retains the historical post-filter innovation bound
`3*dt/(0.15+dt)` (0.75 m/s² per 50 ms), plus the acceleration target limits.
When a correction saturates, the velocity state is adjusted consistently with
the bounded acceleration. Without that adjustment, merely replacing the cascade
with an equivalent linear observer changes its response to large speed spikes.
An independent regression checks equivalence with adaptation disabled, including
saturated corrections.

Per-frame work consists of scalar arithmetic, bounds and two exponential-moving-
average updates. There are no matrix inversions, history fits, or new settings.
The original independent jerk estimator is unchanged.

## Validation and tradeoffs

For a known 0 to -3 m/s² acceleration change at 20 Hz:

| Estimator | 10% response | 50% response | 90% response |
| --- | --- | --- | --- |
| Restored RC=0.10/0.15 s filter, current-frame output | 100 ms | 250 ms | 600 ms |
| Adaptive observer | 100 ms | 200 ms | 500 ms |

For 0 to -6 m/s², the 90% response improves from 600 to 500 ms; the 50% response
remains 250 ms. Weak -0.5/-1 m/s² transitions keep essentially the original
response. This preserves smoothing where evidence for a motion change is weak.
The numbers describe acceleration estimation, not vehicle brake application.

Tests cover 20/50/100 ms sample periods, multiple transition phases, initial
acceleration, constant-acceleration bias, correlated and independent noise, isolated
speed spikes, bounded monotonic braking and acquisition/reset. Known braking
threshold crossings are compared against the original filter without publication
delay. Tests also require a measurable settling improvement on clear braking.

On deterministic independent speed noise with standard deviation 0.10 m/s,
20 Hz acceleration RMS remains approximately 0.194 m/s². Correlated-noise tests
and steady acceleration also retain the original noise level within the test
tolerance. Recorded launch windows remain close to the restored filter's adjacent
acceleration variability. A retained noisy bus window increases that variability
by about 4.7%, rather than eliminating it. Such logged variability is not error
against ground truth, and does not prove the lead's actual acceleration.

This observer does not remove every source of noise or prove improved closed-loop
ride quality. Faster adaptation can also respond to a coherent sensor error.
Offline estimator comparisons and synthetic input tests must be distinguished
from vehicle validation after updating.

## Speed response and execution latency

These are distinct measurements. The acceleration observer's internal velocity
is not published as control speed: `RadarInterfaceBase` preserves each current
point's `vLead`/`vRel`, and `FastRadarOverlay` publishes current `v_ego + vRel` as
both `vLead` and `vLeadK`. Multi-frame regressions cover launch, cruise and braking
through both boundaries. This change adds no speed-filter sample delay; existing
sensor delay, scheduling and transport latency remain.

Internal speed is also compared with the historical RC=0.10 s speed state on
72 known launch/braking/stop trajectories (20/50/100 ms periods, four onset
phases, accelerations +1/+2/+3 and -1/-3/-6 m/s²). Every 10/50/90% speed crossing
is no later, and maximum absolute speed error is smaller. At 20 Hz, braking
from 20 m/s at -3 m/s² reduces maximum speed overestimation from 0.300 to
0.104 m/s; at -6 m/s² it reduces 0.600 to 0.222 m/s. This is synthetic known
motion, not a ground-truth measurement of logged lead speed.

Prediction briefly overshoots after acceleration ends: across those cases,
internal speed reaches as low as -0.227 m/s after stopping, then settles.
It is not sent to control as lead speed. Consequently these results do not
authorize replacing raw control speed with the internal state without further
validation. In steady constant acceleration its near-zero speed lag is a
prediction property, not zero detection latency at a new motion transition.

A Windows 11/Python 3.12.3 microbenchmark alternates versions across six rounds,
with 72,000 full `MyTrack.update` calls and 12,000 batches of 64 tracks per version.
Both versions retain the same independent jerk estimator. Times include the
measurement clock overhead (median 0.1 µs); outliers are retained.

| Scope | Version | Mean | 95th percentile | 99th percentile | Observed maximum |
| --- | --- | --- | --- | --- | --- |
| One track | Restored filter | 2.12 µs | 2.3 µs | 2.8 µs | 183.3 µs |
| One track | New observer | 2.95 µs | 3.2 µs | 6.1 µs | 269.4 µs |
| 64-track batch | Restored filter | 0.135 ms | 0.159 ms | 0.253 ms | 0.837 ms |
| 64-track batch | New observer | 0.191 ms | 0.233 ms | 0.396 ms | 1.092 ms |

The observed new batch maximum is about 2.2% of a 50 ms update period on this PC.
This is not a hard worst-case bound or vehicle timing result. Lightweight point
objects exclude CAN decoding, serialization, transport, planner execution and
competing vehicle workloads. A device-level timing measurement under normal
driving load is still needed to quantify end-to-end latency on the vehicle.

## Integration boundaries

- Front/corner radar tracks use the observer; their existing warmup, loss and
  reusable-corner-slot reset behavior remains. Strong braking does not reset an
  identified track's age and force acceleration to zero.
- SCC's reusable target slot retains its three-sample speed discriminator and
  RC=0.05 acceleration filter. This preserves its target-replacement detection.
- The existing near-standstill detector suppresses acceleration toward zero.
  Observer motion and noise state reset on track acquisition or reacquisition.
- `aLead`/`aLeadK` use the new estimate. Raw `vLead`, `vRel`, distance, lead
  selection, TF, response-level mapping, `aLeadTau` and model choices are unchanged.
- NAS replay uses recorded `liveTracks` inputs to recalculate selection. Deployment
  updates the code fingerprint/cache, but does not rewrite old recorded `aLead`
  or MPC plans. Filter comparisons on old velocity measurements are separate
  offline replays.
