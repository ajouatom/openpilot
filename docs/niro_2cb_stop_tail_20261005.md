# Niro 2cb segments 8–9: remaining terminal crawl

Private incident analysis, 2026-10-05. Route `000002cb--4bcf20d5bc`, segments
8–9, KIA_NIRO_EV / dongle `88baf851efe58d6c`. Source rlogs/videos remain on NAS.
No production changes were made during this review.

Both initData records identify clean `7135acf20989314d927b6ca3980f9eb939b5ece7`,
which includes the distance-band change `be1a9442f0` and J20 trial. Response 3,
TF 0.9 s, manual Normal mode, 6 m stop distance, VEgoStopping 0.1 m/s and
StoppingAccel -0.5 are unchanged from 2c6. Boot setting LongActuatorDelay is
0.20 s versus 0.30 s previously; CP's baked-in delay/stopping values are not
the runtime Params values. These are not controlled A/B drives.

Time zero is segment 8's first carState at boot 536.443112619 s, corresponding
to approximately 2026-10-05 14:03:06.275 KST. Segment 9 first carState is +60.061 s.
All decoded carState/control/plan/radar/model/liveTracks services are valid.
Longitudinal control remains active with no pedal intervention or forceDecel.
Lead 33 remains selected throughout, with no second lead. Video contact frames
show following the same white SUV to a red signal; video alignment is approximate.

## Observed approach

| Event | Segment-8-relative time | Measured ego speed | Published lead range |
| --- | ---: | ---: | ---: |
| Below 10 km/h | 57.772 s | 9.990 km/h | 9.416 m |
| Below 5 km/h | 60.182 s | 4.994 km/h | 8.714 m |
| Lead absolute speed below 0.3 m/s | 61.304 s | 4.008 km/h | 8.038 m |
| Planner shouldStop | 67.191 s | 1.993 km/h | 3.225 m |
| First SCC12 StopReq | 67.213 s | approximately 1.99 km/h | approximately 3.2 m |
| Ego below 0.1 m/s | 68.389 s | 0.353 km/h | 2.688 m |
| Standstill flag | 68.402 s | 0.343 km/h | 2.688 m |

Compared with 2c6, below-10 duration is 10.617 versus approximately 13.400 s,
and below-5 duration is 8.208 versus 8.481 s. Peak measured acceleration in
40–69 s is -1.788 m/s²; target minimum is -1.828 and jTarget minimum -0.826.
Earlier 2c6 peak measured deceleration was -2.341, but lead motion differs;
these differences cannot establish change-induced comfort improvement.

## Extra headroom is essentially absent in this tail

Reconstructed current reference uses the same fast selected lead when active,
published lead otherwise, plan initial speed, response 3 and TF 0.9. Current
reference agreement over 45–69 s has MAE 0.01625 m and maximum 0.11338 m.
Starting state before the provided segments is unknown, but it converges well
before the terminal interval. The 52–68 s maximum extra TF is 1.18e-6 s and
maximum additional current reference 9.90e-6 m; at 58–68 s it is below 2.6e-8 m.

Using the old 7432 gap rule on the same recorded inputs produces effectively
the same current headroom in this interval (maximum difference 8.1e-12 m).
The current distance ratio never exceeds 1.216 in 52–68 s. This is not a useful
demonstration of the new rule removing a large current allowance. It is also
not a full counterfactual MPC/vehicle replay: future margins and changed vehicle
feedback are not established by a current-state reconstruction.

## Low-speed tracking is still the outstanding issue

At approximately 64 s the recorded plan initial/target speed is 2.00 km/h but
measured ego speed is 3.01 km/h. Requested actuator acceleration is -0.538 m/s²
while measured acceleration is only -0.043. At 66 s the corresponding speeds
are 1.05 and 2.40 km/h, with request about -0.700 and measured -0.130 m/s².
Thus the planner is asking for further slowing, but the vehicle continues
faster than the planned speed through much of the terminal approach.

| Window relative to first StopReq | Median SCC12 request | Median aEgo |
| --- | ---: | ---: |
| -1.5 to -0.2 s | -0.680 m/s² | -0.115 m/s² |
| +0.35 to +0.8 s | -0.620 m/s² | -0.504 m/s² |

Stronger actual braking follows entry to stopping mode despite a less negative
CAN request, repeating the earlier observation. This correlation does not
isolate stock ECU interpretation, braking/regen blending, road grade or StopReq
as a single physical cause. Actual acceleration/request are different signal
types; their difference alone is not a calibrated actuator error measurement.

Settled radar range at 72–80 s has median 2.876 m versus 3.299 m in 2c6; these
are measured object ranges, not surveyed bumper clearances. The logged following
reference settles to 6 m. It would be inappropriate to justify further gap
reduction by describing this tail as remaining excessive target headroom.

Assessment: the overall approach is milder and less stretched than the older
example, but terminal 2–3 km/h motion remains unsatisfactory. Retain the new
distance-band rule rather than increasing recovery solely from this log.
If further tuning is requested, first investigate the low-speed speed-target,
stopping transition and vehicle response together; do not infer that earlier
StopReq or stronger braking alone is a demonstrated fix. This review does not
certify physical clearance or driving safety.

Reproduction, selected metadata, timeline, reference reconstruction, CAN-window
metrics and inspected video contact sheet are retained in
`.analysis/archive/2026-10-05/niro-2cb/`. Decoded messages and video/frame copies
are reproducible from the NAS source and removed after analysis.
