# Earlier stationary L1 acquisition from continuous visual/front position evidence

## Problem and behavior

Carnival route `000001c8--95bc7cab1c--1` on `daeda9d19f` has a delayed
stationary lead acquisition. At video 40.542 s the model and measured front
track 51 agree at about 80.3 m, but the model reports 14.0 m/s while radar
reports approximately zero speed. The existing front-position exception
waits for 0.50 s of radar observation before starting another 0.25 s of
stationary confirmation. L1 therefore remains vision-only until 41.294 s.

The matcher now also records uninterrupted visual/front position support.
When that history has already completed the stationary confirmation interval,
it can qualify the existing speed-error exception and complete L1 promotion
without starting a second dwell. Front radar owns the resulting L1 and supplies
its distance and speed. Corner radar is not required for this acquisition.

## Admission and cancellation

Each history sample must satisfy the existing measured-front quality and
position-anchor gates:

- measured front radar, track state at least 2, distance at least 60 m;
- absolute radar speed at most 2 m/s;
- visual/radar distance error at most 5 m and lateral error at most 1.25 m;
- absolute radar dPath at most 1.25 m and absolute yaw rate below 0.10 rad/s;
- continuous radar position/time and no radar speed jump over 2 m/s.

The additional history requires model existence probability at least 0.70
on every sample. Its adjacent range change must agree with trapezoid-integrated
relative velocity within 0.75 m. Missing tracks, weak measurements, lost visual
position support, resets and discontinuities restart this history.

Promotion requires at least 0.25 s of this history, at least three samples,
and current model probability at least 0.80. It still passes the existing
anchored-front velocity checks: model/radar speed difference at most 24 m/s
and at most four model velocity standard deviations. It does not treat high
existence probability as proof of an accurate speed estimate.

Existing competing-moving-object vetoes, near-range velocity conflicts,
primary source/setting policy, stationary retention, handoff and path departure
rules remain in force. A supported visual moving car must not lend its
existence to a nearby stopped reflection. The old 0.50 s anchor path remains
available for cases that do not qualify for the additional history.

## Validation

Local replay against the pre-change matcher:

| Carnival 1c8--1 | Baseline | Updated |
|---|---:|---:|
| First physical L1, video time | 41.294 s | 40.793 s |
| Front 51 distance at acquisition | 64.210 m | 74.882 m |
| First physical L1 with all corner input removed | 41.294 s | 40.793 s |

Acquisition is 0.501 s earlier, at a measured distance 10.671 m greater.
There are ten additional front-L1 frames; an eleventh output differs in its
acquisition score. Both normal and front-only replays show the same acquisition
gain across all 1,199 frames. The earlier stationary Carnival 16f--3 example
and Carnival 178--53 / Ioniq5 cb5--3 reflection counterexamples retain exactly
the same full controller outputs as baseline (1,200 / 1,199 / 1,198 frames).

The maintained corpus adds a timed L1 case for 1c8--1, with a deadline at replay
40.75 s and normal-input continuity from 40.70 through 41.65 s. Video time is
about 0.099 s after replay time. The new case passes with production inputs.
Removing all corner input still passes the acquisition deadline but does not
satisfy that longer continuity interval: the existing near-range speed-conflict
veto later releases the unpaired front. This is not a claim of uninterrupted
front-only tracking through the later handoff.

The pre-existing full corpus has 95 logs and 492 items per mode. Every report
row is unchanged for modes 2 and 3, including existing failures: 10 / 13 primary
expectation failures, 0 / 1 pre-deceleration failures, and 208 unverified items
due to insufficient labelled input. No logs are missing. This is a
no-new-regressions comparison, not an all-pass corpus result.

1,125 focused tests pass: 422 predictor/matcher tests, 384 lead integration
tests, 28 route-vault tests, and 291 input/filter tests. Fourteen new cases
cover early front-only promotion in modes 1/2/3 and interrupted probability,
position, lateral support, track quality, range continuity, radar input, yaw,
reset, precise contradictory model speed, a competing moving front and weak
current confidence. Ruff and whitespace checks pass.

These results use recorded model/radar inputs, not closed-loop vehicle control.
They do not establish braking distance or vehicle safety improvement. The later
41.8 s track 51/53 handoff remains outside this acquisition change.

## Deployment

The shared production matcher is bundled into Carrot Routes. Its Python source
participates in `source_version()`, so the change invalidates server replay
caches. Deploy through the existing Carrot Routes image workflow and NAS
scheduled updater; verify the intended source commit, updater state, actual
1c8--1 result page and recalculated frames/graphs against local export.
