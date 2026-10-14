# Carnival stopped-car selection and radar-only advisory

The user requested a correction and radar regression validation for Carnival
route `00000222--e22b2b29cf--1` on `0245e76f`, then requested a screen warning
with one short sound when vision misses an obstacle measured by front and
corner radar. No vehicle installation was requested or performed.

## Recorded cause and correction

The front radar was receiving other objects, but the relevant front track 57
first appeared at about 62.5 m. Corner tracks 1187/1193 had measured the
approaching stationary object earlier. Vision probability did not reach 0.4
until late in the approach. At incident time 53.50 s, the stopping model path
ended around 70 m and its final 2 m tangent pointed about 47 degrees sideways.
The apparent path offsets of the visual car and corner return were consequently
7.75 m and 5.15 m. This is not fixed by globally admitting corner-only L1 or
enlarging the ordinary matching thresholds.

A separate fallback supplies measured corner **L2 only while L1 is absent**:

- Both actual inner lane lines must have probability >=0.90, std <=0.25 m,
  plausible width, an uncertainty-dependent inward margin and near-field
  agreement with the original path. No lane extrapolation or lane change.
- The model must predict stopping. Current vision must agree with the stopped
  corner in raw range, lateral position and speed near the stopping path end.
- The same measured corner needs >=350 ms of consistent motion; vision needs
  >=3 strong observations spanning >=100 ms. Confirmation starts at p>=0.40.
  An already confirmed target may continue at p>=0.35 only within 150 ms of
  its last strong sample. Missing evidence, gaps, ambiguity and ID changes
  withdraw the fallback; unconfirmed radar-only observations never activate it.
- Output range is <=80 m, modes 1/2/3 only. A second equally plausible return
  beyond 80 m still rejects association; the output cap cannot resolve identity.
  A nearer existing L2 wins.
  Ordinary L1, front/SCC selection and cut-in geometry are unchanged.

Exact recorded-input replay changes 11 consecutive L2 samples, first at 74.32 m,
about 0.57 s / 9 m of travel before the original front L1 acquisition. The
replay clock is 53.542–54.046 s (video 53.642–54.146 s); the original incident
diagnostic used first-carState time, a different origin. All three modes have
the same 11 changed samples. These are selection results, **not measured
earlier braking, a safe stopping distance or closed-loop vehicle validation**.

The NAS/desktop adapter now retains actual model velocity, lane-change state,
lane lines and uncertainty. Raw invalid lane geometry cannot become evidence
through display sanitization. Parsed-frame cache version advances to 3. The
existing shared-source fingerprint also invalidates web replay caches.

## One-shot advisory

The existing `radarStationaryLead` event now displays “전방 물체 주의” /
“전방을 확인하고 감속을 준비하세요” with the existing single-play `prompt` sound.
It does not change control outputs or replace FCW.

The advisory requires active openpilot longitudinal control, no pedal override,
fresh valid model/liveTracks/radarState/carControl, no selected L1/L2, and all
visual lead probabilities below 0.4. Independently measured front (confirmed
native track state >=2) and corner must agree within 3 m longitudinally,
0.75 m laterally and 2 m/s in speed, remain stationary within 2 m/s, and occupy
the confident ego lane. The same pair needs 500 ms continuous evidence; reused
messages cannot confirm it. Ego speed must be >=36 km/h, closing speed >=8 m/s.

Trigger: constant-velocity TTC <=4.8 s and range <=120 m. At 60 km/h toward a
stationary object that corresponds to 80 m; at 100 km/h the 120 m range cap
applies. These are provisional advisory thresholds, not a braking-distance
guarantee. A pair may establish history out to 150 m, only within measured
lane coverage. One alert per encounter; rearming needs >=5 s without a
qualifying pair and >=30 s since the last alert. Track ID changes and brief
dropouts do not rearm it. Existing model/planner FCW suppresses this advisory.
The extra liveTracks subscription is excluded from controls-readiness checks.

There is **no new warning in this incident**: the matching front return is
absent at 80 m, and vision/lead selection has recovered when it arrives.
Requiring both sensors cannot provide an earlier warning without those inputs.

## Validation

- 1,176 focused radar, trajectory, replay, corner fallback and advisory tests
  pass on Windows Python. This includes freshness, duplicate samples, lane
  aliases, re-ID, missing sensors, speed-dependent timing and repeat suppression.
  The corresponding nav suite passes 1,157 tests; branch-specific test sets differ.
- Route-vault tests: 36 pass, 4 Windows symlink-permission skips. Native Linux
  CI is required before accepting the replay image.
- Existing 100 logs / 497 maintained labels: corrected-context baseline and
  candidate report files are byte-identical in modes 1/2/3. Existing expectation
  failures remain **11/10/13**; pre-deceleration failures **0/0/1**; **208 labels
  remain unverified**, missing logs zero. Do not call this a clean corpus pass.
- An all-frame mode-1 audit compares the candidate against the same controller
  with only the new fallback disabled: **119,689 outputs are identical** across
  the 100 prior logs. The offline advisory emits zero events, even with active
  engagement assumed. This screens unwanted new behavior, but supplies no
  real positive alert example and cannot establish an on-road false-alert rate.
- Two additional maintained Carnival cases pass in all three modes: continuous
  L2 1193 in the corrected interval and no early corner-only promotion. Existing
  labels are not weakened. Total corpus is now 101 logs / 499 labels.

Private input files, exact replay traces, complete reports and scripts remain
under `.analysis/archive/2026-10-14-carnival-222` and
`.analysis/scratch/2026-10-14-carnival-fix`. Publish the shared change to both
maintained branches with single-parent commits. Use the automatic Carrot Routes
image workflow and NAS updater, then verify deployed commit, actual result
page and recalculated replay. Branch publication is separate from vehicle
installation and road validation.
