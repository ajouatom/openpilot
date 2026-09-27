# EV9 tentative front-radar false lead, 2026-09-28

## Evidence

EV9 route `00000097--e22228b4eb--35` was recorded on clean
`carrot-wip` commit `835bcce07c`. A central front-radar track 35 persisted
for 22 samples, all native `trackState=1`, from 39.604 to 40.656 seconds
relative to the first `carState`. Its raw range fell from 31.05 to 8.55 m.
The recorded controller selected it as L1 for one frame at 40.674 seconds:
corrected range 9.80 m, absolute speed 6.42 m/s, `modelProb=0`.

The production replay reproduces that selection at replay time 40.647 seconds
(video time 40.700 seconds). Nearby vision lead probabilities are below 0.01,
and the inspected video frames do not show a corresponding close vehicle.
The reflecting object's identity is unknown. The L1 score of 0.963 describes
path alignment, not confidence that a vehicle exists.

The recorded planner requested -2.04 m/s², and measured acceleration reached
-1.20 m/s². Speed fell from approximately 97 to 92 km/h. These are original
vehicle measurements, not results of driving the correction.

## Correction and scope

The radar-only moving-primary candidate filter now excludes a front-radar
point whose native state is 1. Previously, a 0.75-second dwell could promote
that tentative return without vision. This event shows that a persistent,
plausibly moving tentative return is insufficient evidence for that promotion.
Increasing the timer would merely postpone the same decision.

Raw tracks remain available to vision-supported matching and diagnostic
publication. Unknown state 0 remains compatible with radar sources that do
not provide native state; states 2 and 3 retain normal radar-only acquisition.
A tentative challenger cannot displace a supported lead through this path.
Loss of native confirmation also removes a radar-only candidate; recovery
uses the existing acquisition window. Thus a real vehicle remaining in state 1
without vision support will no longer be selected by this fallback. This is
the intended evidence requirement, not a claim that every state-1 return is
false. Corner, SCC, stationary, scheduling and model policies are unchanged.

## Validation

- Isolated committed-source replay of all 1,200 incident frames: baseline
  reproduces the single false L1; the correction produces no L1 in the segment.
- New maintained case `ev9-97-35-tentative-front-phantom-35`: fails on the
  baseline and passes after correction in `EnableRadarTracks` modes 1, 2 and 3.
- 903 radar, lead, cut-in/cut-out and occupancy tests pass, including prolonged
  tentative rejection, native confirmation/recovery, supported-lead retention,
  unknown/confirmed-state compatibility and tentative vision matching.
- 254 longitudinal fast-radar and preview tests pass.
- Full comparison of 98 existing logs / 495 validation items in modes 1, 2
  and 3: no changes to selection, continuity, pre-deceleration or input-validity
  verdicts. Existing selection failures remain 11/10/13, pre-deceleration
  failures 0/0/1 and unverified items 208 in each mode. Strict corpus commands
  therefore still return nonzero; this is unchanged baseline debt, not a clean
  pass of every historical case. The new incident is checked separately above.
- The four new/revised rejection scenarios fail against the baseline and pass
  after correction; four existing compatibility/vision checks pass both.
- Route-service tests: 29 pass, four platform-dependent checks skipped locally.
  Ruff and whitespace checks pass.

Recorded-input replay does not establish corrected closed-loop braking or
vehicle validation. Private captures and reproducible analysis scripts stay
under `.analysis/`; no settings or public user-guide changes are involved.
