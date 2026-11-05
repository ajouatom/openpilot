# ID.4 transient visual lead replacement, 2026-11-05

## Evidence

The reported VOLKSWAGEN_ID4_MK1 segment `0000029e--eeb1d0a58f--46`
was recorded on `carrot-wip-model_selector-ha` at `c6eb3b36a`.
Its front radar continues measuring track 281 while short vision distance and
speed outliers make the primary matcher reject it, including through the
farther-radar guard. Selection then immediately falls back to vision.

At carState-relative 44.634 s, the recorded radar lead is 49.16 m away and
moving at 24.72 m/s. At 44.693 s, the visual replacement is 33.27 m away at
9.06 m/s; at 44.734 s it is 38.81 m away at 7.67 m/s. Radar 281 returns at
44.834 s. Planner messages consuming these radar states reach aTarget -1.572
m/s^2 at 44.790 s. After 38 s there are 24 vision-only radarState frames,
in bursts approximately 0.04--0.14 s long, despite measured track 281.
The production baseline replay reproduces all 1,199 recorded L1 identities.
The log shows no radar/CAN error, invalid service, or dropped model frames.
It does not establish the optical/model cause or ground-truth target velocity.

## Change

Retain an already accepted, measured, central front-radar L1 in positive radar
modes until the candidate visual lead remains consistent for 0.20 seconds.
Use current measured radar distance, velocity, and acceleration during the hold;
do not freeze the previous output. Ordinary matches to another radar can switch
immediately. There is no unconditional delay for vision when no healthy prior
radar lead exists. No new parameter or branch-specific model change is needed.

Consistency uses the published visual coordinates, including model ego-speed
correction: adjacent predicted longitudinal error <=2.5 m, lateral change
<=0.75 m, and absolute speed change <=3 m/s. A jump restarts confirmation.
Camera timestamps advance the timer; repeated radar wakeups at the same model
timestamp cannot satisfy it. Observe the visual candidate while normal radar
matching still succeeds so existing confirmation paths do not acquire another
0.20-second wait.

The retained radar must have the same source/ID, be currently measured, have a
0--0.15 s observation gap, remain within 0.5--180 m and |dPath|<=1 m, and satisfy
the existing position/lateral/speed continuity limits. A stationary point also
needs the primary matcher's stationary identity; its independent contradiction
checks remain effective. Missing/unmeasured/discontinuous/off-path points and
invalid model geometry revoke the hold. Vision-only modes are unchanged.

## Validation

Compared with `4068a12695`, all 104 logs / 504 maintained case and trajectory
labels were replayed separately in modes 1, 2, and 3 with the production
controller. The new `id4-29e-46-night-vision-transition-281` case requires L1 281
continuously over replay 38.0--59.8 s, with no vision-only L1. It changes from
failure to pass in all three modes; no other pass/fail result regresses.
The case is explicitly `human_verified: false`; it checks sensor continuity,
not independently verified scene truth. Radar replay time is about 0.05 s
behind synchronized video in this segment.

| Mode | Baseline expectation failures including new case | Updated | Predeceleration failures, before/after |
| --- | ---: | ---: | ---: |
| 1 | 12 | 11 | 0 / 0 |
| 2 | 11 | 10 | 0 / 0 |
| 3 | 14 | 13 | 1 / 1 |

There are 208 unverified labels and no missing logs in each run. These are not
all-pass results. All CUT-IN event times are unchanged. One already passing
GV80 predeceleration case starts 0.048 s earlier in modes 1/2 and still passes.

A separate complete-frame audit at each log's recorded effective mode covers
124,489 frames. In 33 logs, 677 frames change from vision-only L1 to measured
radar L1; no L2 or predeceleration-candidate identity changes. In this ID.4 log,
32 frames change, including the 24 late-segment outliers. This is a shared
policy change, not a route-specific exception.

Local route-image workflow suites: 1,847 passed, four skipped; Ruff passes.
Added regressions cover exact 0.20-second switching at 10/20/40 Hz, modes 1/2/3,
short flashes, changing visual distance/speed/lateral position/model ego-speed,
latest measured radar values, invalid radar continuity, and repeated timestamps.
Existing contradictory-stationary and cut-in checks remain in the suite.

The NAS radar replay must be deployed through the Carrot Routes image workflow
and compared with the local export for this route. Its acceleration graph still
shows the recorded command, not a newly simulated planner/controller response.
Road comfort, emergency braking, and actual vehicle acceleration after this
change have not been measured. The contributor's vehicle branch requires this
shared `carrot-wip` change before the vehicle uses it.

Private reproduction reports and deployment verification are retained under
`.analysis/archive/2026-11-05-id4-vision-transition/`; original diagnosis evidence
is under `.analysis/archive/2026-11-05-id4-29e-46/`.
