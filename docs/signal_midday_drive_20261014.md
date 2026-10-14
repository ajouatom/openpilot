# Midday signal drive investigation, 2026-10-14

The latest drive does not demonstrate a useful overall signal-assist improvement.
There is a visually confirmed green departure hold, a late acquisition of two
visible red lights, and an uncovered creeping-speed interval. Removing the
post-confirmation stop timer does not solve these earlier failures.

## Evidence and installed version

Read-only collection from the user's vehicle obtained 127 completed qlogs plus
rlog/front video/preview video for 19 segments of routes 10be and 10bf. All 184
unique files were SHA256 checked against the vehicle originals. Detailed coverage
includes the outbound drive around 12:07--12:13 KST, the latest return around
12:30--12:35, and subsequent stationary recording. The 19 segments contain 9,765
observer outputs; this includes parked time and is not a detection accuracy count.

Vehicle checkout was carrot-nav 1b42724c42, SignalAssistMode=2. Route initData says
dabe4f1d67. The relevant signal tracker and carrot_functions.py have no difference
between those commits; the logged tracker source SHA256
918654b8c3d2bf60640419f5433e87c414a77efa4a875ae744f5ef7944c9d46d matches both and the
analysis checkout. The prior release-timer fix is included in this code lineage.
The version-label discrepancy is not evidence of an older signal implementation.

## Confirmed stopped green fails identity association

In 10bf-3, the same overhead housing changes from red to green. Its first sampled
green capture is 12:33:01.771, observer arrival 01.894; gas starts at 04.049, 2.278s
after that capture. Before gas, longitudinal control is active, speed is zero,
brake/blinkers/soft hold are inactive, and the red veto remains held. There is no
recorded green release.

The red core was track 1901, roughly [507,262,531,274]. The green core becomes
track 2000, roughly [519,260,551,276], without red history. Both boxes cover parts
of the same visually reviewed housing. The ordinary tracker centre-distance gate
rejects that shift; the guard's fixed red-box overlap and same-ID fallback also
cannot join them. Fresh green pixels remain available, but they do not complete
the anchored red-to-green confirmation. This is not the removed 0.5s post-release
timer, an old display colour, or absence of a green image.

The outbound 10be-1 case is different: track 987 retains identity. Green capture
at 12:09:01.612 arrives at 01.780; gas starts at 02.401, only 0.621s after the first
green arrival. Driver override precedes the guard's 700ms confirmation. Do not
describe that case as a proven indefinite green hold. During the preceding stop,
reviewed red images also have long intervals without a fresh red candidate.

## Red acquisition is late, and creeping is not protected

The next 10bf-3 approach shows two real red lamps in the reviewed frames around
12:33:27--30. At 12:33:29.606 the driver brakes from 47.55km/h. Confirmed red on the
reviewed overhead group arrives at 31.874 (capture 31.723), after manual braking.
This is acquisition failure before timely preparation, not proof that a qualified
red preparation request was ignored. All reviewed segments have zero active
e2eStopPrepare frames. Recorded preparation reasons include ineligible/no_red/
stale/awaiting_fresh; the coarse ineligible log does not identify every failed gate.
Driver pedals, turns, lead context and the 10s pedal cooldown must not be conflated
with detector misses. Recorded confirmed moving reds have no samples passing the
analysis script's basic active/no-pedal/no-lead/no-turn/not-already-stopping screen.

After brake release around 12:33:35.8, the car remains moving about 2.2--4km/h while
the overhead lights are still red. The base planner briefly enters e2eStop, then
returns to e2eCruise/green; one sampled original model x[-1] is 49.91m at about
12:33:36.8. The auxiliary has fresh red, but the stopped guard reports
moving_no_entry. It can acquire a new hold only at <=0.3m/s (1.08km/h), while moving
preparation requires >5km/h and is also subject to the pedal cooldown. This is a
real gap between the two helper policies. It is not a confirmed-green release.
Positive longitudinal actuator acceleration is present before the later sampled
green at 12:33:40.770. No claim of crossing the intersection or stop line is made.

## Actual cadence and offline image checks

Latest return moving segments 10bf-0..3 run at 7.76/6.49/7.67/6.92Hz. Typical
capture gaps are 102--150ms; median result ages are 107--131ms. Outbound 10be-1
falls to 4.41Hz, median gap 242ms, p95 gap 307ms, median result age 170ms and p95
240ms. Its median image work is 120ms versus 67--76ms in the return's signal
segments. A 20Hz configured ceiling is not a measured 20Hz detector.

Seven exact encoded-image probes supplement the logged camera observations.
On 10bf-3 encoded indices 518/560, the current detector produces no red proposal
on the reviewed real signal pair; index 600 has no proposal at all. At 621 the
housing/day-core proposals do find the pair. Thus the distant miss also appears
in encoded-image replay, before guard logic. Outbound probes mix success and
failure and do not exactly reproduce every live result. Encoded pixels differ
from the original VisionIPC input; do not call this bit-exact perception replay.

## Next implementation targets and scope

1. Track a visually supported whole housing/group, with lamp slots inside it.
   A lamp switching from the left to right must not create an unrelated signal.
   Preserve ambiguous-neighbour and conflicting-red rejection rather than simply
   widening all green matching thresholds or shortening confirmation.
2. Prioritize native-pixel road-ahead crops and frequent updates of acquired
   housings. The proposed 30m/70m, road+5m reference can rank search areas but is
   not measured range, lane assignment, or permission to go.
3. Review the <=5km/h transition between preparation and stopped hold explicitly,
   including manual-brake release, cooldown, no lead, model flip and fresh red.
   A blanket moving-red override is not validated by these observations.

No new training, model, controller, parameter, or vehicle deployment was performed.
This is an investigation with concrete reproducing evidence, not a successful
fix or closed-loop validation. Additional driving is not needed to reproduce
these particular failures. No runtime changes means no new runtime test claim.

Raw logs, video, diagnostic crops and scripts remain private under
`.analysis/scratch/2026-10-14-latest-drive/`: verified_manifest.json, analysis.json,
focus.json, transition_metrics.json, candidate_probe.json, and review/.
