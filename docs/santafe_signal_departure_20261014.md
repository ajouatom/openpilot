# Santa Fe 135-3: unresolved green departure hold

The user reported another failed departure on carrot-wip 0cd29a0d1c. The shared
segment 00000135--99b6cc229f--3 confirms the failure. The earlier midday task
published investigation findings, not a correction of identity loss. The older
post-release timer correction cannot help when green release is never confirmed.

## Recorded evidence

Downloaded front video, preview video, qlog and rlog (53,960,847 bytes), checked
against the public manifest lengths and recorded local SHA256 hashes. These are
download integrity records, not independent source hash verification. Only this
one segment is shared; no earlier red-acquisition history is available.

The log confirms device mici and commit 0cd29a0d1c. Its tracker hash is
918654b8c3d2bf60640419f5433e87c414a77efa4a875ae744f5ef7944c9d46d, matching the
installed-commit source and the current baseline. Soft hold is zero throughout.

Two overhead red lights visibly change to green. First sampled overhead green
capture is 19:33:05.513 KST, arriving at 19:33:06.066. The car remains stationary,
longitudinal control active, and the auxiliary red hold stays set until gas at
19:33:10.072: 4.559s after that capture. There is no recorded auxiliary green
release. This establishes a blocking constraint, not a full counterfactual that
the original controller alone would have departed at a particular instant.

The 166 observer results run at 2.79Hz. Median capture gap is 349ms, p95 599ms,
maximum 897ms. Median result age is 289ms, p95 488ms, maximum 758ms. Median work
wall time is 233ms versus 70ms process CPU time. Scheduling/preemption contributes
to elapsed time; these logs do not isolate the competing process. Only two
results exceed the stopped guard's 650ms age limit, but repeated planner reads
can also age a result out before its successor arrives. The display's 200ms
freshness classification must not be confused with that separate stopped limit.

## Why green cannot qualify

The overhead red tracks 250/251 become green tracks 253/254. Subsequent green
frames repeatedly receive new IDs (257/258, 261, 264, etc.). Their candidate boxes
shift when the illuminated lamp changes. Fresh green is visible in the image and
raw proposals; it is not absent from the camera.

The tracker retains new candidates only for 250ms, confirmed-red identities for
500ms, and uses stricter reacquisition after a 250ms gap. Slow frames repeatedly
clear identities. Transport requires three producer observations before exposing
a colour to the guard; many fresh green proposals consequently become unknown
before the guard's independent 700ms/three-sample confirmation can begin.

Vehicle rear lights are also proposed as red. For example, the region near
[929,350,977,374] belongs to the right-hand vehicle, but falls inside the fixed
forward image ROI. Detection is not signal-object or ego-lane classification.
Do not solve the hold by blindly extending all identities or allowing any green.

## Correction attempt and rejection

A private candidate allowed retention up to 700ms only with high-score, unique,
near-stationary image correspondence and reused a uniquely overlapping current
proposal instead of a mismatched template box. It left colour freshness and the
consumer green-confirmation contract unchanged.

Paired encoded-image replay uses 165 matched capture frames and recorded arrival
times, with nominal/+20ms/+100ms arrival variants. Both baseline and candidate
produce zero green releases before driver override. The candidate also retains
the rear-light false candidate more persistently. It fails the requested outcome
and was restored out of production byte-for-byte. Encoded-image outputs are not
bit-exact VisionIPC results; this experiment is not closed-loop validation.

A separate 12-frame probe with the already available pinned YOLOX/MobileNet
reference sees red/green in the selected transition images, but its boxes also
move with the illuminated lamp and sometimes duplicate one physical housing.
That probe is not proof of a corrected identity or departure pipeline. No trained
weights or new detector were promoted.

## Status

The departure failure remains unresolved. A useful correction must identify a
whole signal housing separately from its lamps, reject vehicle rear lights, and
qualify fresh transitions at measured processing cadence. These recordings
already reproduce the failure; additional driving is not needed to establish it.
No control threshold, runtime, model, setting or vehicle installation changed.
This report records a failed correction experiment, not a successful fix.

Private evidence: `.analysis/scratch/2026-10-14-santafe-start/`, including manifest,
downloaded hashes, decoded logs, review images, baseline and failed tracker
snapshots, replay script, image_results.json, guard_results.json and neural_probe.
