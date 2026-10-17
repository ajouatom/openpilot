# Evening/morning signal recognition review, 2026-10-18

The morning drive contains a real green departure veto failure. The highest-row
and central-selection experiments are **not connected to production**, and are
not the cause of this recorded failure. No vehicle code, settings, model,
processes or control policies were changed during this investigation.

## Evidence and version

- Read-only collection: 129 qlogs, 69 rlogs and 15 road-camera videos; all 213
  files were SHA256-verified against the vehicle originals. Raw recordings,
  frames and decoded data remain private under
  `.analysis/scratch/2026-10-18-signal-drive`.
- Separate long parked/diagnostic recordings from actual drives: October 17
  approximately 17:51–17:59 (`10ea`), October 18 approximately 08:14–08:24
  (`10eb`). The former is evening/twilight, not a fully dark night test.
- Vehicle and both drive initData records identify `carrot-nav/087f862d81`.
  Current mode is 2. Six live signal/planner/display source hashes match the
  installed Git blobs. Both drives record observer SHA256
  `918654b8c3d2bf60640419f5433e87c414a77efa4a875ae744f5ef7944c9d46d`.
- Tracker, worker, stopped guard, moving preparation and display have no Git
  changes between `2ef833b201` and `087f862d81`. CentralSignalSelector and
  UpperSignalGroupTrial remain offline tools, with no production imports.
- Reviewed 15 video overviews (12 samples each), then finer crops around two
  red-to-green transitions and one missed-red approach. This is targeted
  diagnosis, not a fully labelled precision/recall evaluation of every frame.
  Observer images are aligned with `roadEncodeIdx.frameId -> segmentId`.

## Morning: visible green, model response, auxiliary hold remains

`10eb/2`, around 08:17:02: both forward housings visibly change from red to
green, including a round green and left arrow. The reviewed 250 ms video
samples bracket the transition between approximately 08:17:01.76 and 02.02.

The observer already reports current green at capture 08:17:02.022 and a
confirmed green track at 02.263. Nevertheless, the actual guard remains
`hold=True, reason=unconfirmed, green_confirmed=False` through 04.297. The
driver presses gas at 04.390, about 2.4–2.6 seconds after the visible change;
the following guard event is disabled, not a confirmed-green release.

The planner's reported model stopping distance rises from about 1.35 m to
48.9 m during that interval. Thus this case cannot be described as merely the
driving model failing to react: the auxiliary veto is demonstrably still held.
The distance increase alone is not proof of what a closed-loop controller
would have done with the veto removed.

Two image/identity defects are visible:

1. Left housing: red reference `[718,283,738,293]`, ID784, is followed by green
   proposals such as `[723,282,747,294]`, ID789. Other frames use a smaller
   `[721,284,737,292]` box. Raw green is present, but the fixed reference's
   overlap/identity match alternates between green, unknown and no match.
   Its continuous green confirmation keeps resetting. A stopped-guard
   diagnostic with recorded arrivals reproduces no release before gas.
   This diagnostic fixes stopped context and is not a full planner replay.
2. Right housing: its red ID296 disappears at the transition. In encoded frame
   780, the arrow and round green form a 15x8-pixel connected component.
   `day_proposals` rejects the 1.875 aspect ratio against its 1.7 maximum.
   The original red box is not a reliable whole-housing reference either.
   New right-green detection appears after vehicle motion starts.

These are failures to preserve/recognize the same housing across a real change,
not evidence that arbitrary peripheral greens should release a stopped car.

## Evening: missed approach and a successful transition

- `10ea/3`, approximately 17:54:54.5: two large, clearly visible red lamps with
  left arrows have no corresponding current red detection. On encoded frame
  1099, cold image diagnostics return no day/housing/night proposal. The scene
  is classified as non-dark, so the night extension does not run. One near-red
  halo is fragmented and fails the compact component area-fill gate; the
  farther red candidate fails the dark-bar contrast gate. This single encoded
  frame is supporting diagnosis, not an exact raw-NV12 replay of the full
  tracker. No detection-height selection is involved.
- `10ea/5`: a real red-to-green transition occurs around 17:56:28.35–28.60.
  The guard records `green_transition` at 29.840, about 1.24–1.49 seconds later.
  The effective lead-follow exception already removed the veto at 28.594;
  therefore guard confirmation latency is not the vehicle's departure delay.
- Peripheral false proposals remain visible, e.g. morning trees/structure near
  08:17:48 and 08:19:58. A globally higher or brighter candidate is not itself
  a trustworthy signal or permission to depart.

## Cadence and replay

Evening segment averages are 9.29–12.87 Hz (5,352 observations). Morning segments
0–9 average 7.70–12.07 Hz (5,503 observations, including garage/arrival portions).
The morning stopped segment is 8.10 Hz; the failed transition segment is 8.72 Hz.
These are measured observer rates, not camera/DM rates or a controlled CPU
benchmark. No 20 Hz recognition claim is supported. More speed alone will not
repair the demonstrated proposal/identity failures.

Re-running the existing conservative central selector on all 20 new full drive
segments, including two parked tail segments, gives 12,492 observations and
only seven excluded green boxes. Paired stopped-guard replay has zero decision
changes in 20,955 nominal and 20,951 +20 ms updates, one release per variant.
It still does not fix the morning failure. The unsafe all-colour variant would
remove 18 current-red observations and remains rejected. These counts are not
labelled false-positive/false-negative rates. Replay cold-starts each segment
and does not validate MPC, moving preparation or closed-loop driving.

Neither actual drive records a moving-preparation entry. Lead/context gating
and missing qualified red evidence must be distinguished; this observation
does not establish that every visible red should have triggered preparation.

## Direction proposed by the user

Remember several forward-central red housings while stopped. Treat a strong
red-to-green change in one of those *same housings* as a transition candidate.
“Strong” should combine stable housing correspondence, extinction of its red,
fresh round-green evidence and temporal support, not brightness alone. Separate
arrow/round-light evidence, deduplicate fits to one housing, and maintain the
housing through lamp switching instead of redefining its size from each core.
Nearby signals and trees must not inherit the red history by proximity alone.

Current code already attempts central red-to-green release, but its box matching
does not reliably implement that intent. A remaining conflicting red must be
assigned to a different housing/group before being discarded. A validated
transition would clear only the auxiliary veto; ordinary model/lead/driver
departure conditions would still decide motion. No new rule was installed or
declared validated in this review. The morning failure is a concrete regression
case for the next housing-continuity experiment.
