# Signal observation dropout: October 12 midday drives

## Scope and sources

At the user's request, collect and inspect the previously uncollected Ioniq 5 PE
routes `000010a0--b48f187a6a` segments 0-5 and `000010a1--0dff1e551a` segments
0-4. These start at approximately 12:33:34 and 12:47:49 KST. All eleven completed
segments ran carrot-wip `6d7fede8a1`. Current read-only device inventory confirms
SignalAssistMode=2, IsOnroad=0 and IsEngaged=0. No vehicle code, settings, camera
exposure, deployment or restart was changed for this investigation.

All 44 rlog/qlog/qcamera/fcamera files (509,994,025 bytes) have matching PC and
device SHA256 values. Private recordings, extraction scripts, exact-frame
reviews and replay evidence are retained locally under
`.analysis/archive/2026-10-12-signal-latest/`. Do not publish recordings or
settings captures. Full logs contain 2,716 daytime observer records, of which
292 exceed the producer's 200 ms result deadline. There are no active
signalStopPrepare transitions in either route; this is not a detector recall
score and does not imply all road context was eligible for preparation.

## Confirmed temporal failure

In 10a1 segment 3, the car approaches and waits behind a black sedan at an
intersection with two visible red signals. Exact roadEncodeIdx/frameId joins
align observer records to the original front-camera HEVC frames.

- Observer rate: 3.347 Hz, versus the road camera's approximately 20 Hz.
- 185 of 200 adjacent observation intervals exceed 250 ms.
- Capture-to-daytime-result median 203.440 ms; p95 244.956 ms.
- 114 of 201 records (56.7%) exceed the 200 ms result deadline.
- 103 records have at least one fresh raw-red candidate, but none has a
  confirmed red track or red aggregate prediction.
- The 127 fresh raw-red track observations contain 121 observation-count=1
  and six observation-count=2 instances. None reaches three observations.

The running algorithm hash is
`fc47191dce5bbd731053d067efb39e54fd481b14ed20a3bdeed7da9583869b81`, with
daytime comparison and revalidated tracking enabled. Its robust mode retains
established object identity up to 500 ms, but resets color confirmation after
250 ms. More critically, an object without confirmed red history expires after
250 ms. At this recorded cadence, red acquisition repeatedly starts over.
Start-guard publishing suppresses raw colors until observations>=3, explaining
the unconfirmed/unlatched guard in this sequence as well. Displayed raw candidate
boxes are not confirmed red/green permissions.

The worker runs both the legacy baseline and the daytime tracker sequentially
on every selected image. Segment 3 median baseline work is 82.339 ms; daytime
work is 71.235 ms; paired work is 156.194 ms. Its following sleep targets at most
50% of one CPU, further reducing cadence. These are measured live timings;
removing the baseline has not yet been measured on the vehicle and cannot be
claimed to guarantee a particular frequency.

Frames 300 and 700 contain raw-red boxes, but the tracks have only one or two
observations and the results arrive at 208.088 and 224.681 ms respectively.
Frame 901 has raw red with count=1 and 206.952 ms age. These failures occur
before stop-preparation geometry can help. The segment also has a lead in
475/485 sampled radar records: the existing lead exclusion is an independent
reason not to expect moving stop preparation in this particular approach.

## Image evidence and independent color problem

Images show actual red lamp points clearly in some frames, while the same
housings are nearly dark in others, including frame 499. Consecutive crops and
the longer sequence retain this observation without claiming a physical light
state for every dark exposure. Camera/LED sampling is a plausible explanation,
not a measured
signal PWM frequency or established camera fault. The user subsequently confirmed
that it was a continuously illuminated normal red signal to the eye, not a
flashing red. Thus the dark recorded intervals are not evidence of actual
traffic-signal transitions. LED/camera aliasing remains the leading hypothesis,
without a measured emitter waveform or uncompressed sensor capture. Exposure examples report
integLines 13-14; no unverified conversion to exposure duration is made.

The six-frame overview sheets are diagnostic snapshots of fresh raw candidates
at roughly ten-second spacing. They do not reproduce the actual UI's two-second
display memory, so gaps between sheet boxes alone cannot establish UI flicker.
Other exact frames show raw-green boxes on tree/sky boundaries, a pole and a
road sign. These are false *candidates/display evidence*; the recorded 10a1
aggregate never confirms green and no start-guard release is logged.

## Offline replay, unchanged production thresholds

Run the current daytime robust tracker on the original segment-3 HEVC, using
exact road-camera EOF times. Change only which recorded frames are processed.
The additional conservative-green variant is the existing offline option,
not a vehicle modification. Each replay starts without earlier-segment history.

| Sampling | Frames | Aggregate red | Aggregate green | Unknown |
| --- | ---: | ---: | ---: | ---: |
| Recorded observer frame IDs | 201 | 0 | 0 | 201 |
| Every fourth camera frame (~5 Hz) | 299 | 160 | 3 | 136 |
| Every second camera frame (~10 Hz) | 599 | 336 | 43 | 220 |
| ~10 Hz, conservative green | 599 | 360 | 0 | 239 |

This reproduces the temporal failure and demonstrates that the video contains
usable red evidence. It is not an accuracy percentage: rows have different
sample counts, not all frames have ground-truth labels, and HEVC color differs
from the worker's raw NV12 conversion. Live raw-red records=103 versus recorded
frame replay raw-red records=106 illustrates that difference. PC processing
times do not predict device throughput or closed-loop behavior.

Faster sampling alone is not suitable for promotion. The 10 Hz replay produces
green from a causal template at frame 210 and nearby dark-lamp frames, before
the later visibly green transition. Conversely, the conservative option loses
all green confirmations, including the visibly green late part around frame
1101. Thus neither speed alone nor simply tightening the existing hue threshold
is a validated solution. No control outputs are evaluated by these tracker-only
replays, and no false-start or successful braking claim follows from them.

## Next bounded experiment and optical-filter conclusion

Prioritize one selected detector per image in normal mode, keeping dual
comparisons explicit/offline. Measure actual capture age and cadence with the
driving model and DM running. Keep object/previous-red memory separate from
fresh green permission, and require evidence of the illuminated green lamp
inside the tracked housing. Validate both early dark-lamp false green and late
real-green misses on these clips plus the retained night/morning examples before
changing live behavior. Do not widen control freshness just to hide overload.

No CPL or ND filter has been tested. A polarizer can reduce certain reflections
but attenuates light; it cannot correct the demonstrated software timing and
confirmation mismatch. Do not infer that an optical filter restores missing
lamp samples. See [Edmund Optics' polarization guidance](https://www.edmundoptics.com/knowledge-center/application-notes/illumination/successful-light-polarization-techniques/)
for reflection reduction and light loss, and [onsemi's automotive LED sampling
explanation](https://www.onsemi.com/company/newsroom/blog/automotive/autonomous-driving-imaging-in-lfm-and-hdr-situations-on-the-road)
for why short camera exposures may miss an apparently illuminated LED. These
general mechanisms do not establish this signal's exact physical behavior.

Docs-Not-Needed: read-only incident analysis and private offline replay; no user
setting, production behavior or vehicle deployment changed.

## Requested correction and replay verification

The user subsequently requested correction, commit and push on carrot-wip.
The production CPU observer now runs one daytime/robust tracker with
`verified_lamps=True`; it no longer runs the legacy tracker first. Separate
experiment directories retain explicit two-engine comparisons. Production logs
use `signalTrackingShadow` with `single_tracker=True` and `verified_lamps=True`;
the loaded event carries the same fields. `daytime_comparison=True` continues
to mark the isolated helper path: legacy moving-assist publication remains
disabled, while the existing display, stopped-red veto and stop-preparation
publishers receive the single selected result. No extra detector thread or GPU
work is introduced. Mode0/1/2 and the DM-aware neural backend selection remain.

Within that tracker, fresh red detections can accumulate through at most the
existing 500 ms identity interval. Gaps over 250 ms require the existing tight,
unique geometric reacquisition or verified template correspondence. Long gaps,
reordered time and incompatible housings still discard identity. Unknown images
do not count as red observations. Confirmed red history can remain with the
image-tracked object through a dark exposure, but its *current* color becomes
unknown; no stale red is fabricated for moving preparation. Existing stopped
red latching and bounded display memory remain separate.

Green still requires current evidence and its original confirmation interval.
A new image check requires a connected green/cyan component inside the right
lamp, with green intensity and chromatic contrast above the unlit housing.
Weak blue-sky color and housing-border pixels cannot supply it. All proposal
sources, including templates, use the same check. Cyan with a strong color core
is retained. This is still an empirical lamp-layout check, not semantic object,
arrow or lane recognition. Control freshness deadlines and braking limits are
unchanged; increasing red acquisition continuity does not extend result age.

The final paired replay uses the pre-change committed tracker as the baseline.
Four retained HEVC segments, exact camera times and recorded frame IDs are used.
For 10a1-3, extra 5/10 Hz frame schedules isolate cadence effects:

| Segment / sample schedule | Before confirmed red | After confirmed red | Before / after aggregate green |
| --- | ---: | ---: | ---: |
| 10a1-3 / recorded, 201 frames | 0 | 104 | 0 / 0 |
| 10a1-3 / 5 Hz, 299 frames | 160 | 160 | 3 / 0 |
| 10a1-3 / 10 Hz, 599 frames | 336 | 343 | 43 / 0 |
| Morning 109f-6 / recorded, 282 frames | 1 | 3 | 0 / 0 |
| Morning 109f-8 / recorded, 233 frames | 19 | 46 | 1 / 1 |
| Night 109e-3 / recorded, 265 frames | 0 | 0 | 0 / 0 |

The 19 premature aggregate-green samples at 10a1-3 indices210-254 in the 10 Hz
baseline disappear. However, the faint late green-looking lamps in this segment
still do not yield a confirmed green in the new replay. This remains a possible
late/non-release case, not a demonstrated correct departure. Night raw-green
candidates remain available; aggregate green requires preceding red history and
is not a detector recall measure when starting mid-sequence.

Two previously reviewed crop sets contain 140 old-green candidate boxes. The
new current-image check retains all40 visually labeled green-lamp boxes, rejects
both known red-signal wrong-crop cases, and rejects90/98 background boxes.
Eight background candidates still pass. The sets are selected development
examples, not held-out accuracy or proof of zero false green. Stored compressed
pixels also differ from original live NV12, so these are offline measurements.

381 focused tests pass, including delayed red acquisition, tight identity
matching, expiry, dark images followed by illuminated green, sky/border rejection,
green-without-red rejection, single production processing, transport isolation,
mode switching, existing start veto and final preparation acceleration limits.
Windows substitutes native Params storage only; no physical driving response or
onroad CPU/DM timing is established. Actual device rate after removing paired
work remains to be measured. No vehicle installation or restart was performed.

Private reproducible evidence is retained in
`.analysis/archive/2026-10-12-signal-improve/`; recordings remain in their prior
archives. No new ONNX or driving-model weights are created by this correction.
