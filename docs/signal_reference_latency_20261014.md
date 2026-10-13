# Signal reference-point experiment and release latency (2026-10-14)

The user proposed an expected signal 30m ahead while stopped, 70m ahead while
moving, centred 5m above the model's road surface. They also reported that
waiting after green confirmation makes the system slower than manual departure.

## Reference-point experiment: offline only

`tools/signal_analysis/signal_reference_geometry.py` projects those assumptions
through the existing calibrated camera transform. In the model's down-positive
Z convention, the point is `(distance, centre_y, road_z - height)`. Inner-lane
and road-edge midpoints are evaluated separately: road centre is not necessarily
ego-lane centre. Curves are never extrapolated. Low-confidence curves may give
a diagnostic point but keep `reliable=False`; ranking never grants control.

The nearest candidate is ranked by its transverse displacement after projecting
its image ray onto the ASSUMED 30m/70m plane. This is not measured signal range.
The trial marks offsets <=3m as near; this experimental cutoff is not validated
for control. Stale display boxes are excluded. There is no production import,
new setting, or live start/stop connection to this ranking experiment.

Seven previously collected segments supply 3,020 valid-context observations:
1,615 stopped and 1,405 non-stopped (30m and 70m respectively). Two reviewed
stopped examples demonstrate useful ranking:

| Reviewed frame | Source | Nearest real overhead candidates (pixel offsets) | Distractors (pixel offsets) |
| --- | --- | --- | --- |
| 10b7-1, 2211 | inner lanes | 55: 65.7; 54: 87.7 | lower reds 197: 134.7; 198: 146.8 |
| 10b8-2, 2449 | inner lanes | 317: 50.4; 167: 72.2 | right green 353: 391.4 |

On the assumed 30m plane, the first case's real lights are 1.73/2.31m from the
reference and lower reds 3.54/3.86m away. In the second, real lights are
1.33/1.90m away and the right candidate 10.46m away. These offsets describe
ranking relative to a hypothetical reference, not physical signal distances.
Road-edge midpoint ranking swaps the two real overhead lights in each scene,
but still ranks them ahead of those distractors.

All 1,615 stopped projections fail the existing curve confidence thresholds;
for moving observations only 184 lane references pass them, and zero edge
references do. This means rigid confidence gating would discard the real
stopped lights as well. Conversely ignoring confidence would overstate the
point's accuracy. The two reviewed examples support testing a region/priority
around the reference, not claiming precision/recall or direct green permission.
The 70m moving ranking has been computed, not labelled as correct across driving.

A useful next implementation would retain an image-tracked signal group selected
during approach, use a bounded region around the reference rather than one exact
point, and distinguish low-confidence geometry from proven out-of-range objects.
Nearest-object selection alone neither identifies a pedestrian light nor proves
which lane a light controls. Do not let the selected object jump at a color change.

## Measured cadence

From the latest collected drive, not a new native performance benchmark:

| Segment group | Observations/sec | Typical capture interval | Typical result age |
| --- | --- | --- | --- |
| 10b7 evening stop | 6.26-6.43 | 150ms | 131-132ms |
| 10b8 morning | 7.49-10.59 | 100-150ms | 93-118ms |

95th-percentile capture intervals are roughly 152-203ms; isolated gaps are
larger. The worker's 20Hz setting is a ceiling, not achieved cadence. It scans
broad image regions and sleeps to target half of one CPU, preserving the prior
DM protection. A narrow tracked-signal update could be faster than full search;
no new target-device timing or GPU co-load result is claimed here.

## Production change: remove the redundant post-confirmation wait

The start guard repeatedly rearmed the planner's 0.5s stopping timer while red
was held. After >=700ms green confirmation and release, the planner counted that
timer down again, at the actual planner cadence. Now only an actual release
with `xState=e2eStopped`, `carState.standstill=True`, and `abs(vEgo)<0.1m/s` clears
that timer, inside the existing eligible green-input branch. All other native
stops retain their timer. Driver hold/brake/blinkers/manual stop, navigation
stay-stop, mode and validity gates remain. Green confirmation itself is unchanged.
There is no direct acceleration command, inferred go from unknown, or forced
soft-hold release.

A/B replay of 42,064 variant/input records, at nominal/+20ms/+100ms arrivals:

| Reviewed stop | Before: first raw-green capture to positive cruise target | After | Release to positive target after change |
| --- | --- | --- | --- |
| 10b7 evening | 1.508s | 1.007s | 48ms |
| 10b8 morning | 1.610s | 0.960s | 51ms |

The state becomes e2eCruise on the release update; the existing stopped branch
keeps the cruise target zero for that one update. At +100ms added latency, the
two totals are 1.107/1.116s. These are state-machine/target timings from recorded
inputs, not actual pedal-to-wheel movement or a closed-loop acceleration test.
They also exclude any time between the physical lamp switch and its first
sampled raw-green detection.

776 related tests pass on each maintained branch, including reference projection
orientation/no-extrapolation/uncertainty, and immediate release vs moving or
non-standstill context. Raw evidence remains private under
`.analysis/scratch/2026-10-14-signal-transition/` (reference_experiment.json,
cadence.json, timing_replay.json, timing_outcome.json).

Publish the common timer fix and offline experiment to carrot-nav and carrot-wip
using single-parent commits. No vehicle files/settings or ONNX were changed;
reference ranking is not installed into control, and physical validation remains.
