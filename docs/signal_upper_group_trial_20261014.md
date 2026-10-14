# Upper signal grouping and neural comparison

The requested upper-group/lower-rear-light correction was implemented and
replayed. It works when supplied qualified red-to-green observations, but does
not fix the Santa Fe departure failure through the actual observation pipeline.
The candidate therefore remains an **offline experiment**, with no import from
the live worker or planner. Pulling this change does not alter vehicle control.

## Grouping experiment

`tools/signal_analysis/signal_upper_group_trial.py` subclasses the production
guard for comparison. It preserves the existing small/lower exception and adds
a second, provisional image-row exception: a lower red box may be up to twice
the smaller upper box's width/height, but its top must be at least six times the
largest relevant box height below both upper boxes. These ratios are heuristic
development choices, not measured mounting heights or validated lane assignment.

Release still requires two disjoint, comparable, same-row upper objects, each
with its own established red history and three fresh green observations spanning
700ms. A single upper green, new unanchored greens, same-row red, duplicate boxes,
stale input and restarts cannot use this exception. No freshness, producer
confirmation, driver override, acceleration, DM or model policy was changed.

Tests use the reviewed Santa Fe coordinates: upper reds [613,131,653,151] and
[763,120,807,142], subsequent greens [617,128,661,150] and [772,119,812,139], and
right-car rear-light candidate [929,350,977,374]. Supplied qualified observations
now allow the pair to exclude that lower candidate. This synthetic observation
test is explicitly not proof that image recognition produces those observations.

## Recorded-input verification

- Re-ran 43,090 historical planner inputs at nominal and +20ms observer arrival
  delay, with both the baseline and candidate guards. Every output trace is
  identical. Existing six release events are unchanged; reviewed-red releases
  remain zero across 8,744 active reviewed-red samples per timing variant.
  The known 1091-20 acquisition failure remains: this is not complete protection.
- Compared the latest 19 Ioniq segments and Santa Fe segment 135-3: 9,931 observer
  results and 22,446 nominal guard updates. Nominal/+20ms/+100ms comparisons give
  no decision changes and no new green release. The replay preserves capture and
  arrival times and checks available input validity/age; it is not closed-loop
  motion simulation or a full reconstruction of every runtime health gate.
- Applied both guards to the cached baseline image replay of all 165 matched
  Santa Fe encoded frames. Nominal/+20ms/+100ms still give zero releases.
  This reuse is valid because the experiment changes only guard grouping; the
  detector source remains identical. Encoded images are not bit-exact VisionIPC.

The blocker precedes grouping: approximately 2.79Hz observation cadence and
lamp-dependent boxes repeatedly break producer identity. Green proposals usually
have only one observation and become unknown in transport. The second overhead
lamp is also intermittently absent. An upper/lower exception cannot manufacture
the missing qualified green pair. The production guard was restored unchanged.

## Fresh comparison with the existing pretrained neural models

Used the hash-pinned YOLOX-s detector and MobileNet classifier already supported
by `neural_signal.py`. No weights were fitted or exported in this work. Detector
and classifier scores are saved separately; classification must not overwrite
detector confidence.

| Evaluation set and metric | Current colour/shape rules | Neural upper ROI | Neural two overlapping tiles |
| --- | ---: | ---: | ---: |
| 73 previously labelled images with at least one box at IoU >=0.5 | 47 | 68 | 70 |
| 37 selected real green lamp cores covered and classified green, from 21 images | Not rerun for this metric | 10 | 19 |
| 36 selected background cores covered by a neural detection, same 21 images | Not rerun for this metric | 0 | 0 |

The rules comparison uses current conservative-green proposals, hence differs
from the older 49/73 result with the wider hue range. The 73-image measure is
per-image coverage of at least one labelled object, not recall of every signal
or precision. The small-core labels mark lamp pixels rather than full housings:
the second/third rows use centre coverage, not IoU. Neural upper ROI covers 11/37
real cores, of which 10 classify green; tiles cover 21/37, of which 19 classify
green. These selected, correlated frames are not an independent population test.
Zero matched backgrounds does not mean zero other false positives in the image.

On all 165 Santa Fe sampled frames, rules emit lower-row red candidates in 108
frames (158 boxes); the neural ROI emits none. Restricted to the stopped guard's
front ROI, the counts are 87 frames/102 boxes versus zero. Counts include fresh
unconfirmed proposals and are not counts of actual guard interventions. The
reviewed right-car candidate is a rear lamp; the entire lower-row population has
not been independently labelled. Do not call this a measured false-positive rate.

The neural boxes still move with the illuminated lamp and sometimes duplicate
one physical signal. Injecting fresh neural classifications into the unchanged
guard, using recorded CPU-observer arrivals as a timing diagnostic, also produces
zero releases at nominal/+20ms/+100ms. This is not a measured neural runtime or
an integrated neural tracking/control replay. The PC neural detector/classifier
work time was median 108ms (range 82-144ms), not device performance.

## Engineering conclusion

Colour/shape rules can generate inexpensive proposals, but these recordings
support moving object recognition toward a learned detector and lamp classifier.
Autoware likewise separates a CNN traffic-light detector from crop classification;
its documented detector was fine-tuned on Japanese traffic-light imagery:
https://autowarefoundation.github.io/autoware_universe/main/perception/autoware_traffic_light_fine_detector/

The off-the-shelf models are not a completed replacement: tiling helps small
signals, but misses remain, and housing identity across red/green still fails.
The next training target should be stable whole signal housings and lamp states
on the existing local footage, with rear lamps, signs, sky and pedestrian signals
as explicit competing/background examples. Keep encounters and adjacent frames
together when splitting training/validation/test data. This is a proposed
auxiliary-detector fine-tune, not retraining the original driving ONNX's x/v.

Validate false-green transitions, true-green release latency, small/distant
signal coverage and native timing/DM co-load separately before control promotion.
More driving is not needed to reproduce the present failure. Labels and model
adaptation, rather than another unvalidated control exception, are the next work.

## Reproduction and publication

630 related desktop tests pass with the Windows Params stub/OpenCV environment.
The added tests check the offline candidate and recorded-input comparison, not
perception accuracy. Existing NumPy reshape deprecation warnings remain.

The comparison CLI accepts private JSONL rows containing `now`, `observation`,
and the exact `context` keyword arguments for `SignalStartGuard.update`:

```sh
python -m tools.signal_analysis.signal_upper_group_trial inputs.jsonl comparison.json
```

Private replay scripts, paired traces, input rows, model outputs and reviewed
metrics are retained in `.analysis/scratch/2026-10-14-upper-signal-group/`.
Only the offline experiment, tests and findings are published to carrot-nav and
carrot-wip with single-parent commits. No vehicle installation, model change or
live activation is included.
