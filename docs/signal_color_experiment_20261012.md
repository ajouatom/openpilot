# Signal color correction experiment — 2026-10-12

The user requested starting a correction experiment after the morning false-green
investigation. A narrower green-color candidate is implemented and replayed, but
is **offline-only and default OFF**. No vehicle deployment, ONNX retraining,
model x/v change, DM/backend change or control-policy change was made.

## Change

`SignalTracker(conservative_green=True)` limits OpenCV HSV green hue to 38–90
instead of 38–105. Saturation/value, connected-component shape and positional
scoring remain. The same option reaches housing proposals, daytime bright cores,
nighttime halos and causal template tracking. Pure cyan remains eligible;
blue-dominant pixels no longer supply green evidence. No old green is substituted
for an uncertain current observation.

This is a hand-written image-processing experiment, not neural-network training
or a complete signal-housing detector. Narrowing the green mask also changes
proposal competition and can reveal red evidence previously suppressed by mixed
red/blue scores; it does not guarantee identical red detections. Actual road
relevance, arrow interpretation and complete housing alignment remain unresolved.
Road-edge filtering was not added: the preceding probe rejected only 8/62 selected
backgrounds and neither red-signal miscrop.

## Same-input replay

`tools/signal_analysis/replay_color_experiment.py` runs both full causal trackers
on identical decoded HEVC images at the recorded observer capture cadence, with
robust tracking/daytime cores enabled. Exact `roadEncodeIdx` frame IDs join the
images; a delayed observation logged in segment 8 correctly maps to segment 7's
last image. Adjacent available segments preserve tracker history. Missing video
segments reset it. All 3,348 observations map; none are silently approximated.

Sources are the 11 morning 109f segments and nighttime 109d-6/109e-3. The other
collected segments were not replayed because this experiment uses locally retained
full-resolution videos, not the smaller qcamera preview.

| Paired replay | Existing | Candidate |
|---|---:|---:|
| Morning fresh green candidate samples, 2,727 observations | 5,608 | 581 |
| Morning observations containing any fresh green | 1,857 | 342 |
| Morning fresh red candidate samples | 610 | 931 |
| Night fresh green candidate samples, 621 observations | 437 | 419 |
| Night fresh red candidate samples | 203 | 203 |
| Desktop morning mean processing time, ms/image | 9.45 | 7.51 |
| Desktop night mean processing time, ms/image | 10.50 | 10.47 |

The 89.6% morning reduction is **candidate count reduction, not measured false
positive reduction or accuracy**. Repeated objects count repeatedly. Live raw
camera results previously had 6,792 morning green candidates; encoded-image replay
does not reproduce those pixels or detections exactly. Desktop timings use one
OpenCV thread and alternate execution order; they do not establish C4 speed.

## Visual checks and known failures

On the previous deliberately selected 73 candidate boxes:

- 61 background boxes that replayed green become unknown; the remaining background
  was already unknown in the encoded image. This is a development set.
- Six correct red and three correct green boxes retain their colors.
- Both logged red-signal miscrops produce unknown with the candidate. Only
  `f10-0020` reproduces the erroneous green in the old encoded-image scorer;
  `f6-0768` already scores unknown there. Do not claim two independently reproduced
  failures were fixed. The full candidate replay emits no green in segment 6.

After freezing the threshold, 21 additional frames were selected to inspect
retained greens and nighttime removals. Among their old green candidates,
37 visible green-lamp examples remain detected and 29/36 backgrounds are removed.
Seven backgrounds survive: five LED-text samples and two tree/window-reflection
samples. This selection is neither random nor a complete recall annotation.
Green-lamp presence is not proof that the lamp authorizes the ego vehicle to go.

All 18 nighttime removed candidates were additionally inspected in crops and
scene context. One is a **real motion-blurred green signal**, 109e-3 encoded frame
635 while turning. Thus the change has a known sensitivity regression. Other
reviewed sharp near/far nighttime greens remain visible; this does not establish
robustness across all exposure/white-balance conditions.

The unchanged start guard's `observe()` color-confirmation diagnostic was also fed
the replay tracks using the existing three-observation publisher rule and recorded
result age. In morning segment 8 the candidate first confirms green at 21.090 s;
the encoded baseline never confirms it in this probe. The actual live guard had
released at 18.106 s. This diagnostic excludes engagement/validity/vehicle-response
simulation and cannot validate departure timing or show a 3 s causal regression.
It does demonstrate that encoded replay is not an exact live-control reconstruction.

## Checks and disposition

84 focused tests pass, covering existing tracker/guard behavior, blue rejection,
green/cyan and saturated night lamps, causal-template filtering, red-to-green
confirmation, opt-in default and cross-segment frame alignment. The initial test
invocation hit the unavailable Windows native Params extension; tests then ran
with `--noconftest`, without changing production dependencies. Default-OFF output
matches the previously committed tracker exactly on 21 reviewed images.

This is a useful sky-rejection experiment with measured limits, **not a promoted
vehicle fix**. Keep the option off in live workers. Next work should discriminate
actual housings from LED text/reflections, preserve lamp alignment from the red
anchor, and test motion-blurred/cyan greens without re-admitting sky. Keep display
memory separate from fresh control evidence and retain the existing DM GPU guard.

Private evidence: `.analysis/archive/2026-10-12-signal-color-experiment/` contains
paired traces, labels, screenshots, totals and reproduction scripts. Raw drives,
captures and labels remain local. The preceding collection and investigation are
documented in [the morning report](signal_morning_false_green_20261012.md).
