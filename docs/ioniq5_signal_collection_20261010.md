# Ioniq 5 PE signal corpus and replay, 2026-10-10

## Completed scope

The user requested collecting the logs uploaded today together with yesterday's
material and analyzing signal response. The target for offline experiments
remains the small internal ONNX. No production model, signal-start threshold,
vehicle setting or control behavior was modified.

All directories dated October 10 in the exact vehicle NAS folder were inventoried.
All available rlogs and road/wide HEVC streams were copied locally, with SHA256,
source size/mtime and before/after-copy stability checks. No source changed
during its copy. Upload date is not capture date: the new batch includes October 9
evening/night recordings and October 10 morning recordings.

| New route | Nonempty rlog segments | Decodable carState span | Camera originals | Actual recorded model |
|---|---:|---:|---|---|
| 1081 / 979f280eb3 | 12 | 613.05 s | None | Cinque v3 eGPU |
| 1083 / 47f5837c74 | 6 | 347.40 s | Both | Cinque v3 eGPU |
| 1084 / 614a105483 | 12 | 695.72 s | Both | Cinque v3 eGPU |
| 1085 / af228db460 | 4 | 193.52 s | Both | No modelV2 published |
| 1086 / dddc892236 | 8 | 425.13 s | Both | eGPU initially, then internal |

Totals for today's upload batch:

- 42 rlog segments, approximately 37.9 minutes of decodable carState spans.
- 30 segments with both camera originals, approximately 27.7 minutes of
  corresponding decodable log spans.
- 39,749 modelV2 messages, all marked valid in the log. Validity does not imply
  correct perception.
- Three empty directories: 1081--0/1/2.
- Three truncated rlogs: 1081--3 (3.19 s usable prefix), 1081--14 (9.44 s),
  1085--3 (13.51 s). Preserve their valid prefixes and mark the truncation.
- Some source `.lock` files remained. File stability checks passed; this does
  not turn a truncated stream into a complete one.

The combined catalog also references the previously collected 1079--1/2/3/4
and 107c--0, 107e--1/2 without duplicating their source files. It contains
49 nonempty rlog segments, including 33 with both HEVC streams. A segment is
not an independent signal encounter; some waits span consecutive segments.

## Model identity and startup exclusions

Runtime reports identify the eGPU artifact as
`e758b96df27858ea97122d18554930d04f9f8bda417417074edfb3a72b008d0b`, Cinque v3.
Capture commits are `0686c424` for 1081 and `7999cf05` for the other new routes.

1083--0 first publishes modelV2 at approximately 28.02 s; 1084--0 at 30.18 s.
Their earlier camera frames must not inherit later model values in visual
overlays. The generated contact sheets explicitly mark this absence.

1085 has no modelV2 in any of its four segments. This is a startup/runtime
availability problem, not evidence of a particular incorrect signal forecast.
Its video can still support annotation and offline inference. The cause of the
prolonged startup was not established in this signal investigation.

1086--0 first publishes at 32.87 s. At boot monotonic 98.859 s the log explicitly
reports `precompiled eGPU worker timed out` and fallback to the internal GPU.
Later reports are `ModelState, usbgpu=False`; segments 1–7 use the internal model.
Do not pool these predictions with Cinque predictions under one model name.

## Review method

All 42 rlogs were decoded with the full OpenPilot cereal schema. Model trajectories,
car state, planner state, actuator commands, radar leads, calibration, frame IDs
and relevant runtime diagnostics were extracted.

All 30 road HEVC streams were decoded sequentially, with contact-sheet samples
every five seconds. This is a first-pass review of every supplied video segment,
not a claim that every video frame has been manually labeled. Selected incidents
were inspected at 0.25–1 second spacing in full-resolution source frames.

Model `x(+10 s)` and `v(+10 s)` are trajectory outputs, not direct probabilities
of a red or green light. The planner's trafficState is derived from those outputs.
All phase observations below are based on video, not that planner label.

## Confirmed night red-light departure: 1084--3

The near overhead signal remains red throughout the detailed 40.0–44.5 s
camera sequence. There is no leading car in front during departure. The same
lights only turn green later, between rlog-relative camera EOF times 57.987 and
58.239 s. The camera observations therefore distinguish the early false
continuation from the later actual green transition.

| Time relative to first carState | Evidence |
|---:|---|
| 35–41 s | Stationary red wait; recorded Cinque mean v(+10 s) approximately 3.34 m/s, with repeated rises |
| 41.316 s | Model x(+10 s)=22.625 m, v(+10 s)=7.148 m/s while ego is stationary |
| 41.323 s | Planner changes trafficState to green and xState to cruise |
| 41.396 s | Commanded acceleration approximately +0.65 m/s² |
| 42.199 s | Commanded acceleration approximately +2.26 m/s² |
| 42.289 s | Measured speed exceeds 0.1 m/s |
| 42.554 s | Brake input appears at approximately 2.23 km/h |
| 43.037 s | Speed peaks at 4.08 km/h before stopping again |
| 58.360 s | Planner reports green after the later actual green transition |

No accelerator input is recorded during the early departure window. Soft hold
was released beforehand; the source of that release and driver intent were not
reconstructed. Thus the evidence establishes motion under red following a
continuation forecast, not that no other user input occurred.

The initial model near-horizon velocity is much smaller than its +10 s velocity;
the stronger near-horizon continuation follows later. This should not be
described as an independent network signal classifier directly demanding
acceleration. The model/planner/control sequence is preserved in the evidence.

This night example extends yesterday's findings: a brightly visible red lamp
can coexist with an incorrect continuation forecast. It is not justified to
reduce every failure to the daytime orange/white color ambiguity alone.

## Useful normal and ambiguous examples

- **1084--2, 30–44 s:** red wait with no lead in the selected window. Recorded
  Cinque mean v(+10 s)=0.016 m/s, maximum 0.689 m/s, no derived green frames.
- **1084--7, 38–58 s, continuing into --8, 0–24 s:** stable red wait, mostly
  without a lead. Mean v(+10 s)=0.005 and 0.168 m/s respectively. Keep this
  single encounter together when dividing training and validation data.
- **1083--3 and 1084--5:** stopped vehicles under red, with near-zero model
  predictions. These are useful obstacle/lead controls, but do not isolate
  signal recognition from lead-following behavior.
- **1084--4 around 49–59 s:** green signal with a stopped bus ahead. Do not
  label every green frame as a correct instruction to accelerate.
- **1086--1, 35–49 s:** initial sparse samples suggested green with a stopping
  forecast. Detailed video shows red through 43 s, a transition around
  44–45 s, and a braking lead continuing to slow under green. A lead exists
  throughout the selected window. This is not established as pure signal
  misrecognition; use it as a transition-with-lead case.
- **1086--4, 42–50 s:** stopped pickup; internal-model mean v(+10 s)
  approximately 0.408 m/s and near-zero ego speed. The applicable signal is
  not visible/confirmed in the reviewed frames; use this as a lead-stop control,
  not a confirmed red-signal example.
- **1085--1:** daytime intersection sequence suitable for later signal/arrow
  annotation, but no recorded model prediction. Relevant-light and turn-phase
  labels are still pending; offline outputs must not be called recorded failures.

The corpus includes day/night, small lamps, headlight glare, stationary red waits,
green transitions, lead vehicles, cross traffic and no-signal parking/road scenes.
It does not yet establish broad coverage of rain, fog or all signal types.

## Internal ONNX replay on the new data

Seven segments were replayed: 1084--3/7/8, 1085--0/1, 1086--1/4, totaling
8,396 synchronized camera pairs. The unmodified repository internal ONNX is
60,792,584 bytes, SHA256
`f73a9e535523d5e9acb9e642c64e33d631825dc8ba74123757d107cedd047bb5`.
There is no diff in that model, the warp implementation or compile source
against capture commit 7999cf05.

The implementation follows yesterday's sequential YUV decode, calibrated
nearest-neighbor warp, packed image planes, 20 Hz queue updates and 5 Hz history
sampling. Consecutive selected segments retain history; disconnected selections
start with zero history. Comparisons exclude the first six seconds.
Action-time inputs are approximated as [0.475, 0.375], and absent logged desire
is zero. HEVC compression, timing and device/runtime arithmetic limit fidelity.

Same-internal-model comparisons:

| Segment | Compared frames | Mean absolute v(+10 s) error | Mean absolute x(+10 s) error |
|---|---:|---:|---:|
| 1086--1 | 1,079 | 0.381 m/s | 2.694 m |
| 1086--4 | 1,079 | 0.375 m/s | 2.148 m |

For the red-light departure in 1084--3, the internal model's mean v(+10 s) is
0.893 m/s in 35–41 s versus recorded Cinque's 3.34 m/s. However, the internal
forecast also rises later: first above 5 m/s near 41.814 s, and mean 8.939 m/s
over 41–44 s. It is therefore not a uniformly fixed replacement for this case.
Both forecasts are evaluated on the same recorded camera motion, which includes
the actual vehicle starting to move. This open-loop replay does not establish
whether the internal model would have initiated the same departure in a vehicle.

Internal replay also preserves low stop forecasts on the 1084--7/8 control wait
(means 0.174 and 0.575 m/s). These contrasting examples are more useful for a
future learning experiment than collecting only failures.

No weights were trained and no new model artifact was deployed. The replay
outputs are predictions to evaluate, not automatic training targets.

## Collected dataset and remaining annotation

Private archive: `.analysis/archive/2026-10-10-signal-collection/`.

- `corpus_manifest.json`: unified source inventory referencing yesterday's files.
- `corpus_catalog.csv`: human-readable segment/model/quality list.
- `source_manifest.json`: new source hashes, timestamps and copy stability.
- `case_manifest.json/csv`: 14 reviewed or candidate windows, with signal,
  lighting, lead context and confidence/limitations. All are explicitly marked
  `training_ready=false`; these are not finished per-frame bounding-box labels.
- `review.html`: local contact-sheet/case navigation, including every video segment.
- `internal_replay.csv` and `replay_metrics.json`: internal model experiments.
- Per-segment `decoded.pkl`, `frames.csv`, `summary.json`, original media and
  sampled/detailed images.

Before training, annotate the relevant signal for ego's direction, signal state
and transition, stop line, lead/obstacle context, and appropriate longitudinal
target. Do not train directly on a recorded erroneous red-light departure, or
use green alone as an acceleration target. Group adjacent segments and repeated
visits to the same intersection when making an evaluation split; the suggested
roles in the manifest are provisional, not a finalized independent test set.

See yesterday's investigations in `ioniq5_107ce_internal_signal_20261009.md`
and `ioniq5_1079_signal_model_20261009.md`. All source recordings, settings and
camera imagery remain local and ignored by Git.
