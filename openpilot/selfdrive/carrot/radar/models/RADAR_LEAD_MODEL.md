# Radar lead models: front v18 / corner v20

## Production files

- `radar_lead_front.npz`: front-radar and optional SCC input.
- `radar_lead_corner.npz`: corner-radar input only.
- `radar_lead_multitask.npz`: retained legacy single-model artifact; it is not selected when both source models exist.
- `radar_lead_training_manifest.json`: selected corpus logs and train/validation/test split metadata.
- `../data/radar_lead_annotations_v15.json`: manually reviewed video labels.
- `../../cluster/cutin_validation_cases.json`: maintained production scene regression set.

The PC simulator, `radar_lead_validation_review.py`, `validate_radar_lead_model.py`,
and on-device `radard_model.py` resolve the same two production model paths.

## Runtime policy

The source models and their temporal state are independent:

- The front model runs for every vehicle. `EnableRadarTracks=-1/0` supplies SCC,
  `1` supplies front tracks, and `2/3` supplies front tracks plus low-speed SCC.
- The corner model runs only when supported corner-radar points are present.
- Each model owns a separate feature-history builder and
  `RadarLeadDecisionFilter`; probabilities, hit counters, sticky time, and object
  histories are not shared across sources.
- The corner decision normally uses the temporal model probability together with
  lane and body-entry geometry. A corner-only object within 6 m may use a
  two-frame early path only when its instantaneous, 0.4-second, and 0.6-second
  lane-relative motion all point inward and the base model score is at least
  0.95. A single-frame base score cannot activate a cut-in.
- `leadOne` is selected by matching the first high-probability vision lead to a
  sane front/SCC radar object. Distance, lateral position, and velocity sanity
  checks remain mandatory.
- On vehicles with corner radar, the corner model supplies `leadTwo` cut-in and
  external candidates. Without corner radar, the front model supplies those
  candidates.
- A selected corner candidate is matched back to a front object for control
  kinematics when possible. A near-side object inside 5 m keeps the corner
  measurement because near-field front lateral data is too noisy.
- A slow front object can match a stationary vision lead only when a mature
  corner object corroborates the same distance and lane position. The published
  lead remains the front object.

The feature history contains current data plus observations approximately 0.05,
0.1, 0.2, 0.4, 0.6, 0.8, and 1.0 seconds old (`h1`, `h2`, `h4`, `h8`, `h12`,
`h16`, and `h20` at 20 Hz). Near corner anticipation requires consistent inward
motion at both `h4` and `h8`, a 5-12 m range, and lane/body geometry support.
Objects below 5 m require actual body-entry geometry instead of anticipation.

## Training corpus

- Inventory: 1,215 readable logs under `W:\routes`.
- Selected: 180 logs across 56 vehicle folders.
- Train: 124 logs, 146,556 frames, 1,890,692 candidate rows.
- Validation: 25 logs, 29,817 frames, 351,208 candidate rows.
- Test: 30 materialized logs, 34,859 frames, 412,219 candidate rows.
- Video review: 35 source routes, with raw-CAN corner points regenerated for
  offline training. Recorded `liveTracks` supplies front-radar points.

Adjacent route segments stay in one split. The complete selected-log list is in
`radar_lead_training_manifest.json`; maintained positive, negative, timing,
continuity, stationary, and suppression windows are in
`cutin_validation_cases.json`.

## Model metrics

These temporal-head metrics come from each artifact's held-out weak-label
validation set. They measure the raw classifier before production geometry,
vision matching, persistence, and control suppression.

| Source model | Precision | Recall | F1 | FP | FN |
| --- | ---: | ---: | ---: | ---: | ---: |
| Front v18 | 3.2% | 21.5% | 5.6% | 1,400 | 172 |
| Corner v20 | 83.2% | 59.6% | 69.4% | 102 | 342 |

The front raw score is not a standalone deployment score. Front `leadOne` uses
vision-radar association, while front cut-in is a no-corner fallback behind
strict temporal and geometry gates. A higher-recall front retrain reached 72.6%
recall but only 12.8% precision and regressed four maintained scenes, so it was
rejected.

The v20 corner model combines the broad weak-label corpus with the maintained
video-reviewed routes. Normal decisions use its temporal output. The production
filter adds the narrow two-frame close-entry path described above for a
video-reviewed near-field case where the temporal head was late.

## Production regression

The deployed source pair passes the complete maintained video suite:

| Scene source | Passed |
| --- | ---: |
| Front | 8/8 |
| Corner | 28/28 |
| Front + corner | 35/35 |
| Total | 71/71 |

The gate covers real cut-ins, early deadlines, cut-in-to-`leadOne` continuity,
stationary lead selection, adjacent traffic, curves, tunnels, roadside objects,
close reflections, point jumps, buses, parallel traffic, and output suppression
when a cut-in candidate remains behind a closer primary lead. The suite also
requires the Carnival near-field cut-in to be selected by 5.60 s and verifies
that the same object is not reported again after becoming `leadOne`. Two
additional Carnival windows require the left truck to remain clear before
sustained inward motion begins, while preserving its real 47.11 s detection.

An additional full-log comparison replayed 41,980 frames from all 35 maintained
source routes against the previous corner model. New sustained detections were
limited to reviewed positive scenes, while removed detections matched reviewed
false-positive scenes.

This is a regression result for labeled windows, not an accuracy claim for every
unreviewed frame in every route.

## Performance

Direct PC timing of the production `VisionModelRadarController.update()` path
over 3,597 warm frames from representative front, corner, and
front-plus-corner routes:

| Metric | Time |
| --- | ---: |
| Mean | 1.54 ms/frame |
| p95 | 2.41 ms/frame |
| p99 | 3.59 ms/frame |
| Maximum | 4.68 ms/frame |

The controller runs at 20 Hz with a 50 ms frame period. These are PC timings,
not a substitute for on-device profiling, but they show that two small source
models are not expected to dominate the radard cycle.

## Reproduction

Run the production regression with the deployed files:

```powershell
py -3.12 openpilot/selfdrive/carrot/radar/tools/validate_radar_lead_model.py --no-baseline
```

Open the visual review tool with the same files:

```powershell
py -3.12 openpilot/selfdrive/carrot/radar/tools/radar_lead_validation_review.py
```

Model SHA-256:

- Front: `114333c5409bcfe50f2811bb29764c7e39cad6bd5e00500c4c7805f1c837ab41`
- Corner: `10f6fbc71155ae6ea114ddac3f3477b9a3cca84604258227cfcb0bbfdc38997b`
