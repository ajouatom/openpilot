# Cut-in Route Validation Set

Last validated: 2026-07-26

This document records the route-based regression set used for the S50 cut-in
logic in `radard.py` and the standalone offline validation tools.
All paths are below `W:\routes`.

## How To Use This File

Use this document as the human checklist: the table says which video interval
to inspect and whether current code should detect a cut-in. The executable case
list is `cutin_validation_cases.json`; run all current-code checks from the
repository root with:

```powershell
.\.venv\Scripts\python.exe `
  openpilot\selfdrive\carrot\cluster\validate_cutin_routes.py
```

Run one matching case while changing the logic:

```powershell
.\.venv\Scripts\python.exe `
  openpilot\selfdrive\carrot\cluster\validate_cutin_routes.py `
  --case ioniq9-a7-22
```

The script checks only `NEW CUT-IN`, returns exit code 1 on a regression, and
does not count stored `radarState.leadsCutIn`. After it passes, use the replay
commands below to inspect the video, `leadOne`, and the decision recorded by the
device. `cluster_replay_usb.py` does not recompute cut-ins.

Validate a radar lead model against the same detect/clear scenes and compare its
first activation with current radard and the deployed model using:

```powershell
.\.venv\Scripts\python.exe `
  openpilot\selfdrive\carrot\validate_radar_lead_model.py `
  --model openpilot\selfdrive\carrot\radar\models\radar_lead_multitask.npz
```

## Sequential Video Review

Review all 16 labeled windows in order:

```powershell
.\.venv\Scripts\python.exe `
  openpilot\selfdrive\carrot\cluster\review_cutin_routes.py
```

Each replay starts three seconds before its labeled window, ends three seconds
after it, and then opens the next case. Stored `radarState.leadsCutIn` decisions
are always shown. The replay pauses and plays its PC alert at the prompt event
recorded in `selfdriveState`. Use `--output both` to render to the PC and USB
display together.

Resume at a particular case, or keep each replay open until manually closed:

```powershell
.\.venv\Scripts\python.exe `
  openpilot\selfdrive\carrot\cluster\review_cutin_routes.py `
  --start-at ioniq9-a7-22 --manual --pause-on-cutin
```

## Validation Rules

- Use sensitivity 50.
- `corner` cases evaluate corner-radar tracks reconstructed with stable raw CAN
  object identity.
- `front` cases evaluate front-radar tracks with `--front-radar-only`.
- A positive case passes when at least one current-code cut-in detection overlaps
  the validation window.
- A negative case passes when the current-code evaluator produces no cut-in in
  the validation window.
- Stored `radarState.leadsCutIn` decisions are not part of the executable
  validation pass/fail result. They are always shown by `cluster_replay_usb.py`;
  use `validate_cutin_routes.py` or `radar_lead_validation_review.py` when the
  current code must be recalculated from log inputs.
- Results outside a listed validation window are not labeled by this document.
  They must be checked against video before being treated as true or false.

## Positive Cases

| Vehicle folder | Segment and log | Source | Window | Scene to verify | 2026-07-16 result |
| --- | --- | --- | --- | --- | --- |
| `KIA_CARNIVAL_4TH_GEN c4dcf95545dafe68` | `0000005b--02344a68f2--18\rlog.1.zst` | corner | 0-10 s | Multiple early cut-ins | PASS, first detection 5.39 s |
| `KIA_CARNIVAL_4TH_GEN c4dcf95545dafe68` | `0000005b--02344a68f2--15\rlog.1.zst` | corner | 45-50 s | Truck enters from the left | PASS, 47.87-49.11 s |
| `HYUNDAI_AZERA_7TH_GEN f3a537501c7197ec` | `0000006d--50a44bb074--1\rlog.zst` | front | 8-13 s | Left vehicle cuts in | PASS, 10.43-12.82 s, id 63 |
| `HYUNDAI_IONIQ_9 c92fab3f15c0dbfb` | `0000018f--2bb73a4538--2\rlog.zst` | front | 24-30 s | Front-radar cut-in around 26 s | PASS, 26.61-29.20 s, id 55 |
| `KIA_CARNIVAL_4TH_GEN c4dcf95545dafe68` | `0000006a--f0976fc330--27\rlog.zst` | corner | 54-60 s | Very close white vehicle enters from the left | PASS, 57.32-59.97 s, id 32 |
| `HYUNDAI_IONIQ_9 c92fab3f15c0dbfb` | `000001a7--54660d9df7--22\rlog.zst` | corner | 20-29 s | Large truck begins entering around 21 s; compare `leadOne` timing | PASS, `leadOne` first selects id 56 at 22.46 s; `NEW CUT-IN` 25.29-27.08 s, id 2102 |

The large truck in `000001a7--54660d9df7--22` is the slow-entry regression.
Its stable corner track moves from roughly `yRel=-3.5 m` toward `-1.5 m`, but
its inward speed peaks around `0.40 m/s`, below the normal S50 threshold of
`0.50 m/s`. It must be accepted through the sustained radar-motion path.
The front-radar `leadOne` selection at 22.46 s means longitudinal control sees
the truck about 2.8 seconds before the separate cut-in warning. `leadOne`
alternates with the vision lead, so warning timing and control acquisition must
be reviewed as separate results.

## Negative Cases

| Vehicle folder | Segment and log | Source | Window | False-positive scene | 2026-07-16 result |
| --- | --- | --- | --- | --- | --- |
| `HYUNDAI_IONIQ_5_PE 8b06424f3adf2bd3` | `00000cab--0e0be97e78--49\rlog.zst` | corner | 0-30 s | Right curve; adjacent vehicle stays in its lane | PASS, no detection |
| `HYUNDAI_IONIQ_9 c92fab3f15c0dbfb` | `00000192--b0f1546431--7\rlog.zst` | corner | 0-10 s | Early corner-radar false positive | PASS, no detection |
| `HYUNDAI_SANTAFE_MX5_HEV 61d2c91e1039ab5e` | `00000421--6bbe001d3d--1\rlog.zst` | corner | 15-20 s | False cut-in around 17 s | PASS, no detection |
| `HYUNDAI_IONIQ_5_PE 8b06424f3adf2bd3` | `00000cb5--cf3e24bc3d--3\rlog.zst` | corner | 0-60 s | Adjacent vehicles before meeting a center lead | PASS, no detection |
| `KIA_K8_HEV_1ST_GEN 4aa2ded146fd78b9` | `00000216--f69e641982--4\rlog.zst` | corner | 12-17 s | Raw corner slot changes object near 14 s | PASS, no detection |
| `GENESIS_GV80 4857b2d26ed4648e` | `00000218--6aa147e461--2\rlog.zst` | corner | 35-40 s | Passing a close adjacent vehicle near 37 s | PASS, no detection |
| `GENESIS_GV80 4857b2d26ed4648e` | `00000218--6aa147e461--6\rlog.zst` | corner | 39-44 s | Lateral point jump near 41 s | PASS, no detection |
| `KIA_K8_HEV_1ST_GEN 4aa2ded146fd78b9` | `00000221--1ee7be4212--18\rlog.zst` | corner | 48-53 s | Persistent false cut-in around 50 s | PASS, no detection |
| `HYUNDAI_IONIQ_9 c92fab3f15c0dbfb` | `000001a7--54660d9df7--2\rlog.zst` | corner | 0-33 s | False positives near 2 s and 29 s | PASS, no detection |
| `HYUNDAI_IONIQ_9 c92fab3f15c0dbfb` | `000001a5--ba129171a3--22\rlog.zst` | corner | 30-37 s | Adjacent vehicle remains in its lane on a curve near 33 s | PASS, no detection |
| `HYUNDAI_IONIQ_5_PE 8b06424f3adf2bd3` | `00000cd2--ea0776cc10--4\rlog.zst` | front+corner | 12-14.5 s | Stopped id 33 at 7.65 m is behind leadOne at 3.55 m | PASS, current model suppresses cut-in output and audio |
| `HYUNDAI_PALISADE b84b4a4fbb604be1` | `00000bef--a8eb1d8c98--2\rlog.zst` | front | 24.4-25.2 s | Close right vehicle id 35 enters before becoming leadOne | PASS, front-only lane-history cut-in at 24.64 s |

The two `000001a7--54660d9df7--2` failures cover different mechanisms:

- Near 2 s, lane-relative motion reached about `1.25-2.19 m/s` while radar
  inward motion was only about `0.16-0.36 m/s`. The projection must not outrun
  radar motion by more than the normal consistency margin.
- Near 29 s, a corner track at about 46 m swept laterally during a curve while
  the vehicle remained in its lane.

In `000001a5--ba129171a3--22`, corner track 2967 appeared to move inward at
about 34 m while a matching front track and video showed an adjacent-lane
vehicle on a curve. All maintained true corner-radar detections begin within
4.8 m, while the front radar covers the validated 18.5 m cut-in. New corner
cut-in entry is therefore limited to 30 m; the false 34 m and 46 m sweeps are
rejected, while the front-radar cut-in range remains 50 m.

## Replay Commands

Run a corner-radar case from the repository root:

```powershell
.\.venv\Scripts\python.exe openpilot\selfdrive\carrot\cluster_replay_usb.py `
  "W:\routes\HYUNDAI_IONIQ_9 c92fab3f15c0dbfb\000001a7--54660d9df7--22\rlog.zst" `
  --output window --route-overlay full --cutin-radar-source corner `
  --cutin-sensitivity 50
```

Run a front-radar case:

```powershell
.\.venv\Scripts\python.exe openpilot\selfdrive\carrot\cluster_replay_usb.py `
  "W:\routes\HYUNDAI_AZERA_7TH_GEN f3a537501c7197ec\0000006d--50a44bb074--1\rlog.zst" `
  --output window --route-overlay full --front-radar-only `
  --cutin-radar-source front --cutin-sensitivity 50
```

Add `--show-recorded-cutins` to overlay decisions stored in the original
`radarState`. The replay tools window also exposes this as a checkbox. Current
code detections remain labeled `NEW CUTIN`; stored detections are comparison
data and must not be counted as a current-code regression failure.

Use `--no-pause-on-cutin` when scanning a whole route without stopping at each
detection.

## Radar Path-Occupancy Review

Open every maintained case with the same model, features, decision filter, and
final controller used on the device:

```powershell
py -3.12 openpilot/selfdrive/carrot/radar/tools/radar_lead_validation_review.py
```

The replay adapter only converts recorded `liveTracks`, `radarState`, `modelV2`,
and `carState` messages into runtime-shaped inputs. Final `leadOne`, `leadTwo`,
CUT-IN, and CUT-OUT decisions come directly from
`VisionModelRadarController`.

Playback pauses and sounds only when a trajectory CUT-IN becomes the final
production `leadTwo`. A high-probability point that is rejected because it is
farther than `leadOne`, duplicates the primary, or loses final selection remains
visible but does not pause. Seeking backward rearms every later final event, so
replaying the same section can pause again. Press Space to continue and R to
restart the current log.

`--prob` changes only the display threshold. It never changes device decisions
or creates pause events. With no `--prob`, the display starts at the active
source model's deployed threshold. The `PREDICT`/`DISPLAY FUTURE` slider selects
which raw horizon is drawn; it does not rerun or change the model decision.

### Probability contract

The front and corner models are separate, but both predict the same four raw
targets:

- `P.5`, `P1`, `P1.5`, `P2`: probability that this vehicle occupies the ego
  path after 0.5, 1.0, 1.5, and 2.0 seconds.
- `P0`: measured current path occupancy, exactly 1 inside and 0 outside.
- `IN`: maximum of `P0` and usable future occupancy values before ego passes
  the object. A vehicle currently inside therefore has `IN=1`.
- `OUT`: maximum of `1-P0` and usable future outside values. A vehicle currently
  outside therefore has `OUT=1`.

At one horizon, inside and outside probabilities are complements. Aggregated
`IN` and `OUT` need not add to one: both can be high when a vehicle crosses the
path during the two-second window. The model outputs are never overwritten to
force a display value.

The decision filter is deliberately small. An outside vehicle uses `IN` for
CUT-IN; an inside vehicle uses `OUT` for CUT-OUT. It applies only 0.05 release
hysteresis plus a one-second hold when a physically continuous measured track
actually crosses the boundary. A vehicle first observed inside is an ordinary
in-path lead, not automatically a new CUT-IN. The controller keeps only the
essential final checks: candidates farther than `leadOne` cannot become
`leadTwo`, and the current/recent primary is deduplicated. No route-specific
distance, speed, lane-jitter, parked-vehicle, or tentative-intrusion correction
is applied to trajectory candidates.

CUT-OUT can release only a stale-primary hold after vision has already lost the
primary. It does not remove a currently matched `leadOne` and does not directly
change time gap.

### Screen legend and labeling

The map is laterally enlarged 4x. Cyan points are front radar, purple points are
corner radar, and yellow points are SCC. One outline ring is used:

- green: final selected `leadTwo`;
- purple: active/high CUT-IN probability;
- orange: active CUT-OUT probability;
- red: blocked because it is farther than `leadOne`;
- muted gray: not active.

Labels show `id`, `P0`, `IN`, `OUT`, one selected raw horizon, and the final
stage. Raw point text is limited to the nearest 45 m, five map labels, and three
detail rows to keep the screen readable. A farther final `leadTwo` still has its
box and detail row. The lower graph separates outside-candidate `IN` from
inside-candidate `OUT`, and also shows decision, controller output, and final
selection.

The ground-truth `CUT-IN`, `CLEAR`, and `STATIONARY` buttons update only review
labels. They never become training targets. Inside a maintained window they
update `cutin_validation_cases.json`; outside one they update
`radar_trajectory_labels.json`.

Review only positive or negative cases:

```powershell
py -3.12 openpilot/selfdrive/carrot/radar/tools/radar_lead_validation_review.py --expected detect
py -3.12 openpilot/selfdrive/carrot/radar/tools/radar_lead_validation_review.py --expected clear
```

Enable the slower legacy `radard` comparison only when required:

```powershell
py -3.12 openpilot/selfdrive/carrot/radar/tools/radar_lead_validation_review.py --compare-radard
```

### Self-supervised training and held-out validation (2026-07-26)

Training discovered both `rlog.zst` and numbered `rlog.N.zst`. All 34 manually
labeled logs were held out. Of 1,438 requested training logs, 1,356 loaded and
82 corrupt or unaligned logs were recorded as skipped. Manual labels contributed
zero fitting rows. Front training used 2,127,348 rows from 1,043 logs; corner
training used 764,072 rows from 882 logs.

Targets came only from the same physically continuous vehicle's measured future
position. `measured=false` front slots were excluded. Track-ID reuse, excessive
time gaps, and physically discontinuous jumps split continuity identities.
Future samples were used only as targets and are not available to runtime
inference.

The manual labels are intentionally an imperfect review set. Of the scorable
rows, only 52/87 front and 131/191 corner labels agree with measured future path
entry. Full production replay on those manual windows produced:

| source | manual P / R / F1 | measured-future P / R / F1 |
|---|---:|---:|
| front | 0.000 / 0.000 / 0.000 | 0.857 / 0.400 / 0.545 |
| corner | 0.579 / 0.379 / 0.458 | 0.842 / 0.291 / 0.432 |

For comparison, the branch-point `carrot-wip` implementation at `9088829005`
scored measured-future P/R/F1 of `0.500/0.100/0.167` for front and
`0.250/0.018/0.034` for corner. The new implementation therefore improves the
objective same-vehicle future target for both sources without adding
scene-specific corrections. False positives remain visible and are not hidden
by ad hoc post-processing.

The complete training provenance, thresholds, per-label rows, manual table, and
branch comparison are in `radar_path_occupancy_report.json`.

## Unit Coverage

Focused tests cover:

- raw future probabilities remain untouched while current occupancy determines
  `P0`;
- inside means `IN=1`, outside means `OUT=1`;
- CUT-IN and CUT-OUT use state-appropriate scores and generic hysteresis;
- same-vehicle measured boundary transitions are held briefly, while
  first-observed inside leads are not classified as new CUT-IN;
- front and corner histories and decisions stay separate even when numeric track
  IDs match;
- the PC review pauses only for the same final `leadTwo` event produced by the
  production controller, including rewind rearming and display-only sliders.
