# Cut-in Route Validation Set

Last validated: 2026-07-22

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

## Radar Model Visual Review

Open all maintained cases as a visual playlist using the bundled three-head
radar model:

```powershell
py -3.12 openpilot/selfdrive/carrot/radar/tools/radar_lead_validation_review.py
```

Each unique route starts at 0 seconds and plays through the full log. Multiple
validation windows that reference the same rlog are grouped, so that physical
log is opened only once. Playback pauses with a two-tone alert when a shadow
trajectory-model point first reaches the displayed `PROB` threshold. Press
Space to resume after a cut-in, press R to restart the current log, and close
the window to open the next case. Seeking backward rearms every later unlabeled
event, so replaying the same section can alert and pause again. Only final
`leadOne` and `leadTwo` are displayed by default; recorded `radarState`, raw
radar points, and source-head candidates remain available through the display
checkboxes.

The model `leadOne`/`leadTwo` continuity graph remains visible by default. The
current-radard circles and its two additional graph lines require a full legacy
radard recomputation and are disabled by default. Enable that slower comparison
only when needed:

```powershell
py -3.12 openpilot/selfdrive/carrot/radar/tools/radar_lead_validation_review.py --compare-radard
```

The `--hybrid` replay path directly executes the on-device
`VisionModelRadarController`. The replay layer only adapts recorded radar points
and `modelV2` into runtime-shaped inputs; it does not independently reimplement
the final lead or cut-in policy.

Review only positive or negative cases:

```powershell
py -3.12 openpilot/selfdrive/carrot/radar/tools/radar_lead_validation_review.py --expected detect
py -3.12 openpilot/selfdrive/carrot/radar/tools/radar_lead_validation_review.py --expected clear
```

### Reviewing and correcting ground truth

The validation review panel has `CUT-IN`, `CLEAR`, and `STATIONARY` buttons.
Clicking one updates the current case's `expected` value in
`cutin_validation_cases.json` immediately and adds `human_verified: true`.
Use `--list` to audit progress: `[H]` is human verified and `[-]` is still an
automatically prepared label. Review the resulting Git diff before committing.
Track IDs and validation windows remain editable directly in the JSON when the
category alone is not enough.

The review map is laterally enlarged by 4x. Distance runs from 0 m at the bottom
to 100 m at the top; the thick blue line is the model path and gray lines are
lane lines. Point colors are cyan for front radar, purple for corner radar, and
yellow for SCC. A label such as `1013 IN0.93 OUT0.00 S` means track id 1013,
future path-entry probability 0.93, path-exit probability 0.00, and stage `S`.
The detail row retains the raw 0.5/1.0/1.5/2.0-second path-occupancy
probabilities. The
direct-threshold shadow path normally uses these stage codes:

- `S`: selected as the final `leadTwo`
- `O`: emitted by the controller but another target was selected
- `D`: decision was active but was removed by later selection or deduplication
- `L`: farther away than the current `leadOne`; raw score/history only
- `P`: below the decision probability
- `X`: above the PATH-EXIT threshold

The right-side point rows show `IN`, `OUT`, geometric projection `H`, stage,
distance, lateral position, and the four raw occupancy probabilities. `IN--`
on the map means that point did not produce a scored model candidate. The
probability slider controls the green IN highlight and automatic review pauses.
Changing it rebuilds pause events immediately. Its last value is saved in the
PC user settings and reused by later logs and invocations; `--prob 0.70`
overrides the initial value. It does not alter production decisions or the
on-device threshold. When `leadOne` exists, points behind it retain raw model
scores and history but are marked `L` and cannot highlight, alert, pause, or
become a shadow cut-in decision. Orange and yellow boxes and graph lines are
final model `leadOne` and `leadTwo`; a break in a line means that output was
absent at that time.

The runner groups all validation rows for the same physical rlog into one
simulator window. Each maintained window remains visible as an independent
timeline marker and remains independently editable.

Enable `POINTS` to show current sensor returns and `PATH` to show each point's
recent history and predicted future positions at 0.25-second intervals. The map
label `IN` is entry probability, `OUT` is PATH-EXIT probability, and `H` is the
independent geometric occupancy estimate. The `PREDICT` slider changes the
trajectory horizon from 0.25 to 2.0 seconds. The uncertainty bar combines radar
history variance with `position.yStd`, `laneLineStds`, and lane confidence.
The diagnostics also retain `carState.yawRate`, steering angle, and steering
rate. `yawRate * dRel` is displayed as ego-rotation lateral speed so turn-induced
apparent motion can be labeled without hard-rejecting every candidate on a curve.
Low-confidence lane lines cause a fallback to the model position path.

The three ground-truth buttons update `cutin_validation_cases.json` while the
playback cursor is inside a maintained window. At a trajectory event outside
all maintained windows they instead upsert a separate review item in
`radar_trajectory_labels.json`. This lets trajectory candidates be labeled
without weakening or overwriting the production regression set.

### Path-occupancy model validation (2026-07-25)

The final self-supervised run discovered every `rlog.zst` and numbered
`rlog.N.zst`. It reserved all 34 manually labeled logs, requested 1,438
training logs, loaded 1,356, and recorded 82 corrupt or unaligned logs as
skipped. Manual labels contributed zero fitting rows. Front training used
2,127,348 rows from 1,043 logs; corner training used 764,072 rows from 882
logs.

The four occupancy heads have strong grouped out-of-fold F1, but high-precision
transition recall remains low:

| source | occupancy F1 range | CUT-IN threshold | CUT-IN precision / recall | PATH-EXIT threshold | PATH-EXIT precision / recall |
|---|---:|---:|---:|---:|---:|
| front | 0.923-0.935 | 0.995 | 1.000 / 0.00001 | 0.955 | 0.955 / 0.00074 |
| corner | 0.934-0.958 | 0.945 | 0.827 / 0.028 | 0.815 | 0.872 / 0.105 |

The held-out manual table was front TP/FP/FN/TN `0/4/9/93` and corner
`6/2/23/169`. The actual-future audit agreed with 52 of 87 scorable front
labels and 131 of 191 scorable corner labels; 19 front and 9 corner labels
lacked a valid future measurement. Five front and 17 corner manual DETECT
windows contained no measured outside-to-inside transition, while 30 front and
43 corner CLEAR windows did contain one. These labels remain unchanged and are
review targets, not training corrections.

The exact branch-point `carrot-wip` implementation at `9088829005` and the new
final shadow decision were also replayed in isolated Python processes over the
same 34 held-out logs. This comparison includes the `leadOne` distance block,
not only raw model thresholds:

| truth | source | carrot-wip P / R / F1 | path-occupancy P / R / F1 |
|---|---|---:|---:|
| manual CUT-IN/CLEAR | front | 0.000 / 0.000 / 0.000 | 0.091 / 0.111 / 0.100 |
| manual CUT-IN/CLEAR | corner | 0.200 / 0.034 / 0.059 | 0.643 / 0.310 / 0.419 |
| measured future entry | front | 0.500 / 0.100 / 0.167 | 0.444 / 0.133 / 0.205 |
| measured future entry | corner | 0.250 / 0.018 / 0.034 | 0.615 / 0.145 / 0.235 |

Corner improves materially under both truth definitions. Front recall and F1
increase slightly, but measured-future precision drops from 0.500 to 0.444, so
front is not treated as control-validated. The complete per-label rows are in
`radar_path_occupancy_report.json` under `carrot_wip_comparison`.

Automatic trajectory-model `leadTwo` promotion therefore remains disabled for
both sources. Front/SCC PATH-EXIT alone may cancel a stale-primary hold after
vision has already lost `leadOne`; it never removes a currently matched
`leadOne` or changes time gap. Corner PATH-EXIT remains display-only.

Review one matching case directly:

```powershell
py -3.12 openpilot/selfdrive/carrot/radar/tools/radar_lead_simulator.py `
  --validation-case ioniq9-a7-22-truck
```

The initial three-head radar-model scan on 2026-07-16 passed 14 of 16 cases.
It missed cut-in output in these two positive windows, which should be reviewed
first even though lead acquisition can still occur:

- `carnival-5b-15-truck`: no model cut-in from 45-50 s.
- `ioniq9-a7-22-truck`: `leadTwo` id 56 at 20.58 s and `leadOne` id 56 at
  22.74 s, but no model cut-in from 20-29 s.

The source-separated production-controller regression run on 2026-07-25 passed
all 76 maintained windows: 32/32 corner, 9/9 front-only, and 35/35
front-plus-corner windows. The deployed front and corner models keep independent
feature history and decision filters. On a corner-equipped vehicle, cut-in and
secondary decisions come from the corner model; front matching happens only
after selection to obtain control-quality longitudinal values. Only the listed
windows are labeled; new reports must be added to
`cutin_validation_cases.json` before a later change can be called
regression-safe.

The Carnival `carnival-5b-18-early` window is deadline-checked: corner id 1013
must be selected by 5.60 s. A separate clear window verifies that the same
physical vehicle is not reported as a late cut-in after it has already become
`leadOne`.

The Carnival `carnival-5b-15-truck` scene also contains two explicit clear
windows at 42.3-43.6 s and 44.8-46.1 s. Weak lane-relative jitter must not
activate id 39/1071 there; the real inward entry remains detected at 47.11 s.

The very close Carnival `carnival-6a-27-close` vehicle enters beside the bumper
at low ego speed while the learned cut-in score is still zero. Sustained inward
corner history plus physical body intrusion now emits a tentative id 1024 at
55.86 s instead of waiting for model activation at 57.64 s. Its measured range
is reported, but longitudinal speed and acceleration are softened to ego speed
and zero acceleration until the candidate is confirmed.

## Unit Coverage

The route fixes are supported by focused tests in
`openpilot/selfdrive/carrot/tests/test_radar_lead_model.py` and
`openpilot/selfdrive/carrot/tests/test_radar_vision_model_controller.py`:

- slow sustained radar motion accepts the large-truck cut-in;
- close sustained corner intrusion is reported tentatively with softened control;
- lane projection cannot substantially outrun measured radar motion;
- unreliable far corner tracks cannot start a new cut-in.
