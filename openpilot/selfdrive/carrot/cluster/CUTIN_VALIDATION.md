# Cut-in Route Validation Set

Last validated: 2026-07-16

This document records the 16-case route-based regression set used for the S50 cut-in
logic in `radard.py` and the offline evaluator used by `cluster_replay_usb.py`.
All paths are below `W:\routes`.

## How To Use This File

Use this document as the human checklist: the table says which video interval
to inspect and whether current code should detect a cut-in. The executable case
list is `cutin_validation_cases.json`; run all 16 current-code checks from the
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
commands below to inspect the video, `leadOne`, and detection timing manually.

## Sequential Video Review

Review all 16 labeled windows in order:

```powershell
.\.venv\Scripts\python.exe `
  openpilot\selfdrive\carrot\cluster\review_cutin_routes.py
```

Each replay starts three seconds before its labeled window, ends three seconds
after it, and then opens the next case. Stored cut-ins are off by default. Add
`--show-recorded-cutins` only for old-versus-current comparison, or use
`--output both` to render to the PC and USB display together.

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
- Stored `radarState.leadsCutIn` decisions are not part of the pass/fail result.
  Add `--show-recorded-cutins` only when comparing the old stored decision with
  the current evaluator.
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
py -3.12 openpilot/selfdrive/carrot/radar_lead_validation_review.py
```

Each unique route starts at 0 seconds and plays through the full log. Multiple
validation windows that reference the same rlog are grouped, so that physical
log is opened only once. Playback pauses
with a two-tone alert only when a model cut-in becomes active. Press Space to
resume after a cut-in, press R to restart the current log, and close the window
to open the next case. Only final `leadOne` and `leadTwo` are displayed by
default; recorded `radarState`, raw radar points, and source-head candidates
remain available through the display checkboxes.

Review only positive or negative cases:

```powershell
py -3.12 openpilot/selfdrive/carrot/radar_lead_validation_review.py --expected detect
py -3.12 openpilot/selfdrive/carrot/radar_lead_validation_review.py --expected clear
```

Review one matching case directly:

```powershell
py -3.12 openpilot/selfdrive/carrot/radar_lead_simulator.py `
  --validation-case ioniq9-a7-22-truck
```

The initial three-head radar-model scan on 2026-07-16 passed 14 of 16 cases.
It missed cut-in output in these two positive windows, which should be reviewed
first even though lead acquisition can still occur:

- `carnival-5b-15-truck`: no model cut-in from 45-50 s.
- `ioniq9-a7-22-truck`: `leadTwo` id 56 at 20.58 s and `leadOne` id 56 at
  22.74 s, but no model cut-in from 20-29 s.

## Unit Coverage

The route fixes are supported by focused tests in
`openpilot/selfdrive/controls/tests/test_front_radar_cutin.py`:

- slow sustained radar motion accepts the large-truck cut-in;
- lane projection cannot substantially outrun measured radar motion;
- unreliable far corner tracks cannot start a new cut-in.
