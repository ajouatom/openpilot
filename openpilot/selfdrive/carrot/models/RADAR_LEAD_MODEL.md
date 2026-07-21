# Radar lead model v13

## Runtime layout

- Base network: deployed v6 lead, cut-in, and moving-external outputs.
- Stationary network: v11 external classifier, evaluated only when `abs(vLead) < 1.8 m/s`.
- Anticipatory cut-in network: v13 geometry-gated 19-input logistic classifier.
- The stationary score is normalized against its own threshold before it is combined with the base external score.
- Base lead/cut-in/external weights, calibration, and thresholds are unchanged from v12.
- The anticipatory score can only raise cut-in probability after stable identity, 0.4/0.6-second inward lane-relative motion, usable lane confidence, distance, speed, and future-path gates all pass.

The model contains two small `121 -> 64 -> 32` MLPs and one `19 -> 1` classifier. In the full Python replay path over a 1,200-frame route, median processing changed from 1.25 to 1.35 ms per frame. The approximately 0.10 ms/frame increase at 20 Hz is about 2 ms of CPU time per second.

## Radar sources

Offline datasets never use recorded corner points from `liveTracks`. Corner objects are decoded again from raw CAN and assigned stable reconstructed IDs. Recorded `liveTracks` remains the source of front-radar points.

On a device, `radard_model` consumes the current `liveTracks` message. Those points were just decoded from raw CAN by the car's `radar_interface`; decoding CAN again inside radard would duplicate the parser and introduce timing differences.

## Corpus

- Inventory: 1,215 readable logs from `W:\routes`; 926 usable and 75 isolated parse failures.
- Selected: 180 logs across 56 vehicle folders.
- Train: 124 logs, 146,556 frames, 1,890,692 candidate rows.
- Validation: 25 logs, 29,817 frames, 351,208 candidate rows.
- Test: 30 materialized logs, 34,859 frames, 412,219 candidate rows. One selected test log failed export.
- The 16 hand-reviewed cut-in routes were excluded from train, validation, and test.
- Adjacent route segments stay in one split.

The complete selected-log list and profiling fields are in `radar_lead_training_manifest.json`.

## Stationary evaluation

At the validation-selected stationary threshold (0.67), the held-out test changed from:

| Model | Precision | Recall | False positives | False negatives |
| --- | ---: | ---: | ---: | ---: |
| v6 base | 44.1% | 7.2% | 356 | 3,604 |
| v12 combined | 46.6% | 22.1% | 986 | 3,025 |

The controller additionally requires a raw-corner identity, at least seven frames of history, 5-120 m range, `abs(vLead) < 1.8 m/s`, `abs(yvRel) < 0.8 m/s`, and distance-dependent `dPath` limits before a corner-only stopped object can become `leadTwo`.

## Cut-in regression

The 52 maintained detect/clear video-review cases pass 52/52. This count covers only the labeled windows in `cutin_validation_cases.json`; it is a regression gate, not a claim about unlabeled portions of every route. The v13 candidate introduced no activation in the maintained right-curve, parallel-adjacent, close-pass, point-jump, stopped-traffic, close stationary-reflection, near-field front-radar ghost, or tunnel-ghost clear scenes.

The anticipatory labels use a later current-radard confirmation only when the same front/corner alias was already moving consistently toward the lane. They look back at most 0.8 seconds and never overwrite manual negative labels. This added 142 soft-positive training rows and 126 validation rows. It does not copy the current-radard decision time directly; it teaches the stable motion immediately before that decision.

Within the geometry-eligible validation subset, the auxiliary classifier reached 82.8% precision and 46.1% recall (`fp=28`, `fn=158`) at threshold 0.90. On the untouched 30-log test split, combining it with the v12 cut-in score at the production 0.82 temporal threshold changed row/group agreement as follows:

| Model | Precision | Recall | True positives | False positives | False negatives |
| --- | ---: | ---: | ---: | ---: | ---: |
| v12 base cut-in | 35.7% | 39.9% | 65 | 117 | 98 |
| v13 combined cut-in | 35.9% | 54.6% | 89 | 159 | 74 |

The weak test labels are derived from the current-radard confirmation and therefore count some earlier predictions as false positives. The independent video set remains the deployment gate. There, v13 moved `azera-6d-1-front` from 9.99 to 9.45 seconds and `ioniq9-18f-2-front` from 25.74 to 25.60 seconds. Other positive scenes stayed unchanged; several were already earlier than current radard.

The safety override handles a moving object only when front and raw-corner radar identify the same stable object. Inside 8 m it uses the strict near-range geometry directly. From 8-20 m it additionally requires `abs(dPath) > 1.8 m` and consistent inward lateral motion over 0.4 and 0.6 seconds. Closing objects may qualify at predicted `dPath < 2.2 m`; objects holding range or pulling away wait until `dPath < 1.85 m`. This recovered `carnival-5b-15-truck`, `ioniq9-a7-22-truck`, and `santafe-421-1-truck` without triggering any maintained clear case or prematurely promoting the faster adjacent vehicle in `carnival-5b-18-early`.

The front-only close-range override requires raw `yRel` history and lane-relative history to agree on inward motion. At 2-4.5 m it additionally requires a stable 0.4/0.6-second range history, `abs(dPath) < 1.35 m`, and two confirming frames. This detects the very close vehicle in `carnival-6a-27-close` at 57.32 seconds while rejecting the maintained 3-5 m front-radar reflection, longitudinal-pass, and tunnel-ghost cases.

Front-only vehicles at 12-50 m use a separate strict override. Some front radars report zero instantaneous lateral velocity, so the override projects 0.5 seconds from agreeing 0.4-second and 0.6-second raw and lane-relative history before three-frame confirmation. This detects `pv5-129-8-right-front-cutin` before vision promotes the vehicle to `leadOne` without relaxing the close-range ghost filter.

For objects still outside the path (`abs(dPath) > 1.8 m`), cut-in persistence accumulates only while predicted `dPath` decreases by at least 0.03 m. This rejects high model probabilities for adjacent vehicles that are parallel or moving away, including the false activation at 41.5 seconds in `carnival-5b-15-truck`.

Validation review events re-arm after an ID clears for at least one second. A false activation can therefore no longer hide a later real cut-in by the same stable track ID.

The offline workbench adapts recorded points and `modelV2` into the normal runtime input shape and directly calls `VisionModelRadarController`, the same controller used by `radard_model.py`. Final `leadOne`, `leadTwo`, cut-in reporting, low-speed suppression, submeter rejection, SCC inclusion, and sensor track priority therefore use production code rather than a replay-side copy. Raw high-probability candidates remain available through the display filters without being mislabeled as final controller outputs.

The stationary network does not alter cut-in probabilities.

## On-device hybrid policy

`RadarLeadModelMode=1` runs the independent `radard_model.py` process path.
Its compact vision/radar matcher uses the first `modelV2` lead as an existence
and association observation, applies Laplacian distance/lateral/velocity
scoring, and requires all three sanity gates. The published `leadOne`
kinematics always come from the matched front radar or SCC object; raw vision
distance and velocity are never published as a fallback.

The model lead head remains in this artifact for offline analysis, but it is
not used for `leadOne`. Temporally confirmed cut-in and external predictions
populate `leadTwo`. A model object is allowed to match `leadOne` and also appear
as `leadTwo`; consumers already evaluate both leads, and preserving the model
decision avoids hiding a real cut-in transition.

The v13 anticipatory classifier is independent from the base heads. Its output
is threshold-remapped to the production temporal threshold and combined only
inside the strict eligibility gate, so leadOne, external/stationary scoring,
and the existing cut-in probability remain unchanged elsewhere.

## Reproduction

Use `radar_lead_corpus.py` to scan, select, and export weak teacher labels. `radar_lead_anticipatory_dataset.py` adds same-identity soft early labels and can produce a compact set containing all positives, hard negatives, and sampled easy negatives. `radar_lead_anticipatory_train.py` trains and appends the gated auxiliary arrays without changing the base artifact. `validate_radar_lead_model.py` compares current radard, the deployed baseline, and a candidate over every maintained video case.

The general trainer preserves both `stationary_*` and `anticipatory_*` arrays when a base head is fine-tuned. The stationary v11 run used a maximum positive weight of 5; larger automatic weighting produced unacceptable false positives.

Model SHA-256: `3878f25c72397e70263eadab54bda97d6fa995b1b217bdb1ef7f0151191acd3a`
