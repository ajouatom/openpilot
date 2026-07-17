# Radar lead model v12

## Runtime layout

- Base network: deployed v6 lead, cut-in, and moving-external outputs.
- Stationary network: v11 external classifier, evaluated only when `abs(vLead) < 1.8 m/s`.
- The stationary score is normalized against its own threshold before it is combined with the base external score.
- Lead and cut-in weights, calibration, thresholds, and per-candidate probabilities are unchanged from v6.

The model contains two small `121 -> 64 -> 32` MLPs. On the development PC, inference for 20 objects changed from 0.0934 ms to 0.1395 ms per frame.

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

The 17 maintained video-review cases across 16 routes pass 17/17 after identity de-duplication and the fused inward safety override.

The safety override handles a moving object only when front and raw-corner radar identify the same stable object. Inside 8 m it uses the strict near-range geometry directly. From 8-20 m it additionally requires `abs(dPath) > 1.8 m` and consistent inward lateral motion over 0.4 and 0.6 seconds. Closing objects may qualify at predicted `dPath < 2.2 m`; objects holding range or pulling away wait until `dPath < 1.85 m`. This recovered `carnival-5b-15-truck`, `ioniq9-a7-22-truck`, and `santafe-421-1-truck` without triggering any maintained clear case or prematurely promoting the faster adjacent vehicle in `carnival-5b-18-early`.

For objects still outside the path (`abs(dPath) > 1.8 m`), cut-in persistence accumulates only while predicted `dPath` decreases by at least 0.03 m. This rejects high model probabilities for adjacent vehicles that are parallel or moving away, including the false activation at 41.5 seconds in `carnival-5b-15-truck`.

Validation review events re-arm after an ID clears for at least one second. A false activation can therefore no longer hide a later real cut-in by the same stable track ID.

The offline workbench uses the production controller's final selection gates: `leadOne` accepts only front/SCC objects within `abs(dPath) < 2.4 m`, falls back to the first vision lead when radar is absent, and `leadTwo` is reserved for control-usable cut-in or external objects. Raw high-probability candidates remain available through the display filters without being mislabeled as final controller outputs.

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

This policy does not require retraining the current artifact because its
cut-in and external heads were trained and calibrated independently. A future
two-head retrain can remove the unused lead output after collecting more
hard-negative cut-in and stationary examples.

## Reproduction

Use `radar_lead_corpus.py` to scan, select, and export weak teacher labels. The trainer supports a parsed `--cache`, head-specific training, and `--stationary-external`. The stationary v11 run used a maximum positive weight of 5; larger automatic weighting produced unacceptable false positives.

Model SHA-256: `75b6efe4c765551b3c31ee93fbf50864ee6cf866eaa5b6ed2007544d036a5b6c`
