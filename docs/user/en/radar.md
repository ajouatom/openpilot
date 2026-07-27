# Radar Tracks and Corner Radar

[한국어](../ko/radar.md)

> [!NOTE]
> This is the canonical English user guide maintained with the `carrot-wip` code. When user-visible behavior changes, update this document together with the related code and tests.

A radar fitted to the vehicle does not guarantee that carrotpilot can read the required CAN messages. Results can differ by model year, trim, radar part, firmware, message group, and harness connection even for the same vehicle name.

> [!WARNING]
> An unverified radar configuration can cause dashboard warnings, CAN faults, incorrect lead selection, or false cut-in detection. Record the original values and test one setting at a time only when the exact vehicle configuration has been validated.

<a id="front-radar"></a>
## Front radar tracks

`EnableRadarTracks` selects the source and processing used for front lead information.

| Value | Current code behavior | Guidance |
|---:|---|---|
| `-2` | VOACC vision-only experiment | Development testing only |
| `-1` | Always use SCC | Confirm the vehicle configuration |
| `0` | Use stock SCC radar | Default |
| `1` | Use raw front-radar tracks | Requires vehicle-specific activation and message support |
| `2` | Combine radar tracks with low-speed SCC | Test only on an identical validated configuration |
| `3` | Add cut-in and low-observability vehicle processing | Experimental; false detections are possible |

On non-CAN FD Hyundai/Kia vehicles, a positive value attempts to enable radar tracks during startup and stores the result in `EnableRadarTracksResult`. Confirm both the activation result and actual incoming tracks; physical radar presence alone is not enough.

<a id="corner-radar"></a>
## Corner radar

| `EnableCornerRadar` | Meaning |
|---:|---|
| `0` | Disabled |
| `1` | Use supported corner-radar tracks |
| `2` | Also use corner radar for cut-in detection |

Corner-radar objects are created only when the vehicle code recognizes a supported message group. The supported 0x430 message family is also classified as corner-radar input rather than front radar. The cut-in processing in mode `2` currently focuses on Hyundai-family implementations and must not be generalized to other manufacturers.

### dPath physical radar processing

| `RadarDPathMode` | Meaning |
|---:|---|
| `0` | Keep the existing `radard` lead and cut-in processing (default) |
| `1` | Run the independent dPath RadarD, calculate front/SCC-to-vision leadOne first, then use only physical dPath CUT-INs as leadTwo |

Mode `1` does not use a learned model, call `controls/radard.py`, or mix in its output. Following the order used by the removed `radard_model.py`, the independent process first matches model lead zero to front/SCC radar and assigns leadOne. Only then can a different OUT-to-IN object confirmed by the physical predictor for 0.25 seconds become leadTwo. A 0.75-second recent-primary identity hold also checks distance, lateral position, and velocity continuity so a brief vision ID change cannot republish the same vehicle as leadTwo. A vehicle farther than leadOne or beyond `max(20 m, vEgo × 2 seconds + 10 m)` is excluded from longitudinal control.

Once corner-radar points have been observed, the mode uses only corner motion for the rest of that process run. A configuration with no corner points uses only `frontRadar` raw tracks. It does not switch between front and corner from frame to frame, and SCC is never a dPath-motion input. An incorrect leadTwo can affect real deceleration, so enable this mode only on the same vehicle configuration after completing shadow validation.

The `leadLeft`, `leadRight`, and side lists used by lane-change assistance are also published from visible adjacent vehicles on that same motion sensor. A point below `|vLead| < 3 km/h` may supply its current position there, but it never builds dPath history or becomes predicted leadTwo.

<a id="lead-selection"></a>
## Lead selection and validation

The manager never runs both radar implementations together. With `RadarDPathMode=0`, only the conventional `openpilot.selfdrive.controls.radard` runs and its leadOne/leadTwo selection is unchanged. With `RadarDPathMode=1`, that process is stopped and only the independent `openpilot.selfdrive.carrot.radar.radard_dpath` runs. It calculates front/SCC-to-vision leadOne first, calculates leadTwo with the physical predictor below, and publishes `radarState` directly. Front radar, SCC, and corner radar retain their input roles and source identity, and no learned radar-lead model is used.

The headless validator can report existing radard and the experimental physical predictor separately. The visual replay deliberately shows only the physical predictor, never imports existing radard `leadOne`, `leadTwo`, or CUT-IN markers, and does not change longitudinal control.

The shadow predictor:

- uses only `measured=true` radar points;
- uses motion points only from 5 m behind ego through 100 m ahead;
- aligns each replay radar point to the model-path timestamp with its measured relative velocity, then projects the point onto the same-time model-path polyline: `S` is arc distance along the centerline and `dPath` is signed normal distance from it;
- keeps only the ego lane and its immediate left/right lanes, using the fixed model-path-relative range `|dPath| <= 5.4 m`;
- on each adjacent side, keeps points closer than 5 m and the nearest visible vehicle at or beyond 5 m, while excluding vehicles hidden farther ahead on that same side from detection; measured in-scope history is retained so a vehicle can be evaluated continuously when it becomes visible;
- uses only corner-radar motion when measured corner data is available for the log, otherwise uses `frontRadar` raw-track motion; SCC remains visible to existing radard but is not motion-predictor input, and the predictor does not switch sources on individual frames;
- shows points below `|vLead| < 3 km/h` as position-only references and does not build or extrapolate motion history for them;
- never switches between lane center and model path from frame to frame;
- does not apply yaw-rate correction again after the radar point and path share the same timestamp and ego coordinate frame;
- verifies reused track IDs and short gaps using physical position and velocity continuity;
- maintains independent front and corner histories and parameters;
- forms a two-dimensional path-relative history from projected centerline progress `S` plus integrated ego travel and signed lateral offset `dPath`, rather than treating raw `dRel` as path distance;
- fits `dPath` against actual target progress in `S`, so ego-time lateral drift is not extrapolated when the target has little longitudinal progress;
- uses the long-window `(S, dPath)` motion vector and its angle relative to the model-path tangent for the prediction mean, limits confidence when a future extrapolation exceeds its observed spatial baseline, and uses short-window disagreement to increase curvature and uncertainty rather than forcing a turn;
- for corner radar, compares position-derived normal motion with the radar-reported lateral velocity and lowers CUT-IN/CUT-OUT confidence when those measurements do not describe the same physical motion, such as a reflection point migrating across a vehicle body;
- predicts future `dRel` and `dPath` at synchronized 0.5, 1.0, 1.5, and 2.0 second horizons; and
- reports CUT-IN and CUT-OUT probabilities separately.

The current path-overlap check includes ego and target vehicle half-widths. A measured vehicle already overlapping the path is shown as current `IN`; a newly observed point that starts there is not treated as a new shadow CUT-IN. A physically tracked `OUT -> IN` crossing retains its pending entry evidence across the boundary, so the 0.25-second confirmation can finish after overlap begins. Only small path-state and confirmation hysteresis are used. The predictor has no per-route, per-vehicle, or scene-specific exceptions.

### PC replay

`radar_lead_validation_review.py` groups maintained cases by log and opens all 40 unique logs in sequence. Each window shows synchronized qcamera video and only the physical predictor's points, trajectories, probabilities, and CUT-IN events. Existing radard `leadOne`, `leadTwo`, CUT-IN points, and event markers are intentionally absent. At the end of one log the window closes and the next log opens automatically. `--front-only` removes corner points before replay. `--prob` changes the predictor display and pause threshold without changing its physical equations.

Playback pauses only when the physical predictor confirms a new CUT-IN for 0.25 seconds. The replay uses a readable Korean-capable font and Korean operator labels. In the bird's-eye map, gray lines are model lane lines, the white dashed line is their displayed center, and the blue line is the model path used as the predictor's only corridor. The lane center is never substituted into the calculation. By default, every track's source-colored filled fading trail is exactly the path-relative `(S, dPath)` history used by the predictor. Gray or green hollow rings are its synchronized 0.5/1.0/1.5/2.0-second future `(S, dPath)` positions; rings become orange only for a confirmed predictor CUT-IN. `H` shows or hides these predictor histories and futures. `A` separately overlays the ego-motion-stabilized raw radar `(xRel, yRel)` history in gray for diagnosis. That optional overlay is observation-derived, not ground-truth target motion, and is not the prediction history. Ego yaw is used only to align this optional raw overlay; it is not applied again to synchronized `dPath`. The horizontal seek bar marks confirmed predictor CUT-IN entries in orange and maintained validation windows above the bar; it has no existing-radard markers. The radar map is an ego-coordinate view, not a perspective overlay on qcamera. Click the bar to seek, use Space to pause, Left/Right to seek by key, Up/Down to change playback speed, `M` to show or hide predictor CUT-IN markers, and `R` to restart and re-arm handled predictor pauses. `I`, `C`, and `S` apply CUT-IN, CLEAR, or STATIONARY labels. Inside a maintained validation window they update that case; outside those windows a label is stored in `radar_trajectory_labels.json`.

`validate_radar_lead_model.py` keeps its historical filename for compatibility, but it no longer loads or validates a learned model. It replays the full maintained `cutin_validation_cases.json` and `radar_trajectory_labels.json` sets, reports existing radard and physical-shadow results separately, and treats all human labels as validation-only data. Those labels are never used to tune equations, thresholds, or training.

## Radar detection sounds

When openpilot is enabled, a newly confirmed cut-in plays a two-tone cue. A continuously tracked object sounds only once. On the speakerless C3X Lite, the same event uses a GPIO buzzer pattern. The cue reports the selected existing-radard result; it does not change lead selection or longitudinal control. A higher-priority safety alert can take precedence.

## Relationship to harness presets

The first-run presets currently start with these values:

| Configuration | `EnableRadarTracks` | `EnableCornerRadar` |
|---|---:|---:|
| ADAS-module harness | `0` | `1` |
| Camera harness | `0` | `0` |
| Retain stock SCC | `0` | `0` |

The ADAS preset enabling corner radar means that its harness can access such a configuration. It does not mean that every ADAS-equipped vehicle has validated corner-radar messages.

## Verification order

1. Confirm the vehicle, model year, trim, HDA generation, and exact harness location.
2. Record the current values and verify normal behavior at the default value `0`.
3. Find a validation record for the same configuration and supported radar messages.
4. Change only one setting, then restart the vehicle/device.
5. While stationary, check for dashboard warnings and CAN faults.
6. In a safe test environment, verify lead distance, relative speed, and false cut-in detections.
7. Restore the original value immediately if anything is abnormal.

## Code references

- Setting ranges and descriptions: `openpilot/selfdrive/carrot_settings.json`
- Existing production radar lead selection: `openpilot/selfdrive/controls/radard.py`
- Independent dPath RadarD: `openpilot/selfdrive/carrot/radar/radard_dpath.py`
- Front/SCC-to-vision leadOne matching: `openpilot/selfdrive/carrot/radar_motion/primary.py`
- LeadOne-first and leadTwo-second control order: `openpilot/selfdrive/carrot/radar_motion/controller.py`
- Physical shadow predictor: `openpilot/selfdrive/carrot/radar_motion/predictor.py`
- dPath leadTwo selection: `openpilot/selfdrive/carrot/radar_motion/lead_selection.py`
- PC replay: `openpilot/selfdrive/carrot/radar/tools/radar_validation_replay.py`
- Hyundai/Kia radar parsing: `opendbc_repo/opendbc/car/hyundai/radar_interface.py`
- Non-CAN FD radar activation: `opendbc_repo/opendbc/car/hyundai/interface.py`
- First-run presets: `openpilot/selfdrive/carrot/server/features/intro/presets.py`
