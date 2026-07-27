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

### Radar Motion mode

| `RadarMotionMode` | Meaning |
|---:|---|
| `0` | Keep the existing `radard` lead and cut-in processing (default) |
| `1` | Run the independent dPath RadarD, calculate vision-supported radar leadOne first, then use only physical dPath CUT-INs as leadTwo |

Mode `1` does not use a learned model, call `controls/radard.py`, or mix in its output. Following the order used by the removed `radard_model.py`, the independent process first matches model lead zero to front/SCC radar and assigns leadOne. A fresh moving leadOne match must also meet the conventional radard's `1e-4` minimum joint distance/lateral/velocity likelihood; an already continuous identity may tolerate a brief lower score. A separate near-stationary leadOne fallback accepts a measured in-path radar object with `|vLead| <= 2.5 m/s` after model lead zero supplies at least 0.05 probability at a consistent position, its reported speed differs from the radar object by no more than 10 m/s, and the radar remains physically continuous for 0.25 seconds. This prevents a model lead moving at road speed from seeding a stationary infrastructure reflection. Corner object tracks are preferred when corner data is available because their object identity is more stable; otherwise front tracks are used. Once confirmed, the same stationary object remains leadOne through weak or missing vision and physically continuous radar-reflection ID handoffs. It releases on physical discontinuity or when model lead zero is matched to a different moving radar object. This fallback does not promote a new stationary point to leadTwo.

The process then assigns leadTwo to the nearest different measured moving radar point already overlapping the current model-path corridor, without waiting for a CUT-IN event. This in-path second lead may be farther than leadOne. If the motion track physically matching leadOne reaches at least 0.90 CUT-OUT probability, leadOne is released. The exiting physical identity remains latched and excluded from both lead roles while its CUT-IN probability remains zero and either CUT-OUT is at least 0.60 or its vehicle body still overlaps the current path. This prevents the outgoing body from being selected again just before overlap ends, while allowing the next eligible current-path object to become leadTwo. During this latch, a different measured moving object already in the current path may start leadTwo without the normal new-track history wait. The latch clears when that physical identity is no longer measured, gains CUT-IN evidence, or is both below 0.60 CUT-OUT and outside the current path. If leadOne otherwise disappears, the corresponding in-path moving radar point may immediately continue as leadTwo; duplicate suppression applies while leadOne is actually present, so both roles never publish the same vehicle together. A different OUT-to-IN object confirmed by the physical predictor for 0.25 seconds is also eligible, but unlike an already in-path second lead it must be strictly closer than leadOne. An outside vehicle in the same longitudinal row, within ±8 m of leadOne, is not promoted from path-proximity score alone: its two-second physical forecast must contain an actual vehicle-body entry into the corridor. This does not discard every side-by-side vehicle; it becomes eligible when a real entry trajectory appears. A selected leadTwo is sticky while the same sensor, track ID, physical continuity ID, and position continuity remain valid. A newly closer point on the same adjacent side may hide unselected points, but it does not hide this already selected and still measured leadTwo. It can stay leadTwo if its reported speed later approaches zero, but a newly observed stationary point cannot become leadTwo. When vision/front-radar matching recognizes that object as leadOne, it transfers from L2 to L1 instead of being published in both roles. The hold clears on that leadOne handoff, clear outward motion, range exit, discontinuity, or track-ID reuse; a missing track may reconnect for up to 0.75 seconds only when its predicted longitudinal and lateral positions still agree. All leadTwo candidates use the requested fixed 80 m forward limit.

Once corner-radar points have been observed, the mode uses only corner motion for the rest of that process run. A configuration with no corner points uses only `frontRadar` raw tracks. It does not switch between front and corner from frame to frame, and SCC is never a dPath-motion input. Front radar does not supply the same object quality as corner radar, so a new front-only predicted CUT-IN additionally requires at least `0.75 m/s` of sustained long-window inward dPath motion; this is independent of the probability sensitivity. When a corner motion track and a front-radar point are mutually nearest within fixed distance, lateral-position, and speed gates, the corner history still decides motion and CUT-IN state, but the published `dRel`, `vLead`, `aLead`, and `jLead` use the front-radar measurement. If that front point is leadOne, normal duplicate suppression prevents the same vehicle from also being leadTwo. An incorrect leadTwo can affect real deceleration, so enable this mode only on the same vehicle configuration after completing shadow validation.

The `leadLeft`, `leadRight`, and side lists used by lane-change assistance are also published from visible adjacent vehicles on that same motion sensor. A point below `|vLead| < 3 km/h` may supply its current position there, but it never builds dPath history or becomes predicted leadTwo.

Every published radar-backed lead keeps the measured `jLead`. Its per-track `aLeadTau` follows the same `RadarReactionFactor` setting, quiet-acceleration threshold, jerk threshold, and 0.45-second decay filter as conventional `radard`; it is not fixed merely because Radar Motion mode is active. This applies consistently to leadOne, leadTwo, side leads, and all side/center/CUT-IN lists.

<a id="lead-selection"></a>
## Lead selection and validation

The manager never runs both radar implementations together. With `RadarMotionMode=0`, only the conventional `openpilot.selfdrive.controls.radard` runs and its leadOne/leadTwo selection is unchanged. With `RadarMotionMode=1`, that process is stopped and only the independent `openpilot.selfdrive.carrot.radar.radard_dpath` runs. It calculates the normal front/SCC-to-vision leadOne or the vision-seeded continuous stationary leadOne first, calculates leadTwo with the physical predictor below, and publishes `radarState` directly. Front radar, SCC, and corner radar retain their source identity, and no learned radar-lead model is used.

The headless validator can report existing radard and the experimental physical predictor separately. The visual replay runs only the new independent controller and physical predictor. Its leadOne/leadTwo are calculated again from the logged model and radar inputs; recorded conventional-radard lead roles and CUT-IN markers are never imported. Replay does not change longitudinal control.

The shadow predictor:

- uses only `measured=true` radar points;
- uses motion points only from 5 m behind ego through 100 m ahead;
- aligns each replay radar point to the model-path timestamp with its measured relative velocity, then projects the point onto the actually measured same-time model-path polyline without extending its noisy terminal segment: `S` is arc distance along the centerline and `dPath` is signed normal distance from it; scope uses distance to that finite polyline, so a short or reversing path at a stop cannot create an artificial centerline through a distant side object;
- uses `modelV2.timestampEof` as the camera measurement time, the configured front-radar delay for front tracks, and one 50 ms radar cycle for corner-object measurement delay before this alignment;
- keeps only the ego lane and its immediate left/right lanes, using the fixed model-path-relative range `|dPath| <= 5.4 m`;
- on each adjacent side, keeps points closer than 5 m and the nearest visible vehicle at or beyond 5 m; a farther same-side vehicle is also retained when it is still closer than the current leadOne, while other occluded vehicles are excluded from detection; measured in-scope history is retained so a vehicle can be evaluated continuously when it becomes visible, and an already selected physically continuous leadTwo is protected from this display/detection occlusion until it transfers to leadOne or becomes physically invalid;
- uses only corner-radar motion when measured corner data is available for the log, otherwise uses `frontRadar` raw-track motion; SCC remains visible to existing radard but is not motion-predictor input, and the predictor does not switch sources on individual frames;
- requires a front-radar CUT-IN candidate to be at least 5 m ahead, and does not let a newly observed front-radar point inside 5 m start leadTwo merely because it overlaps the path; every new current-path leadTwo also waits for the sensor's minimum measured motion history, while an already selected, physically continuous leadTwo can remain sticky as it passes inside 5 m;
- uses front-radar longitudinal kinematics for a mutually matched corner object without mixing the two sensors' dPath histories;
- shows points below `|vLead| < 3 km/h` as position-only references and does not build or extrapolate motion history for them;
- never switches between lane center and model path from frame to frame;
- does not apply yaw-rate correction again after the radar point and path share the same timestamp and ego coordinate frame;
- verifies reused track IDs and short gaps using physical position and velocity continuity;
- maintains independent front and corner histories and parameters;
- forms a two-dimensional path-relative history from projected centerline progress `S` plus integrated ego travel and signed lateral offset `dPath`, rather than treating raw `dRel` as path distance;
- fits `dPath` against actual target progress in `S`, so ego-time lateral drift is not extrapolated when the target has little longitudinal progress;
- uses the long-window `(S, dPath)` motion vector and its angle relative to the model-path tangent for the prediction mean, limits confidence when a future extrapolation exceeds its observed spatial baseline, and uses short-window disagreement to increase curvature and uncertainty rather than forcing a turn;
- scales proximity evidence by how much the predicted inward displacement exceeds the measured path uncertainty, so static proximity or sub-noise adjacent-lane drift is not sufficient by itself;
- for corner radar, compares position-derived normal motion with the most recent 0.1 seconds of radar-reported lateral velocity, treats a stronger same-direction report as supporting the measured position trend, and lowers CUT-IN/CUT-OUT confidence when the current measurement or short dPath trend no longer supports an older inward trend, such as a reflection point migrating across a vehicle body;
- predicts future `dRel` and `dPath` at synchronized 0.5, 1.0, 1.5, and 2.0 second horizons; and
- reports CUT-IN and CUT-OUT probabilities separately.

The current path-overlap check includes ego and target vehicle half-widths; its base boundary is 1.8 m to either side of the model-path center. It continuously evaluates the probability that the future target body overlaps this area from its predicted center and uncertainty, instead of requiring that center to cross a separate fixed inner boundary before any score exists. A measured vehicle already overlapping the path is shown as current `IN`; a newly observed point that starts there is not treated as a new shadow CUT-IN. A new front point inside 5 m is shown as control-ineligible `NEAR-IN` instead. A physically tracked `OUT -> IN` crossing retains its pending entry evidence across the boundary, so the 0.25-second confirmation can finish after overlap begins. Only small path-state and confirmation hysteresis are used. The predictor has no per-route, per-vehicle, or scene-specific exceptions.

### PC replay

`radar_lead_validation_review.py` groups maintained cases by log and opens all 40 unique logs in sequence. Each window shows synchronized qcamera video and only the new controller's recalculated lead roles plus the physical predictor's points, trajectories, probabilities, and CUT-IN events. Recorded conventional-radard lead roles, CUT-IN points, and event markers are intentionally absent. At the end of one log the window closes and the next log opens automatically. `T` switches the saved processing mode between normal (prefer corner when present, otherwise front) and front-only (ignore corner input completely). `--motion-mode normal|front` selects the initial mode; `--front-only` remains a compatibility alias for front-only mode.

For older logs whose recorded corner tracks predate stable object IDs, replay reconstructs those IDs from raw corner CAN. A reconstructed point must have a raw measurement no more than 100 ms old; stale display-only extrapolations are not predictor input.

Playback pauses only when the physical predictor confirms a new control-eligible CUT-IN for 0.25 seconds. One physical continuity produces only one automatic pause even if the object briefly changes between L1 and L2; a physically discontinuous reuse of the same track ID can produce a new pause. The replay uses a readable Korean-capable font and Korean operator labels. Its distance plot covers -10 through 120 m, marks ego as a white point, encloses recalculated leadOne in an orange square, and encloses recalculated leadTwo in a yellow square. The right panel shows the current L1/L2 IDs separately from new CUT-IN events in the current frame, so a retained leadTwo remains visible after its entry event ends. The full-log continuity graph below the map plots recalculated leadOne distance in orange and leadTwo distance in yellow. Missing leads and unrelated track-ID changes break the line; a near-stationary ID handoff remains connected only when its adjacent distance, lateral position, and speed are physically continuous. The continuity graph expands across the window and shares the seek bar's exact horizontal time axis: clicking anywhere in the lead graph seeks both cursors to that time. Manually seeking on the bar or graph re-arms all later handled events, so resuming with Space pauses again at the following CUT-IN. Press `F` to show or hide measured front-radar points without changing the processing sensor. In the bird's-eye map, gray lines are model lane lines, the white dashed line is their displayed center, and the blue line is the model path used as the predictor's only corridor. The lane center is never substituted into the calculation. By default, every track's source-colored filled fading trail is exactly the path-relative `(S, dPath)` history used by the predictor. Gray hollow rings mean an unselected or control-ineligible future, including a new front point inside 5 m; green means current-path motion eligible for control selection or a sticky selected lead, and orange means a confirmed predictor CUT-IN. `H` shows or hides these predictor histories and futures. `A` separately overlays the ego-motion-stabilized raw radar `(xRel, yRel)` history in gray for diagnosis. That optional overlay is observation-derived, not ground-truth target motion, and is not the prediction history. Ego yaw is used only to align this optional raw overlay; it is not applied again to synchronized `dPath`. The horizontal seek bar marks confirmed predictor CUT-IN entries in orange and maintained validation windows above the bar; it has no existing-radard markers. The radar map is an ego-coordinate view, not a perspective overlay on qcamera. Use Space to pause, Left/Right to seek by key, Up/Down to change playback speed, `M` to show or hide predictor CUT-IN markers, and `R` to restart and re-arm handled predictor pauses. `I`, `C`, and `S` apply CUT-IN, CLEAR, or STATIONARY labels. Inside a maintained validation window they update that case; outside those windows a label is stored in `radar_trajectory_labels.json`.

The `CUT-IN path-proximity sensitivity` slider covers 0.20 through 0.80. A lower value is more sensitive. Validation and production share sensor defaults of 0.30 for corner and 0.67 for front radar. The two replay values are stored independently in the user's local `carrotpilot/radar_validation.json`; switching with `T` immediately restores that sensor's last replay value. The score combines uncertainty-aware vehicle-body overlap with normalized physical clearance from the ego-path corridor. Clearance falls from full evidence at overlap to zero 1.2 m outside the vehicle-width-aware boundary, then motion consistency and future-history support are applied. Thus 0.20 can react earlier while approaching the boundary, while 0.80 requires the predicted body to be much closer to or overlap it. Lowering it does not bypass measured-history length, inward path motion, 0.25-second confirmation, range, speed, or leadOne-priority checks. While dragging, the slider distinguishes the requested value from the currently applied value. Releasing it immediately reevaluates only temporal confirmation and leadTwo selection from the selected sensor's already-built physical trajectories and leadOne history; it does not rebuild dPath motion. Completion reports CUT-IN entries and leadTwo frames. `--prob` is a one-run replay override for the selected sensor when a fixed comparison value is needed. The saved slider value and `--prob` do not change the fixed production sensor thresholds, physical equations, conventional radard, or stored labels.

An arbitrary log outside the maintained list can be opened directly as a module. Replay automatically uses `qcamera.ts` from the same segment directory.

```powershell
.venv\Scripts\python.exe -m openpilot.selfdrive.carrot.radar.tools.radar_lead_simulator `
  "W:\routes\vehicle\segment\rlog.zst" --start 30 --paused
```

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
