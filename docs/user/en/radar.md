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

### Carrot Radar Mode

| `CarrotRadarMode` | Meaning |
|---:|---|
| `0` | Keep the existing `radard` lead and cut-in processing (default) |
| `1` | Run the independent dPath RadarD, calculate vision-supported radar leadOne first, then use only physical dPath CUT-INs as leadTwo |

On vehicles with neither corner-radar nor radar-track support, this behaves the same as the existing mode.

Manager refreshes the setting while off-road and latches it for the whole drive when the next OnRoad session starts. After changing it, end the current drive and restart the vehicle or reboot the device. A live parameter write does not immediately replace one radar publisher with the other.

Mode `1` does not use a learned model, call `controls/radard.py`, or mix in its output. Following the order used by the removed `radard_model.py`, the independent process first matches model lead zero to front/SCC radar and assigns leadOne. A fresh moving leadOne match must also meet the conventional radard's `1e-4` minimum joint distance/lateral/velocity likelihood; its longitudinal gate includes up to `2.5 * xStd` (capped at 15 m) so a high-confidence existence estimate with uncertain visual range can still match the correct radar object. An already continuous identity may tolerate a brief lower score. A separate near-stationary leadOne fallback accepts a measured in-path radar object with `|vLead| <= 2.5 m/s` only after model lead zero first reaches at least 0.40 probability at a consistent position, its reported speed differs from the radar object by no more than 10 m/s, and the radar remains physically continuous for 0.25 seconds. A front-radar stationary seed additionally needs three consecutive vision-supported frames; a corner seed needs one. This prevents a brief low-probability road or infrastructure hypothesis from becoming an indefinitely retained stationary lead. Once confirmed, the same stationary object remains leadOne through weak or missing vision and physically continuous radar-reflection ID handoffs. It releases on physical discontinuity or when model lead zero is matched to a different moving radar object. This fallback does not promote a new stationary point to leadTwo.

The process then assigns leadTwo to the nearest different measured moving radar point already overlapping the current model-path corridor, without waiting for a CUT-IN event. Every leadTwo candidate—including current-path motion, predicted CUT-IN, and sticky retention—must be strictly closer than an actual leadOne whenever leadOne exists. A predicted CUT-OUT never removes leadOne, latches its identity out, or excludes a candidate from either control role. CUT-OUT remains available only as diagnostic output. LeadOne changes only when the primary model-to-front/SCC-radar matcher loses the match or switches to another object; this conservative behavior may retain a departing vehicle briefly, but prevents an uncertain future-path prediction from creating a no-lead acceleration gap while the vehicle still occupies the path. If leadOne otherwise disappears, the corresponding in-path moving radar point may immediately continue as leadTwo; duplicate suppression applies while leadOne is actually present, so both roles never publish the same vehicle together. A different OUT-to-IN object is eligible under the same distance-priority rule only when its five-second physical forecast contains at least 0.5 seconds of continuous vehicle-body overlap with the corridor. Normal temporal confirmation is 0.25 seconds, but a longer stable future overlap progressively reduces it to no less than 0.10 seconds. For a corner-radar vehicle within 5 m, confirmation is also shortened to 0.10 seconds only when its body-aware path clearance is at most 0.45 m and its short/long dPath histories and current radar lateral velocity all agree inward. This is not a general near-range exception; it applies only to an already near-boundary vehicle making a consistent rapid entry. An outside vehicle in the same longitudinal row, within ±8 m of leadOne, is not promoted from uncertainty or a one-sample path-proximity score alone. This does not discard every side-by-side vehicle; it becomes eligible when a sustained physical entry trajectory appears. A selected leadTwo is sticky while the same sensor, track ID, physical continuity ID, and position continuity remain valid. A newly closer point on the same adjacent side may hide unselected points, but it does not hide this already selected and still measured leadTwo. It can stay leadTwo if its reported speed later approaches zero, but a newly observed stationary point cannot become leadTwo. When vision/front-radar matching recognizes that object as leadOne, it transfers from L2 to L1 instead of being published in both roles. The hold clears on that leadOne handoff, clear outward motion, range exit, discontinuity, or track-ID reuse; a missing track may reconnect for up to 0.75 seconds only when its predicted longitudinal and lateral positions still agree. All leadTwo candidates use the requested fixed 80 m forward limit.

Once corner-radar points have been observed, the mode uses only corner motion for the rest of that process run. A configuration with no corner points uses only `frontRadar` raw tracks. It does not switch between front and corner from frame to frame, and SCC is never a dPath-motion input. Front radar does not supply the same object quality as corner radar, so a new front-only predicted CUT-IN normally additionally requires at least `0.75 m/s` of sustained long-window inward dPath motion; this is independent of the probability sensitivity. A lower long-window rate is accepted when the latest 0.8-second history has at least 0.20 m of net inward progress, net progress divided by total back-and-forth travel is at least 0.75, at least 65% of measured steps and both short/long dPath rates point inward, and the future forecast has at least 1.0 second of continuous corridor overlap. The raw positions are not median-smoothed; only this clearly one-way case blends the short- and long-window rates equally so the future trajectory responds sooner. A front point already overlapping the current corridor at 5–10 m may also pass with a lower long-window rate only when its short-window, long-window, and current radar-reported lateral velocities all independently agree inward. When a corner motion track and a front-radar point are mutually nearest within fixed distance, lateral-position, and speed gates, the corner history still decides motion and CUT-IN state, but the published `dRel`, `vLead`, `aLead`, and `jLead` use the front-radar measurement. An established mutual sensor association is retained with wider hysteresis while both track identities and their position and speed continuity remain compatible. This covers front/rear reflection migration on a long vehicle without treating every newly observed pair within the wider range as one object. A corner object may also receive vision-existence support when its matched front range is below the uncertain vision range, that vision range is below leadOne, radar and vision speeds agree, and at least one second of both short- and long-window dPath history moves inward. Vision support is applied only after the physical forecast already contains the required continuous corridor overlap, so vision alone cannot create a CUT-IN. It never averages visual range into control: front radar still supplies longitudinal control values and corner radar still supplies dPath motion. If that front point is leadOne, normal duplicate suppression prevents the same vehicle from also being leadTwo. An incorrect leadTwo can affect real deceleration, so enable this mode only on the same vehicle configuration after completing shadow validation.

The `leadLeft`, `leadRight`, and side lists used by lane-change assistance are also published from visible adjacent vehicles on that same motion sensor. Consistent with the stationary-object boundary, a point at `|vLead| <= 2.5 m/s` may supply its current position there, but it does not start new dPath history or become predicted leadTwo. An already selected leadTwo that later slows below this speed remains sticky while the same physical object and position continuity remain valid.

Every published radar-backed lead keeps the measured `jLead`. Its per-track `aLeadTau` follows the same `RadarReactionFactor` setting, quiet-acceleration threshold, jerk threshold, and 0.45-second decay filter as conventional `radard`; it is not fixed merely because Radar Motion mode is active. This applies consistently to leadOne, leadTwo, side leads, and all side/center/CUT-IN lists.

<a id="lead-selection"></a>
A side or rear reflection on a long vehicle can pass behind ego before the reflected point's future `dRel > 0` trajectory reaches the corridor. To cover that geometry, a corner point 0.8–5 m ahead may qualify as a `near-side directional entry` when it is within 0.85 m of the normal body boundary, its latest 0.8 seconds contain at least 0.20 m net inward progress, directionality is at least 0.90, at least 75% of measured steps move inward, short/long inward rates are at least 0.25/0.15 m/s, and both radar lateral-motion agreement and recent-motion support are at least 0.90. This already represents sustained measured history, so it confirms without another frame dwell. It does not infer a vehicle class or generally admit close points; it requires a near-boundary measured object moving strongly and consistently inward.

When a target center is within 2.7 m of the model path, its assumed 0.9 m half-width is already straddling the nominal 1.8 m ego-lane boundary. A corner target in that state qualifies as a `lane-boundary directional entry` only when its latest 0.8-second net displacement, directionality, inward sample ratio, short/long dPath rates, and radar-reported lateral velocity all consistently point inward. This current-state rule detects a real lane intrusion before the target and ego bodies overlap at the tighter 1.8 m control corridor. Mere boundary proximity or model-path movement is insufficient; sustained measured position history and the corner-radar velocity direction must agree.

## Lead selection and validation

The manager never runs both radar implementations together. With `CarrotRadarMode=0`, only the conventional `openpilot.selfdrive.controls.radard` runs and its leadOne/leadTwo selection is unchanged. With `CarrotRadarMode=1`, that process is stopped and only the independent `openpilot.selfdrive.carrot.radar.radard_dpath` runs. It calculates the normal front/SCC-to-vision leadOne or the vision-seeded continuous stationary leadOne first, calculates leadTwo with the physical predictor below, and publishes `radarState` directly. Front radar, SCC, and corner radar retain their source identity, and no learned radar-lead model is used.

The headless validator can report existing radard and the experimental physical predictor separately. The visual replay runs only the new independent controller and physical predictor. Its leadOne/leadTwo are calculated again from the logged model and radar inputs; recorded conventional-radard lead roles and CUT-IN markers are never imported. Replay does not change longitudinal control.

The shadow predictor:

- uses only `measured=true` radar points;
- uses motion points only from 5 m behind ego through 100 m ahead;
- aligns each replay radar point to the model-path timestamp with its measured relative velocity, then projects the point onto the actually measured same-time model-path polyline without extending its noisy terminal segment: `S` is arc distance along the centerline and `dPath` is signed normal distance from it; scope uses distance to that finite polyline, so a short or reversing path at a stop cannot create an artificial centerline through a distant side object;
- uses `modelV2.timestampEof` as the camera measurement time, the configured front-radar delay for front tracks, and one 50 ms radar cycle for corner-object measurement delay before this alignment;
- keeps only the ego lane and its immediate left/right lanes, using the fixed model-path-relative range `|dPath| <= 5.4 m`;
- retains measured history throughout `|dPath| <= 5.4 m`, but starts a new predicted CUT-IN only after the current position reaches `|dPath| <= 3.0 m`, within 1.2 m of physical vehicle-body/path overlap. A point on the outer side of an adjacent lane or entering that lane from the second adjacent lane is therefore not promoted merely because a long forecast eventually reaches the ego path; once it physically approaches, the already retained history is immediately available;
- on each adjacent side, keeps points closer than 5 m and the nearest visible vehicle at or beyond 5 m; a farther same-side vehicle is also retained when it is still closer than the current leadOne, while other occluded vehicles are excluded from detection; measured in-scope history is retained so a vehicle can be evaluated continuously when it becomes visible, and an already selected physically continuous leadTwo is protected from this display/detection occlusion until it transfers to leadOne or becomes physically invalid;
- uses only corner-radar motion when measured corner data is available for the log, otherwise uses `frontRadar` raw-track motion; SCC remains visible to existing radard but is not motion-predictor input, and the predictor does not switch sources on individual frames;
- normally requires a front-radar CUT-IN candidate to be at least 5 m ahead, and does not let a newly observed front-radar point inside 5 m start leadTwo merely because it overlaps the path. A 2–5 m point may qualify only when the same physical identity was measured continuously from outside the corridor through a real `OUT -> IN` boundary crossing, with at least 0.40 m net inward progress, 0.90 directionality, 70% inward measured steps, and independently agreeing short/long dPath rates and radar-reported lateral velocity. This does not apply to a reflection first born inside 5 m. Every new current-path leadTwo also waits for the sensor's minimum measured motion history, while an already selected, physically continuous leadTwo can remain sticky as it passes inside 5 m;
- uses front-radar longitudinal kinematics for a mutually matched corner object without mixing the two sensors' dPath histories;
- shows points at `|vLead| <= 2.5 m/s` as position-only references and does not build or extrapolate new motion history for them; an already selected leadTwo may still retain through later deceleration under the normal sticky continuity rule;
- never switches between lane center and model path from frame to frame;
- does not apply yaw-rate correction again to `dPath` after the radar point and path share the same timestamp and ego coordinate frame; corner-radar `vLead`/`yvRel` are still velocities reported in the rotating ego frame, so only their yaw-rotation component is removed before comparing them with model-path-normal velocity, without moving measured `dPath` positions or history;
- verifies reused track IDs and short gaps using physical position and velocity continuity;
- maintains independent front and corner histories and parameters;
- forms a two-dimensional path-relative history from projected centerline progress `S` plus integrated ego travel and signed lateral offset `dPath`, rather than treating raw `dRel` as path distance;
- fits `dPath` against actual target progress in `S`, so ego-time lateral drift is not extrapolated when the target has little longitudinal progress;
- uses the long-window `(S, dPath)` motion vector and its angle relative to the model-path tangent for the prediction mean, limits confidence when a future extrapolation exceeds its observed spatial baseline, and normally uses short-window disagreement to increase curvature and uncertainty; only a front track whose 0.8-second net progress, directionality, inward-step ratio, and short/long inward rates all pass their physical gates blends half of the short-window rate into the future trajectory;
- scales proximity evidence by how much the predicted inward displacement exceeds the measured path uncertainty, so static proximity or sub-noise adjacent-lane drift is not sufficient by itself;
- for corner radar, compares position-derived normal motion with the most recent 0.1 seconds of radar-reported lateral velocity, treats a stronger same-direction report as supporting the measured position trend, and lowers CUT-IN/CUT-OUT confidence when the current measurement or short dPath trend no longer supports an older inward trend, such as a reflection point migrating across a vehicle body;
- predicts future `dRel` and `dPath` at synchronized 0.5-second steps through 5.0 seconds;
- when leadOne exists, compares both vehicles at the candidate's first predicted corridor-overlap time; if constant-relative-velocity prediction does not place the candidate at least 2 m clearly closer than leadOne, it is not entering the space between ego and leadOne and cannot become a CUT-IN leadTwo; and
- reports CUT-IN and CUT-OUT probabilities separately.

The current path-overlap check includes ego and target vehicle half-widths; its base boundary is 1.8 m to either side of the model-path center. The existing 0.12 m path-state hysteresis is also applied to predicted overlap so a boundary sample does not chatter between states. A predicted CUT-IN needs at least two consecutive 0.5-second future samples inside that boundary, which is 0.5 seconds of continuous predicted body/corridor overlap. Uncertainty and proximity still scale the score inside that sustained overlap, but cannot create an entry outside the corridor by themselves. A measured vehicle already overlapping the path is shown as current `IN`; a newly observed point that starts there is not treated as a new shadow CUT-IN. A new front point inside 5 m is shown as control-ineligible `NEAR-IN` instead. Only a close front point meeting the strict measured-direction and physical boundary-crossing rule above is shown as `TRACKED-IN`. A physically tracked `OUT -> IN` crossing retains its pending entry evidence across the boundary, so confirmation can finish after overlap begins. Normal confirmation is 0.25 seconds; stable predicted overlap shortens it toward 0.10 seconds, and a close boundary crossing already backed by sufficient one-way history confirms without another dwell. Only small path-state and confirmation hysteresis are used. The predictor has no per-route, per-vehicle, or scene-specific exceptions.

One-sample proximity still cannot create an entry outside the corridor. The only supplement is the strict 0.8-second measured `near-side directional entry` history above; its immediate confirmation is safe only because the sustained-history requirement has already been met.

### PC replay

`radar_lead_validation_review.py` groups maintained cases by log and opens all 40 unique logs in sequence. Each window shows synchronized qcamera video and only the new controller's recalculated lead roles plus the physical predictor's points, trajectories, probabilities, and CUT-IN events. Recorded conventional-radard lead roles, CUT-IN points, and event markers are intentionally absent. At the end of one log the window closes and the next log opens automatically. `T` switches the saved processing mode between normal (prefer corner when present, otherwise front) and front-only (ignore corner input completely). `--motion-mode normal|front` selects the initial mode; `--front-only` remains a compatibility alias for front-only mode.

For older logs whose recorded corner tracks predate stable object IDs, replay reconstructs those IDs from raw corner CAN. A reconstructed point must have a raw measurement no more than 100 ms old; stale display-only extrapolations are not predictor input.

Playback pauses only when the physical predictor confirms a new control-eligible CUT-IN after the applicable dynamic 0.10–0.25-second confirmation. One physical continuity produces only one automatic pause even if the object briefly changes between L1 and L2; a physically discontinuous reuse of the same track ID can produce a new pause. The replay uses a readable Korean-capable font and Korean operator labels. Its distance plot covers -30 through 120 m, marks ego as a white point, encloses recalculated leadOne in an orange square, and encloses recalculated leadTwo in a yellow square. The right panel shows the current L1/L2 IDs separately from new CUT-IN events in the current frame, so a retained leadTwo remains visible after its entry event ends. Each candidate reports both its continuous predicted-overlap duration/start, such as `침범=1.5s@+3.0s`, and recent net inward progress/directionality, such as `방향=0.28m/0.85`. The full-log continuity graph below the map plots recalculated leadOne distance in orange and leadTwo distance in yellow. Missing leads and unrelated track-ID changes break the line; a near-stationary ID handoff remains connected only when its adjacent distance, lateral position, and speed are physically continuous. The continuity graph expands across the window and shares the seek bar's exact horizontal time axis: clicking anywhere in the lead graph seeks both cursors to that time. Manually seeking on the bar or graph re-arms all later handled events, so resuming with Space pauses again at the following CUT-IN. Press `F` to show or hide measured front-radar points without changing the processing sensor. In the bird's-eye map, gray lines are model lane lines, the white dashed line is their displayed center, and the blue line is the model path used as the predictor's only corridor. The lane center is never substituted into the calculation. By default, every track's source-colored filled fading trail is exactly the path-relative `(S, dPath)` history used by the predictor. Gray hollow rings mean an unselected or control-ineligible future, including a new front point inside 5 m; green means current-path motion eligible for control selection or a sticky selected lead. Orange covers the complete continuous corridor-overlap interval of a confirmed predictor CUT-IN; it does not turn gray at a farther horizon merely because uncertainty lowers that individual horizon's score. `H` shows or hides these predictor histories and futures. `A` separately overlays the ego-motion-stabilized raw radar `(xRel, yRel)` history in gray for diagnosis. That optional overlay is observation-derived, not ground-truth target motion, and is not the prediction history. Ego yaw is used only to align this optional raw overlay; it is not applied again to synchronized `dPath`. The horizontal seek bar marks confirmed predictor CUT-IN entries in orange and maintained validation windows above the bar; it has no existing-radard markers. The radar map is an ego-coordinate view, not a perspective overlay on qcamera. Use Space to pause, Left/Right to seek by key, Up/Down to change playback speed, `M` to show or hide predictor CUT-IN markers, and `R` to restart and re-arm handled predictor pauses. `I`, `C`, and `S` apply CUT-IN, CLEAR, or STATIONARY labels. Inside a maintained validation window they update that case; outside those windows a label is stored in `radar_trajectory_labels.json`.

The `future lookahead` slider covers 3.5 through 5.0 seconds; a larger value accepts a farther-future entry earlier. The predictor builds the trajectory once at 0.5-second steps through 5.0 seconds, then accepts CUT-IN evidence only when the vehicle body overlaps the corridor continuously for at least 0.5 seconds inside the selected lookahead. A single grazing future sample or uncertainty alone touching the corridor is not sufficient. Longer continuous overlap progressively reduces temporal confirmation from 0.25 seconds toward a 0.10-second minimum. Corner and front lookahead values are stored independently in the user's local `carrotpilot/radar_validation.json`; switching with `T` immediately restores that sensor's last value. While dragging, the slider distinguishes the requested value from the currently applied value. Releasing it reuses the cached five-second physical trajectories and leadOne history and reevaluates only temporal confirmation and leadTwo selection, avoiding the previous delay from rebuilding dPath history. Completion reports CUT-IN entries and leadTwo frames. `--lookahead-s 3.5` supplies a one-run lookahead override. `--prob` remains only as an advanced one-run probability-threshold override for fixed comparisons. The replay time control does not change production's fixed five-second prediction, corner 0.30/front 0.67 thresholds, conventional radard, or stored labels.

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
