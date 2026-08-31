# Radar cut-in validation

The maintained validation sources are:

- `cutin_validation_cases.json`: control-facing CUT-IN, CLEAR, and STATIONARY windows.
- `radar_trajectory_labels.json`: point-level CUT-IN and CLEAR review labels.

Both files are validation-only. Do not use their values to fit a model, tune the
physical equations, select thresholds, or add scene-specific exceptions.

## Current architecture

`RadarLeadModelMode`, `CarrotRadarMode`, the learned radar-lead/path-occupancy
runtimes, and the former `openpilot/selfdrive/controls/radard.py` implementation
have been removed. The manager always starts the production `radard` process
from `openpilot/selfdrive/carrot/radar/radard_dpath.py`. It first calculates
the normal front/SCC vision-matched `leadOne` or a vision-seeded physically
continuous stationary `leadOne`, then supplies `leadTwo` from either a
different measured moving point already in the current path or a physically
confirmed dPath CUT-IN. A third path may expose a central stopped front-radar
return behind a moving lead that is cutting out, but only when an independent
slow corner-radar return agrees in range, model-path position, and speed for
the full confirmation interval. Front-only returns and nearby moving corner
returns cannot start this stationary `leadTwo`. After confirmation, the same
state-2 front identity remains eligible through corner dropouts while its
reported position stays physically continuous; it cannot transfer that trust
to a new or reused track. A central slow corner point that was itself
vision-confirmed as leadOne may likewise hand off to leadTwo when the primary
matcher switches to a farther object; this support expires after one second
unless leadTwo has already retained the same physically continuous identity.
An in-path second lead may be
beyond leadOne, while a predicted CUT-IN must be closer than leadOne. It rejects
the primary object
itself and every candidate beyond the fixed 80 m limit. An outside candidate
within ±8 m longitudinally of leadOne also needs an actual corridor-entry
sample in its two-second forecast; proximity alone cannot promote a same-row
vehicle. Front, SCC, and corner inputs retain their production source identity.
CUT-OUT never removes leadOne, latches an identity out, or excludes either
control-lead candidate. Its only control use is arming the cross-sensor-confirmed
stationary `leadTwo` path above. LeadOne changes only when the primary
model-to-front/SCC-radar matcher loses its match or switches to another object.
This intentionally prefers a brief conservative hold on a departing vehicle
over a no-lead acceleration gap caused by an uncertain future-path prediction.
A separate measured moving object already in the current path still needs the
normal motion history before starting leadTwo.

Before normal CUT-IN confirmation, a separate corner-radar-only risk path may
request bounded pre-deceleration without creating leadTwo. It requires 0.10
seconds of the same closing track, `8 <= dRel <= 45 m`, `2.4 <= |dPath| <=
3.4 m`, longitudinal closing speed of at least `1.5 m/s`, TTC no greater than
8 seconds, and mutually agreeing short/long position history plus reported
lateral velocity. The measured inward displacement must be at least 0.35 m,
directional consistency at least 0.90, and inward sample ratio at least 0.80.
The longitudinal planner converts this signal to only a `-0.25` through
`-0.65 m/s^2` acceleration ceiling with a bounded ramp. It is disabled while
longitudinal control is inactive or the driver presses the accelerator. SCC
requests and `objDist` are not inputs to this path.

The stationary leadOne path requires a measured in-path point with
`|vLead| <= 4.0 m/s`, model-lead support at probability 0.40 or higher, and
0.25 seconds of physical continuity. A front-only stationary point needs three
vision-supported frames. A radar point that needs the broader immediate
vision-correlated recovery path must already have been measured continuously
for 0.25 seconds; brief infrastructure reflections remain pending instead of
borrowing a nearby visual lead. A central radar-only corner/SCC point instead
uses a 0.50 second confirmation and a narrow center gate. When a front point and a
corner point are mutually consistent within 5 m longitudinally, 0.75 m raw
lateral position, and 2.5 m/s target speed, a high-uncertainty visual lead may
seed the front point despite up to 30 m visual range error. The front point
must still satisfy the 4.0 m/s stationary limit; the corroborating corner point
may be as high as 6.0 m/s to tolerate corner velocity noise. Once seeded, weak
vision may be bridged only while the same measured pair remains physically
continuous. A strong visual lead outside its uncertainty gate or a broken
radar pair releases the hold. This path never creates leadTwo. A fresh moving
leadOne also needs at least `1e-4` joint distance/lateral/velocity likelihood,
matching conventional radard's score floor; an already continuous identity
may tolerate a brief lower score.

Hyundai CAN-FD group-2 front-radar slots retain the radar's categorical
`VALID` state in `RadarPoint.trackState`. A state-1 tentative object remains
available for vision matching, corner/physical CUT-IN corroboration, and later
state-2/3 promotion. With no independent existence evidence it needs 0.75
seconds of continuous radar-only observation instead of the normal 0.25
seconds before becoming a moving leadOne. This still admits a persistent
vision-invisible target while rejecting the maintained state-1 overpass
reflection, which disappears after 0.66 seconds in `liveTracks`. Sources
without this metadata use state 0 and preserve their existing behavior. PC
replay reconstructs the state from raw CAN for older logs before lead
selection.

PC visual replay runs only the new `DPathRadarController` and
`RadarMotionPredictor`. Its lead roles are recalculated from logged model and
radar inputs; it does not import or display recorded conventional-radard lead
roles or CUT-IN events. The headless validator may compute existing-radard
metrics separately, but those values are never input to the physical predictor.
Replay first orders rlog messages by `logMonoTime`, advances predictor history
on the logged `modelV2.timestampEof` exposure clock used by production RadarD,
and prefers recorded stable corner `liveTracks`. Raw-CAN corner reconstruction
is used only when the corresponding recorded corner group is absent.

When an older log needs raw-CAN corner-track reconstruction, reconstructed
points older than 100 ms are excluded from predictor input. The longer display
tracker hold must never create a synthetic motion candidate.

The predictor:

1. accepts only `measured=true` points;
2. limits motion prediction to `-5 <= dRel <= 100 m`;
3. projects replay points to the model-path timestamp with measured relative
   velocity, using `modelV2.timestampEof`, the configured front-radar delay,
   and a 50 ms corner-object measurement delay, then projects them onto that
   actually measured same-time model-path polyline without extending a noisy
   terminal segment; `S` is centerline arc distance and `dPath` is signed
   centerline-normal distance. Scope also uses distance to that finite
   polyline, so a short or reversing path at a stop cannot pull a distant
   side object onto an artificial centerline;
4. limits history and shadow candidates to the ego lane plus the immediate
   left/right lanes with the fixed model-path-relative
   `|dPath| <= 5.4 m` range;
5. keeps points inside 5 m and the nearest point at or beyond 5 m on each
   adjacent side; a farther same-side point is also visible when it remains
   closer than the current leadOne, while other occluded points are excluded
   from detection and retain only their in-scope physical history; an already
   selected, still-measured, physically continuous leadTwo is protected from
   this occlusion until it transfers to leadOne or becomes physically invalid;
6. uses corner motion only when the log has measured corner data, otherwise
   `frontRadar` raw-track motion; setting-permitted SCC remains available to
   leadOne but is not CUT-IN predictor input, and the source choice never
   switches per frame;
7. treats `|vLead| < 3 km/h` as position-only and never builds or extrapolates
   motion history for those points;
8. requires front-radar CUT-IN detection at `dRel >= 5 m`, prevents a newly
   observed front point inside 5 m from starting current-path leadTwo, and
   requires the sensor's minimum measured motion history before any new
   current-path point starts leadTwo, while still allowing an already selected
   physically continuous leadTwo to remain sticky inside that distance;
9. does not switch to lane center and does not apply yaw correction again
   after the point and path share a timestamp and ego frame;
10. checks track-ID reuse and short gaps with physical continuity;
11. keeps front and corner histories and parameters independent; when a
   corner object and front point are mutually nearest within fixed
   longitudinal, lateral, and speed gates, the corner history supplies motion
   state while published longitudinal kinematics use the front measurement;
12. builds a 2-D path-relative history from projected centerline progress `S`,
   integrated ego travel, and signed-normal `dPath`, without treating raw
   `dRel` as centerline distance;
13. fits `dPath` against target progress in `S`, uses the long-window vector
   and model-path tangent angle error as the prediction mean, reduces
   confidence beyond the observed spatial baseline, and sends short-window
   disagreement to curvature and uncertainty;
14. scales path-proximity evidence by inward displacement relative to measured
   path uncertainty, so static proximity or sub-noise drift is insufficient;
15. for corner radar, checks position-derived normal motion against the most
   recent 0.1 seconds of reported lateral velocity; a strongly established
   one-way position history uses narrower hold hysteresis so small directional
   metric jitter cannot reset CUT-IN confirmation, while loss of recent inward
   rate or raw lateral support still promptly lowers confidence;
16. for front-only radar, requires a new predicted CUT-IN to sustain at least
   `0.75 m/s` of inward long-window `dPath` motion independently of the
   path-proximity sensitivity, and prevents forecast-only control until the
   measured target reaches `|dPath| <= 2.20 m`; this keeps coarse azimuth
   steps from promoting a parallel adjacent vehicle;
17. predicts synchronized future `dRel` and `dPath`;
18. confirms a threshold crossing for 0.25 seconds before producing a CUT-IN
   event; and
19. reports CUT-IN and CUT-OUT probabilities independently; and
20. reports the stricter early corner-risk signal separately from confirmed
    CUT-IN, so it cannot enter the normal leadTwo/MPC obstacle path.

A confirmed corner candidate that is still outside `|dPath| = 2.20 m` may
enter the normal leadTwo path only with at least `0.65 m/s` long-window inward
motion or `3.0 m/s` longitudinal closing speed. Weaker outside motion remains
an adjacent candidate instead of causing full lead control. This gate does not
change the separate bounded pre-deceleration path.

The current IN state includes ego and target vehicle half-widths. A tracked
OUT-to-IN crossing keeps its pending entry evidence after overlap begins so
the confirmation interval can complete. A point first observed inside has no
such entry evidence. The corner-radar lane-boundary straddle path requires at
least 0.35 m of measured inward displacement so a parallel adjacent vehicle's
small quantized lateral shift cannot create a boundary-only CUT-IN. The closer
near-side emergency path retains its separate 0.20 m displacement requirement.
Only small path-state and confirmation hysteresis are allowed. Do not add per-route,
per-vehicle, or scene-specific exceptions in response to a poor validation
case; report the underlying history, continuity, geometry, or uncertainty
failure instead.

## Headless full replay

From the repository root:

```powershell
python openpilot/selfdrive/carrot/validate_radar_lead_model.py
```

The historical script name is retained for operator compatibility. It no
longer loads a learned model. By default it replays both maintained JSON files
from `W:\routes`, reports existing-radard and physical-shadow metrics
separately, and does not fail on shadow regressions.

Useful options:

```powershell
# Maintained control cases only
python openpilot/selfdrive/carrot/validate_radar_lead_model.py --cases-only

# One named case
python openpilot/selfdrive/carrot/validate_radar_lead_model.py --case carnival-5b-18-early

# Fast predictor iteration: skip the unchanged existing-radard replay
python openpilot/selfdrive/carrot/validate_radar_lead_model.py --shadow-only

# Treat existing-radard expectation failures as a nonzero result
python openpilot/selfdrive/carrot/validate_radar_lead_model.py --strict-radard

# Treat physical dPath expectation failures as a nonzero result
python openpilot/selfdrive/carrot/validate_radar_lead_model.py --strict-shadow

# Check only early corner pre-deceleration detections and clear-window safety
python openpilot/selfdrive/carrot/validate_radar_lead_model.py --shadow-only --strict-predecel

# Remove corner inputs
python openpilot/selfdrive/carrot/validate_radar_lead_model.py --front-only
```

`--shadow-only` still loads every maintained rlog and runs the complete
physical dPath controller. It skips only the separate existing-radard CUT-IN
reconstruction, which otherwise rereads each rlog once per requested radar
source. Use it while iterating on `radar_motion`; run the normal command before
final integration when the existing-radard comparison is also required.

An optional `--report PATH` writes a validation replay report. Reports are
diagnostic output, not a production artifact.

## Visual replay

List or open maintained cases:

```powershell
python openpilot/selfdrive/carrot/radar_lead_validation_review.py --list
python openpilot/selfdrive/carrot/radar_lead_validation_review.py --case carnival-5b-18-early
```

The screen shows only new-controller and physical-predictor data:

- synchronized qcamera video;
- a -30 through 130 m distance view with ego as a white point, recalculated
  leadOne in an orange square, and recalculated leadTwo in a yellow square;
- measured front points as an optional `F`-key overlay and corner points with
  their source identity;
- source-colored `(S, dPath)` history actually consumed by the predictor and
  its 0.5/1.0/1.5/2.0-second future paths;
- an optional `A`-key gray overlay of ego-motion-stabilized raw radar history
  (observation-derived, not predictor input or target-motion ground truth);
- gray model lane lines, a display-only white dashed lane center, and the blue
  model path that remains the predictor's only corridor;
- current IN, CUT-IN, and CUT-OUT probabilities; and
- short/long `dPath` rate, curvature, uncertainty, and continuity ID;
- a full-log continuity graph of recalculated leadOne distance in orange,
  leadTwo distance in yellow, and the first raw model vision lead as a blue
  `V` whenever its probability is at least 0.4. Missing leads and unrelated
  track-ID changes break the line; near-stationary handoffs remain connected
  only when adjacent distance, lateral position, and speed are physically
  continuous. It spans the window at twice the previous height, its horizontal
  time axis exactly matches the seek bar, and
  clicking the graph seeks both cursors;
- a clickable seek bar with physical-predictor CUT-IN entry markers in orange
  and validation windows above it. Existing-radard markers are absent.

The UI uses a Korean-capable font and Korean operator labels. Gray future rings
are unselected or control-ineligible motion, including new front points inside
5 m. Green means control-eligible current-IN motion or a sticky selected lead.
Orange points, rings, and timeline markers mean confirmed
physical-predictor CUT-IN only. One physical continuity creates at most one
automatic pause even if it briefly moves between lead roles; a physically
discontinuous reuse of the same track ID may create another pause.

With no filters, the maintained cases cover 49 unique logs. They open in
sequence, and finishing one log automatically opens the next.

Controls:

- Space: pause/resume.
- Left/Right: seek.
- Up/Down: playback speed.
- Mouse click on the horizontal bar: seek directly.
- `H`: show/hide continuity-local measured-history trails and synchronized
  0.5/1.0/1.5/2.0-second future trajectories.
- `A`: show/hide the separate raw-radar observation overlay. It is off by
  default and is never used as the predictor's displayed input history.
- `F`: show/hide current measured front-radar points without changing the
  selected motion sensor or predictor inputs.
- `T`: switch the saved processing mode. Normal mode prefers corner radar when
  present and falls back to front radar; front-only mode ignores corner points
  for motion and stationary leadOne matching.
- Drag the `CUT-IN path-proximity sensitivity` slider and release it to
  recalculate validation events and lead continuity, then save the value for
  later logs and review runs. The score combines uncertainty-aware body overlap
  with normalized physical clearance: full evidence at the vehicle-width-aware
  ego-path boundary and zero 1.2 m outside it, followed by motion-consistency
  and future-history support. The 0.20 end reacts earlier near the boundary;
  the 0.80 end requires stronger overlap or proximity evidence.
  It does not bypass measured-history, inward-motion, confirmation, range,
  speed, or leadOne-priority checks. Recalculation immediately reuses the
  physical trajectories and leadOne history, reevaluating only temporal
  confirmation and leadTwo selection. Requested and applied values remain
  distinct while dragging. Corner and front values are saved independently.
  Validation and production share sensor defaults of 0.30 and 0.67
  respectively.
- A measured moving point already overlapping the model-path corridor can be
  leadTwo without creating a CUT-IN event or pause. Once selected, physical
  identity and position continuity can retain it if its reported speed later
  approaches zero; a new stationary point is never promoted this way. If
  leadOne disappears, its in-path moving radar point can immediately continue
  as leadTwo; duplicate suppression is applied while leadOne is present.
- `M`: show/hide physical-shadow timeline markers.
- `R`: restart and re-arm already handled physical-predictor CUT-IN pauses.
- `I`: CUT-IN/detect label.
- `C`: CLEAR label.
- `S`: STATIONARY label.
- Escape: close.

Inside a maintained window, a label updates the matching validation case.
Outside every maintained window, it is stored in
`radar_trajectory_labels.json`.

The slider writes both sensor values and the `T` processing mode to the
user-local `carrotpilot/radar_validation.json`. `--prob` provides a one-run
override for the selected sensor without replacing its saved value. Neither
changes production Carrot Radar's fixed 0.30 corner and 0.67 front thresholds,
physical equations, or stored labels.
`--motion-mode normal|front` chooses the
initial mode; `--front-only` is a compatibility alias for front mode. Manual
seek-bar or L1/L2-graph navigation re-arms handled events at and after the new
position so later CUT-IN entries pause again after playback resumes.

Open an arbitrary log outside the maintained case list:

```powershell
.venv\Scripts\python.exe -m openpilot.selfdrive.carrot.radar.tools.radar_lead_simulator `
  "W:\routes\vehicle\segment\rlog.zst" --start 30 --paused
```

The replay finds `qcamera.ts` beside that `rlog.zst` automatically.

## Review discipline

- Keep the original log path, source, window, and physical track identity.
- A reused ID is not the same vehicle unless position and velocity are
  continuous.
- Inspect measured points and `dPath` history before interpreting a probability.
- Compare front and corner independently; never average their histories.
- Record poor results as predictor limitations. Do not silently add a
  case-specific exception.
