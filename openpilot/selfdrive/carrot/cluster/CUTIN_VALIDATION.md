# Radar cut-in validation

The maintained validation sources are:

- `cutin_validation_cases.json`: control-facing CUT-IN, CLEAR, and STATIONARY windows.
- `radar_trajectory_labels.json`: point-level CUT-IN and CLEAR review labels.

Both files are validation-only. Do not use their values to fit a model, tune the
physical equations, select thresholds, or add scene-specific exceptions.

## Current architecture

`RadarLeadModelMode` and the learned radar-lead/path-occupancy runtimes have
been removed. `RadarMotionMode=0` runs only
`openpilot/selfdrive/controls/radard.py` and preserves its existing lead
selection. `RadarMotionMode=1` does not start or import that implementation. It
runs only `openpilot/selfdrive/carrot/radar/radard_dpath.py`, first calculates
front/SCC vision-matched `leadOne`, then supplies `leadTwo` only from a
physically confirmed dPath CUT-IN.
It rejects the primary object itself, anything at or beyond the primary while
that primary is valid, anything beyond 80 m, and anything beyond the
ego-speed-based two-second control range. Front, SCC, and corner inputs retain
their production source identity.

PC visual replay runs only the new `DPathRadarController` and
`RadarMotionPredictor`. Its lead roles are recalculated from logged model and
radar inputs; it does not import or display recorded conventional-radard lead
roles or CUT-IN events. The headless validator may compute existing-radard
metrics separately, but those values are never input to the physical predictor.

The predictor:

1. accepts only `measured=true` points;
2. limits motion prediction to `-5 <= dRel <= 100 m`;
3. projects replay points to the model-path timestamp with measured relative
   velocity, then projects them onto that same-time model-path polyline; `S`
   is centerline arc distance and `dPath` is signed centerline-normal distance;
4. limits history and shadow candidates to the ego lane plus the immediate
   left/right lanes with the fixed model-path-relative
   `|dPath| <= 5.4 m` range;
5. keeps points inside 5 m and the nearest point at or beyond 5 m on each
   adjacent side, excluding farther vehicles occluded on that same side from
   detection while retaining their measured in-scope history for later
   physical continuity;
6. uses corner motion only when the log has measured corner data, otherwise
   `frontRadar` raw-track motion; SCC stays available to existing radard but is
   not predictor input, and the source choice never switches per frame;
7. treats `|vLead| < 3 km/h` as position-only and never builds or extrapolates
   motion history for those points;
8. does not switch to lane center and does not apply yaw correction again
   after the point and path share a timestamp and ego frame;
9. checks track-ID reuse and short gaps with physical continuity;
10. keeps front and corner histories and parameters independent;
11. builds a 2-D path-relative history from projected centerline progress `S`,
   integrated ego travel, and signed-normal `dPath`, without treating raw
   `dRel` as centerline distance;
12. fits `dPath` against target progress in `S`, uses the long-window vector
   and model-path tangent angle error as the prediction mean, reduces
   confidence beyond the observed spatial baseline, and sends short-window
   disagreement to curvature and uncertainty;
13. for corner radar, checks position-derived normal motion against reported
   lateral velocity and lowers motion confidence when the two are physically
   inconsistent;
14. predicts synchronized future `dRel` and `dPath`;
15. confirms a threshold crossing for 0.25 seconds before producing a CUT-IN
   event; and
16. reports CUT-IN and CUT-OUT probabilities independently.

The current IN state includes ego and target vehicle half-widths. A tracked
OUT-to-IN crossing keeps its pending entry evidence after overlap begins so
the confirmation interval can complete. A point first observed inside has no
such entry evidence. Only small path-state and confirmation hysteresis are allowed. Do not add per-route,
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

# Treat existing-radard expectation failures as a nonzero result
python openpilot/selfdrive/carrot/validate_radar_lead_model.py --strict-radard

# Remove corner inputs
python openpilot/selfdrive/carrot/validate_radar_lead_model.py --front-only
```

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
- a -10 through 120 m distance view with ego as a white point, recalculated
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
- a full-log continuity graph of recalculated leadOne distance in orange and
  leadTwo distance in yellow. Missing leads and track-ID changes break the line;
- a clickable seek bar with physical-predictor CUT-IN entry markers in orange
  and validation windows above it. Existing-radard markers are absent.

The UI uses a Korean-capable font and Korean operator labels. Gray or green
future rings are unconfirmed/current-IN motion; orange points, rings, and
timeline markers mean confirmed physical-predictor CUT-IN only.

With no filters, the maintained cases cover 40 unique logs. They open in
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
- Drag the `CUT-IN sensitivity prob` slider and release it to recalculate
  validation events and lead continuity, then save the value for later logs
  and review runs. The 0.20 end is more sensitive; the 0.80 end is more
  conservative.
- `M`: show/hide physical-shadow timeline markers.
- `R`: restart and re-arm already handled physical-predictor CUT-IN pauses.
- `I`: CUT-IN/detect label.
- `C`: CLEAR label.
- `S`: STATIONARY label.
- Escape: close.

Inside a maintained window, a label updates the matching validation case.
Outside every maintained window, it is stored in
`radar_trajectory_labels.json`.

The slider writes its value to the user-local
`carrotpilot/radar_validation.json`. `--prob` provides a one-run override
without replacing that saved value. Neither changes conventional radard,
production Radar Motion's 0.50 threshold, physical equations, or stored labels.
`--front-only` removes corner points before both replay and shadow prediction.

## Review discipline

- Keep the original log path, source, window, and physical track identity.
- A reused ID is not the same vehicle unless position and velocity are
  continuous.
- Inspect measured points and `dPath` history before interpreting a probability.
- Compare front and corner independently; never average their histories.
- Record poor results as predictor limitations. Do not silently add a
  case-specific exception.
