# Moving-vision / stationary-reflection review, 2026-09-05

The Carnival segment `00000178--52f7f496c5--53` reproduces a stationary
front-radar false L1 despite corresponding vision and physical radar for the
real moving vehicle. The recorded commit is `f8ce05da2e`, after the earlier
no-vision stationary-acquisition fix, with `EnableRadarTracks=3` and
`EnableCornerRadar=2`.

## Evidence and cause

The full OpenPilot cereal schema was used to decode the original rlog.
Relevant message counts are 6,004 carState, 6,019 carControl, 1,200 modelV2,
1,200 radarState, 1,199 liveTracks, and 1,193 longitudinalPlan messages.
Times use the replay frame clock rather than the initData boot timestamp.
Original qcamera frames were inspected before, during, and after the event.

At 13.806 s, the original and replayed L1 is front track 32:

| Observation | Range (m) | Absolute speed (m/s) |
|---|---:|---:|
| Vision, probability 0.990 | 77.26 | 17.07 |
| Front 35, actual moving lead | 86.64 | 15.73 |
| Corner 4983, actual moving lead | 82.58 | 15.91 |
| Front 32, stationary reflection selected as L1 | 71.05 | -0.27 |

Front 32 crosses the visual range while ego approaches the green-light
intersection. The video is consistent with a reflection from an overhead
signal or its supporting structure; these radar measurements do not identify
the exact reflecting surface. The real lead vehicle continues moving.

The position-lock exception permits up to 18 m/s of model/radar speed
disagreement after 0.50 seconds of radar observation. That is radar age,
not continuous evidence that vision and radar track the same object.
Track 32 accumulates four briefly matching visual positions beginning at
13.058 s. At 13.357 s it completes confirmation even though its current
range is already outside the 5 m position-lock gate: a pending support-gap
hold preserves the previous evidence. The closer high-quality stationary
retention rule then keeps it after vision moves tens of metres away.

The simultaneous moving front/corner measurements agree with visual speed.
They contradict the assumption that only the model's velocity estimate is
wrong. The earlier validation covered unsupported no-vision acquisition but
did not cover a stationary reflection borrowing another vehicle's vision.

Throughout 13.3–16.4 s, longitudinalPlan reports `trafficState=2` (green) and
`xState=0` (lead), with the braking plan using `lead0` and fast lead ID 32.
This is a radar lead-selection error, not entry into the red-light stop state.
Before the first recorded brake-pedal press at 15.192 s, commanded acceleration
reaches -4.00 m/s² and measured acceleration reaches -3.32 m/s². Later pedal
input must be considered separately when interpreting measured deceleration.

## Correction

A measured moving target can disprove stationary vision association when it
agrees with current vision in range, raw lateral position, and speed, remains
central, and has sufficient observation history. An incompatible stationary
front return without its own matching corner pair is removed from all matcher
admission paths and from pending or confirmed retention. The position-lock
exception remains available for the maintained real stopped-vehicle case
whose model velocity is late and has no such contradictory moving evidence.

Corner confirmation is attached to the selected physical object. A different
stationary corner near the visual range cannot grant confirmation to a front
return. A closer front-only hold also needs the normal speed agreement when
its range has separated from confident vision by more than 15 m.

## Validation and limits

- Modes 2 and 3 each remove all **60 false L1 frames**, from 13.357 through
  16.300 s. The front-only mode 3 replay also passes the incident case.
- At 13.806 s, mode 2 uses the central moving vision lead. Mode 3 uses its
  configured SCC fallback (track 0, about 15.76 m/s); front 35 is over 8 m
  farther than vision at that frame and remains subject to the existing gate.
- Later in this same video, the actual lead 35 slows and stops at the next
  signal. All **349 frames** between 27.5 and 45.0 s retain that lead in both
  modes. The previous Carnival false positives and real stopped-vehicle
  regression cases remain covered.
- **487 focused radar, replay, CUT-IN, and longitudinal tests pass**, with
  additional cases for competing front/corner observations, acquisition,
  release of a previously seeded identity, unrelated corner evidence, and
  preservation of an independently paired stopped object. Ruff passes.
- The complete comparison uses **84 logs and 466 maintained/manual labels**
  in both modes 2 and 3. Failed expectations fall from **39 to 38** in each
  mode, with no new failures, no missing
  logs, and no pre-deceleration failures. The 38 pre-existing failures remain
  visible: 37 manual detection labels and `gv80-218-2-adjacent-right-14`.
  The full strict validator therefore still exits with status 1.

The [machine-readable comparison](carrot_radar_moving_vision_stationary_validation.json)
records the failure IDs and incident-frame checks. Lead replay does not
recalculate a counterfactual MPC trajectory or establish an on-road result;
the displayed original acceleration traces remain recorded measurements.

```powershell
.\.venv\Scripts\python.exe openpilot/selfdrive/carrot/radar/tools/validate_radar_lead_model.py --case carnival-178 --shadow-only --strict-shadow --strict-predecel
.\.venv\Scripts\python.exe openpilot/selfdrive/carrot/radar/tools/validate_radar_lead_model.py --case carnival-178 --front-only --enable-radar-tracks 3 --shadow-only --strict-shadow --strict-predecel
.\.venv\Scripts\python.exe openpilot/selfdrive/carrot/radar/tools/validate_radar_lead_model.py --shadow-only --strict-shadow --strict-predecel --report replay-report.json
```

No route ID, vehicle identity, or traffic-signal classification is used by the
production correction.
