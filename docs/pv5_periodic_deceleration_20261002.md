# PV5 repeated deceleration — 2026-10-02 analysis

## Conclusion

The repeated deceleration in `000001ca--5106eff38f--4` is commanded by the
model-based traffic-stop path. Carrot repeatedly interprets the model's lower
future speed and short predicted path as a stop, enters `e2eStop`, and applies
a virtual stopping constraint. Accelerator intervention releases this state
and inhibits another stop entry for approximately ten seconds. Two subsequent
entries coincide with expiration of that inhibition.

The uploaded video shows a dark, divided roadway; sampled frames during the
decelerations show no traffic signal or stop line requiring these stops. This
supports a false traffic-stop interpretation. The exact visual feature causing
the model's lower future-speed predictions is unresolved; this does not establish
a model-runtime defect, a particular model regression, or a PV5 actuator defect.

Analysis only. No production code, parameters, vehicle state, or model changed.

## Evidence and timing

- Vehicle: KIA_PV5, dongle `aa7dd885d7782f0c`, C4/mici.
- Full rlog confirms clean commit `abe1a232d81fd5b0f8db4b7212587952514e0274`.
- Source: `\\DS1821P\openpilot\routes\KIA_PV5 aa7dd885d7782f0c\000001ca--5106eff38f--4`.
- Full cereal schema: 6,000 carState, 6,002 carControl, 1,200 modelV2,
  1,200 radarState and 1,197 longitudinalPlan messages; qcamera has 1,200 frames
  at 20 FPS. Time zero is the first carState, not initData.
- Recorded startup settings include `TrafficLightDetectMode=2`,
  `MyDrivingMode=3`, `ApplyModelSpeed=0`, `CruiseCoastingPercent=0`, and
  `EnableRadarTracks=3`. Runtime stop states establish that traffic-stop control
  was active regardless of startup-setting freshness.

| Segment time | Observation |
| --- | --- |
| 2.448 / 3.346 s | Initial short `e2eStop` entries; measured speed falls from about 66.5 to 63.4 km/h by 5.5 s. |
| 7.211 s | Sustained stop entry. At 7.5–10.8 s speed falls 65.5→54.4 km/h; minimum command −1.98 m/s². |
| 10.135 / 10.156 s | Accelerator pressed / stop state released; ten-second inhibition starts. |
| 20.249 s | Stop entry resumes 10.093 s after release. A brief lead transition at 20.550 s is followed by another stop entry at 20.599 s. |
| 20–24 s | Measured speed reaches 67.5 km/h then falls to 52.3 km/h; minimum command −2.28 m/s², measured acceleration −2.15 m/s². |
| 24.154 / 24.169 / 24.200 s | Stop entry / accelerator pressed / stop released. |
| 34.208 s | Stop entry resumes 10.008 s after that release. |
| 37.106 / 40.857 s | Further stop entries as inferred red/green status changes; speed eventually reaches 37.2 km/h. |

The ten seconds are counted in planner updates, so elapsed wall time is only
approximately ten seconds. The timer starts when accelerator intervention
releases `e2eStop`; it is not a ten-second timer from pedal release.

## Mechanism in the recorded revision

`openpilot/selfdrive/carrot/carrot_functions.py:321` (`check_model_stopping`)
uses the model's predicted trajectory, not a direct observation of red-light
color. Below 82 km/h, stop classification can occur when:

1. Predicted endpoint is closer than the lead (or the 1,000 m no-lead fallback),
   and below a roughly 120–150 m speed-dependent threshold.
2. Smoothed terminal model speed is below 3 m/s **or below 70% of its initial
   predicted speed**.
3. Predicted lateral endpoint magnitude is below 5 m.

`stopSignCount * DT_MDL > 0.0` permits a red classification after one qualifying
update. There is a ten-sample terminal-speed moving average, but no additional
multi-frame stop-candidate confirmation at that gate. Actual red-light or
stop-line corroboration is not required by this condition.

For example, around the 20.249 s entry, the latest published model predicts
99.49 m at the endpoint, initial speed 15.69 m/s and terminal speed 7.36 m/s,
with only 0.21 m lateral endpoint displacement. At 34.208 s these are 104.06 m,
15.65 m/s, 6.95 m/s and 0.21 m. These raw endpoint samples illustrate the
condition; the code evaluates a filtered terminal speed. A prediction of
slower travel can therefore be promoted into an actual stopping constraint
even though the model endpoint speed is well above zero.

At lines 530–535 accelerator intervention during `e2eStop` selects `e2eCruise`
and sets `traffic_starting_count = 10.0 / DT_MDL`. At lines 566–573, entry is
permitted again when that counter reaches zero. This explains the conspicuous
repetition after the driver's interventions.

The stop path also lowers its internal cruise speed according to stopping
distance (lines 632–636). The MPC uses both that cruise constraint and a
traffic-stop obstacle (`longitudinal_mpc_lib/long_mpc.py:437` onward).
Consequently a recorded `longitudinalPlanSource=cruise` does not exclude
traffic-stop braking. There are 134 `e2e` and 1,063 `cruise` source frames;
none are lead0/lead1. The published `cruiseTarget` remains 70/72 km/h because
it does not expose this internal reduced speed.

## Alternatives checked and limits

- Set speed stays 70 km/h. Navigation `xSpdType=-1` throughout; navigation
  desired speed remains above set speed. No enforcement-camera or navigation
  speed reduction explains these events.
- Main stop-entry instants have no selected lead; lead0/lead1 never owns the
  published MPC source. A distant/SCC object appears elsewhere, but its presence
  does not explain the repeated stop entries.
- No brake-pedal input; carState CAN validity stays true throughout.
- `forceDecel` is true only around 23.825–23.975 s, with a `posenetInvalid`
  soft-disable alert beginning at 23.822 s. This is a separate transient after
  the second major deceleration has already happened, and cannot explain the
  repeated pattern. Its own underlying cause is outside this investigation.
- `experimentalMode=false` does not disable Carrot's separate traffic-stop
  path. Likewise `ApplyModelSpeed=0` does not prevent this stop interpretation.
- Low-resolution video sampling cannot establish all road semantics or which
  image features caused the model prediction. No modified-controller replay or
  physical validation was performed.

Review should focus on the qualification and re-entry policy of the model-based
traffic-stop detector, including the use of a relative future-speed drop as
stop evidence. Changing radar selection or merely shortening recovery would
not address the demonstrated trigger. Any fix must retain legitimate stopping
behavior and be assessed separately from this recorded-input diagnosis.

Private reproduction scripts, derived timeline, and video contact sheet are
archived in `.analysis/archive/2026-10-02/pv5-periodic/`. Original uploads remain
on NAS; no private captures or settings should be committed.
