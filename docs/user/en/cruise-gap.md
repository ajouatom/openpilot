# Cruise and Following-Gap Settings

[한국어](../ko/cruise-gap.md)

> [!NOTE]
> This is the canonical English user guide maintained with the `carrot-wip` code. When user-visible behavior changes, update this document together with the related code and tests.

This page explains all **31 cruise and following-gap settings** from the current implementation, including where each value enters the calculation and the direction of adjustment.

Change them in **Carrot Web → Settings → Driving control → Cruise and following gap**.

> [!CAUTION]
> These are not recommended values for a particular car. Some settings do not apply when openpilot is not controlling acceleration and braking, and vehicle controllers or safety limits can restrict their effect. Back up the current profile and settings file, then change one item at a time.

## Seven sections

1. [Driving mode](#driving-mode)
2. [Speed-based acceleration table](#acceleration-table)
3. [Stopping and restarting](#stop-resume)
4. [Longitudinal tuning](#longitudinal-tuning)
5. [Following gap](#following-gap)
6. [Lead-vehicle response](#lead-response)
7. [Carrot cruise](#carrot-cruise)

## Order in which settings act

| Stage | Settings | Role |
|---:|---|---|
| 1 | Driving mode, speed acceleration table | Maximum acceleration allowed at the current speed |
| 2 | Following gap, lead response | Predicted lead motion and target distance |
| 3 | Stop/restart | Stop target and stopping/starting state transitions |
| 4 | Longitudinal tuning | Make the vehicle follow planned speed and acceleration |
| 5 | Carrot cruise | Separately adjust final deceleration on supported Hyundai/Kia controllers |

The same symptom can therefore have different causes. A slow launch might come from the low-speed acceleration table, start acceleration-change cost, PID gains, or a vehicle limit.

Catalog defaults and initial Params values currently differ for `CruiseMaxVals1` through `6` and `StopDistanceCarrot`. Use the value shown on your device as the baseline.

<a id="driving-mode"></a>
## 1. Driving mode

### `MyDrivingMode`

| Value | Mode | Max acceleration | `comfort_brake` | Time-gap term | Additional behavior |
|---:|---|---:|---:|---:|---|
| `1` | Eco | ×0.9 | ×1.0 | ×1.1, then clamped | Traffic-light detection retained |
| `2` | Safe | ×0.8 | ×0.9 | ×1.2, then clamped | Congestion state used by auto mode |
| `3` | Normal | ×1.0 | ×1.0 | ×1.0 | Baseline |
| `4` | High speed | ×1.2 | ×1.0 | ×1.0 | Traffic stop/go detection forced off |

A smaller `comfort_brake` increases the stopping-distance term. Baseline time-gap factors are 1.1 in Eco and 1.2 in Safe; speed scaling, clamps, deceleration allowance and selected-TF priority at levels 4–5 determine the final gap.

> [!WARNING]
> High-speed mode raises the acceleration ceiling by 20% and ignores traffic-light control.

In Safe mode, levels 4–5 retain existing launch response and boost entry. Only when ego out-accelerates the lead while catching the target gap does the future positive-acceleration ceiling taper. Renewed lead acceleration or sufficient opening gap removes the extra restriction. This Safe acceleration limiter itself adds no gap allowance; existing Safe acceleration limits, TF processing and braking limits remain active.

### `MyDrivingModeAuto`

`0` uses the stored mode. `1` switches only between Safe and Normal according to traffic conditions; `2` switches between Safe and Eco. High-speed mode is never selected automatically.

The current code enters congestion after repeated observations of either:

- Lead distance at most 12 m and lead speed at most 2 km/h; or
- Lead speed below 5 km/h, lead acceleration below 0.2 m/s², ego speed above 1 km/h, and lead distance below 200 m.

It exits when lead acceleration exceeds 1.5 m/s², ego speed exceeds 35 km/h, or no lead is present within 200 m. The speed-based congestion exit threshold is **35 km/h**.

Changing the stored `MyDrivingMode` during a drive can suspend automatic switching until the planner process restarts. For a stable comparison, use `MyDrivingMode=3` and `MyDrivingModeAuto=0`.

<a id="acceleration-table"></a>
## 2. Speed-based acceleration table

`CruiseMaxVals0` through `6` are maximum planner acceleration values, not accelerator-pedal percentages. Multiply the stored value by `0.01 m/s²`; intermediate speeds are linearly interpolated.

| Setting | Reference speed | Catalog default | Effective acceleration |
|---|---:|---:|---:|
| `CruiseMaxVals0` | 0 km/h | 160 | 1.60 m/s² |
| `CruiseMaxVals1` | 10 km/h | 160 | 1.60 m/s² |
| `CruiseMaxVals2` | 40 km/h | 120 | 1.20 m/s² |
| `CruiseMaxVals3` | 60 km/h | 100 | 1.00 m/s² |
| `CruiseMaxVals4` | 80 km/h | 80 | 0.80 m/s² |
| `CruiseMaxVals5` | 110 km/h | 70 | 0.70 m/s² |
| `CruiseMaxVals6` | 140 km/h | 60 | 0.60 m/s² |

All range from 1 to 250 in steps of 5, so one step is 0.05 m/s². With the table above, 50 km/h interpolates to about 1.10 m/s². Driving-mode multipliers make that about 0.99 Eco, 0.88 Safe, 1.10 Normal, or 1.32 m/s² High.

This is a ceiling, not the final command. Curve limits, the vehicle interface, safety limits, or a lower desired speed can take priority. The experimental blended planner uses a separate acceleration range; stock-SCC vehicles may see little or no direct effect.

Tune the speed band containing the symptom instead of changing the whole table. Large jumps between adjacent points can produce an abrupt change in acceleration feel while crossing a speed band.

<a id="stop-resume"></a>
## 3. Stopping and restarting

| Setting | Stored-value interpretation | Direction when increased or moved toward zero |
|---|---|---|
| `StopDistanceCarrot` | `600` → 6.00 m | Increases fixed clearance to a stopped lead |
| `StoppingAccel` | `-50` → -0.50 m/s² | Moving toward zero weakens the stopped-state brake target |
| `VEgoStopping` | `50` → 0.50 m/s | Higher values enter stopping state at a higher planned speed |
| `AChangeCostStarting` | MPC acceleration-change cost | Higher values smooth initial acceleration changes |

### `StopDistanceCarrot`

Range 400–1000 cm, step 10 cm. The code divides by 100 and uses it as the fixed-distance term:

    ego braking distance + time gap × ego speed + StopDistance - lead braking distance

It is therefore not the actual moving following distance. Its direct effect is clearest near zero speed behind a stopped lead. When there is no active `leadOne` but the camera model consistently associates a stationary vehicle with the E2E stop endpoint, the planner first corrects that endpoint toward the inferred vehicle position and then applies this fixed clearance. No SCC/radar object is created. Although the catalog description says “stop position ×0.8,” the running code does not apply 0.8.

### `StoppingAccel`

Range -100 to 0 in steps of 10, scaled by `0.01 m/s²`.

- More negative: allows earlier stop-state entry and a stronger stopped brake target.
- Closer to zero: weaker target.
- Exactly `0`: Hyundai, Kia, and Genesis automatically save `-50` when vehicle control initializes after boot and use `-0.50 m/s²` from the first control update. Other brands use the vehicle's `CP.stopAccel`.

Existing negative values are preserved for Hyundai, Kia, and Genesis. If `0` is saved again later, it is restored to `-50` at the next vehicle-control initialization.

An excessively negative value can make final braking harsh.

### `VEgoStopping`

Range 1–100, step 5. A value of 50 is 0.50 m/s (about 1.8 km/h). `shouldStop` becomes true when both the planner's current and one-second-ahead target speeds are below this threshold.

Lowering it delays stop recognition and may release stop state sooner on departure. Raising it enters stop state earlier but can make departure feel sluggish.

### `AChangeCostStarting`

Range 0–200, step 10, catalog default 10. Zero permits the quickest acceleration change; a larger value produces a smoother but potentially slower launch. It mainly matters while stopped or immediately after planner reset and does not behave identically in blended mode.

<a id="longitudinal-tuning"></a>
## 4. Longitudinal tuning

> [!IMPORTANT]
> Hyundai, Kia, and Genesis vehicles are fixed at `Kp=1.0`, `Ki=0`, and `Kf=1.0` to preserve safe acceleration and braking tracking. These three gain settings are hidden on those vehicles, and previously stored values are ignored by control. `LongActuatorDelay` remains visible and effective.

| Setting | Default | Stored range (step) | Actual scale | Role |
|---|---:|---:|---:|---|
| `LongTuningKpV` | 100 | 0–200 (5) | ×0.01 | Immediate proportional response |
| `LongTuningKiV` | 0 | 0–2000 (1) | ×0.001 | Accumulated correction for persistent error |
| `LongTuningKf` | 100 | 0–200 (5) | ×0.01 | Feedforward from target acceleration |
| `LongActuatorDelay` | 20 | 0–200 (5) | ×0.01 s | How far ahead in the plan to compensate for response delay |

> [!IMPORTANT]
> The displayed `LongTuningKiV` title says `×0.01`, but `longcontrol.py` currently applies **×0.001**. Stored `100` is Ki `0.100`, not `1.00`.

Hyundai, Kia, and Genesis do not read the stored `LongTuningKpV`, `LongTuningKiV`, or `LongTuningKf` values. On other brands, the overrides apply only when the vehicle's base longitudinal tune has a single Kp point and a single Ki point. Multi-point vehicle tunes retain their defaults. These gains are also not the primary controller when stock SCC controls acceleration and braking.

- Raising Kp corrects present speed error more strongly; too much can oscillate.
- Raising Ki removes persistent error faster; too much can accumulate into overshoot.
- Raising Kf commands more for the same target acceleration in both acceleration and braking directions.
- Raising delay uses a more future plan point and acts earlier; too much can lead the real car and surge.

On other brands, tune delay first only if acceleration and braking are both consistently late, in 0.05-second steps. Then consider Kf, Kp, and finally Ki. Restore the saved profile immediately if oscillation appears.

<a id="following-gap"></a>
## 5. Following gap

### Four base time gaps

Multiply stored values by 0.01 seconds. All four range from 40 to 300 in steps of 5.

| Longitudinal personality | Setting | Catalog default | Time |
|---|---|---:|---:|
| aggressive | `TFollowGap1` | 110 | 1.10 s |
| standard | `TFollowGap2` | 120 | 1.20 s |
| relaxed | `TFollowGap3` | 140 | 1.40 s |
| moreRelaxed | `TFollowGap4` | 160 | 1.60 s |

Hyundai/Kia configurations can expose all four personalities. Other vehicles can be limited to three. A time gap is not a fixed distance: at 100 km/h, 1.20 seconds is about 33 m before braking-distance and fixed-distance terms are included.

### Actual application order

1. Select baseline TF from personality or the speed table.
2. Apply speed reduction and driving-mode factors, subject to the level 4–5 accelerating-lead exception below.
3. Hold baseline TF against reduction during braking, then add `TFollowDecelBoost` once.
4. Apply configured/global bounds and rate-limit TF increases.
5. Release extra deceleration margin at 0.10 seconds per second as braking eases.

### `EnableSpeedTF`

| Value | Behavior |
|---:|---|
| `0` | Use the selected Gap1–4 without a speed adjustment |
| `1–50` | Below 100 km/h, reduce the time gap increasingly at lower speed by this percentage |
| `-1` | Treat Gap1–4 as a table at 0/30/60/90 km/h |
| `-2` | Table at 0/40/80/120 km/h |
| `-3` | Table at 0/50/100/150 km/h |

For a positive value of 20, the time gap is 80% of base at 0 km/h, 90% at 50 km/h, and 100% at 100 km/h or above.

Negative modes build a speed table and then apply personality multipliers of ×1.0, ×1.3, ×1.6, and ×2.0. The result is clamped back to the four values' minimum/maximum, so large multipliers may stop near `TFollowGap4`.

Tracking a lead with `LeadAccelResponse=4` or `5` is an exception at every following-distance level. The selected gap’s `TFollowGap1`–`TFollowGap4` setting takes priority over positive or negative `EnableSpeedTF` adjustments and Eco/Safe gap factors only while a stable radar lead is accelerating positively and the gap is opening. When lead acceleration falls to `0.1 m/s²` or below, the exception is removed immediately and normal gap control—including the existing TF increase ramp—and braking behavior resume. It does not change the no-lead cruise target. During lane-change starting and finishing, this exception and stronger acceleration response are disabled, retaining normal base TF.

### `DynamicTFollowLC`

This setting permits bounded relief of the old lead's following-distance requirement after predicted body clearance. It does not reduce the common TF or acceleration-change costs during a lane change. Range 20–100, step 5.

- `100`: no additional relief for the old lead.
- Lower values: more relief after confirmed departure.
- `80` and below: the same cap. Legacy values such as `50` or `20` no longer substantially shorten TF.

The catalog and initial Params default are both `100`. Invalid values, including `0`, disable additional relief. Establish baseline behavior at `100` and change one step at a time. Lower values do not mean faster acceleration in every lane change.

Relief requires measured lateral movement, continuous observations of the same lead, predicted body clearance, and sufficient selected leadTwo spacing. It starts only after predicted clearance plus 0.35 seconds. The cap is the smallest of 20% of base TF, 0.25 seconds, and 4 metres. Current leadOne/leadTwo retain normal TF.

A blind-spot warning, cancellation, changed or missing tracks, missing pose data, or a missing normally selected second lead blocks additional relief. A lane that appears empty does not authorize earlier acceleration without sufficient observations. This setting does not replace lane-change permission checks or checking rear traffic.


The normally selected leadOne/leadTwo at lane-change entry form the reference pair. If either selected lead changes or disappears, additional acceleration relief stops for the rest of that maneuver. Braking uses only the currently selected leads; side radar candidates and stored entry vehicles never become additional braking obstacles.

### `TFollowDecelBoost`

The margin is added once to baseline TF and does not accumulate during sustained deceleration. As braking eases, or the setting is changed to zero, applied extra margin releases at 0.10 seconds per second. Increasing braking margin is not delayed by this release rate.

The default is `0%`, which adds no extra time gap based on deceleration strength. Existing saved vehicle settings are preserved after an update.

At ego acceleration around -0.2 m/s² or below, the code first prevents speed adjustment from reducing the target gap. This prevention works even when the setting is zero. The setting then adds gap based on deceleration strength.

At `TFollowDecelBoost=50`, the addition is approximately 0.03 s at -0.3 m/s², 0.125 s at -1.0 m/s², and a maximum around 0.25 s at -2.5 m/s². Range is 0–100 in steps of 10.

For a clean baseline, use `EnableSpeedTF=0`, `DynamicTFollowLC=100`, `MyDrivingMode=3`, and `MyDrivingModeAuto=0`. If the result is still wrong, check the base gaps, stop distance, selected personality, and radar lead before adding dynamic features.

<a id="lead-response"></a>
## 6. Lead-vehicle response

Use `LeadAccelResponse` to adjust response to a lead starting, accelerating or being approached. Its range is 0–5; the default 0 disables acceleration boost and recovers extra TF most slowly.

### `LeadAccelResponse`

Sets lead-start and acceleration response at every following-distance level. Levels 1–3 soften small changes and response near the target gap; level 4 is quick and level 5 retains the immediate maximum response. The selected TF remains the reference; a separate lead-jerk adjustment no longer expands or shrinks TF.

| Level | `aChangeCost` at full boost | Multiplier on existing jerk cost |
|---|---:|---:|
| 0 Relaxed recovery | 200 | 100% |
| 1 Most gradual | 190 | 95% |
| 2 Gentle follow | 170 | 85% |
| 3 Balanced follow | 130 | 70% |
| 4 Urgent follow | 36 | 35% |
| 5 Maximum follow (test) | 10 | 15% |

With sufficient input, levels 1–4 ramp boost entry over 0.80/0.60/0.40/0.15 seconds. Boost scales down when distance margin is below 2.0/1.5/1.0/0.5 metres respectively, or the acceleration signal is small. The table gives full-boost costs; small changes stay closer to baseline costs. Level 5 has neither fade nor entry delay. Vehicles previously using DynamicTFollow may feel different because its additional TF reduction and jerk boost are removed.

Acceleration boost at every level requires normal ACC, no accelerator override or stop request, and a stable radar lead. Levels 1–2 use lead0/lead1 sources; levels 3–5 also support cruise. Cruise requires more than 1 km/h of set-speed headroom. Lead acceleration must exceed 0.1 m/s²; levels 1–4 with a lead source also require relative acceleration above the 0.1 m/s² deadband. Existing relative-speed and level-specific prediction gates remain active.

Boost ends immediately at the TF target distance, when lead acceleration ends, or when closing-speed conditions fail. A changed lead restarts gradual entry at levels 1–4. Level 5 retains the −0.2 m/s relative-speed floor and 0.5-second prediction condition. All levels disable boost during lane-change starting/finishing, blended mode, and vision-only lead tracking.

Levels 4–5 prioritize the selected `TFollowGap1`–`TFollowGap4` while a stable lead accelerates and the gap opens. Levels 1–3 retain normal speed/mode TF processing. `CruiseMaxVals`, curve, cut-in, lead-distance and danger-distance limits, and deceleration preview remain active. No acceleration is added after MPC. This setting does not change `AChangeCostStarting` or PID gains. Lower levels do not delay braking required by an urgent approach.

Levels 0–4 capture half of the excess over the base following distance when acquiring a radar lead or while the measured following gap opens. Base TF plus extra TF is capped at 2.5 seconds without reducing a larger base TF. A first-order filter recovers the extra TF even while the gap opens, most slowly at level 0. A large gap alone does not repeatedly refill it. A stopped lead retains it; a slow lead recovers it more slowly. Level 5 adds no extra TF. This replaces the previous relative-closing-speed distance allowance rather than stacking with it.

Headroom applies to a stable radar lead in normal ACC. Accelerator override, disengagement, forced deceleration and lane changes disable it. Unlike acceleration boost, it also applies at level 0 and during stopping. Target loss/replacement or a level change does not transfer the old allowance.

Acquisition allowance ramps in over 0.8 seconds. Afterward, a larger candidate is accepted when measured relative speed, filtered with a 0.3-second time constant, exceeds 0.2m/s; further increases are limited to 0.5 TF seconds per second. A brief lead-acceleration lull does not end capture while the gap keeps opening, and renewed opening can capture more headroom.

Stored extra TF continues recovering during capture. A constant or closing gap does not refill it. A newly acquired slower or stopped lead can receive initial headroom without an opening gap. The stopped-lead hold rule is described below.

| Level | Base recovery time constant | Extra TF |
|---|---:|---|
| 0 | 5 seconds | Active |
| 1 | 4 seconds | Active |
| 2 | 3 seconds | Active |
| 3 | 2 seconds | Active |
| 4 | 1 second | Active |
| 5 | — | None |

Lead speed at or below 0.3m/s holds extra TF. From 0.3 to 5m/s, recovery strength increases linearly with lead speed; at 5m/s and above the table applies. A five-second time constant leaves roughly 37% after five seconds rather than completing recovery. For a stopped lead, the extra-TF distance term shrinks with ego speed toward normal stopping clearance.

Capture uses actual distance minus the base target distance, with a 1m/s minimum divisor at low ego speed. Without excess over the base target including braking-distance terms, there is no new extra TF. Only the MPC comfort reference changes; physical lead positions, base TF, danger constraints and braking limits stay unchanged. Temporary TF does not guarantee a particular braking onset or ride quality. Ego-deceleration `TFollowDecelBoost` remains separate existing TF processing.

In Safe mode, levels 4–5 retain existing launch response and boost entry. Only when ego out-accelerates the lead while catching the target gap does the future positive-acceleration ceiling taper. Renewed lead acceleration or sufficient opening gap removes the extra restriction. This Safe acceleration limiter itself adds no gap allowance; existing Safe acceleration limits, TF processing and braking limits remain active.

Current target-distance headroom, relative speed and lead acceleration estimate the approach over about two seconds. Settling is considered only when ego acceleration exceeds the lead’s positive acceleration by more than 0.1 m/s². The future ceiling descends from current acceleration at 0.8 m/s² per second of prediction time; this is not a fixed vehicle jerk limit and never blocks negative acceleration.

Safe entry and exit blend the correction over 0.8 seconds. Target change/loss and existing boost inhibits such as accelerator override or lane change clear the state. Steady operation in Normal and levels 0–3 receive no settling correction. Configured TF is not increased, and existing selected-TF priority conditions for levels 4–5 during lead acceleration remain. Prompt launches still respect the existing Safe acceleration ceiling and do not guarantee prevention of cut-ins.

### Adjustment sequence

1. Keep driving mode and time gap fixed, and use `LeadAccelResponse=0` to check gradual gap recovery without acceleration boost.
2. Adjust `LeadAccelResponse` one level at a time to change response to a lead starting or accelerating at the selected gap.
3. Compare launch response, acceleration settling during approach and deceleration in the same driving mode at similar speeds and lead conditions.
4. Restore the previous value if surging or unintended acceleration appears.

<a id="carrot-cruise"></a>
## 7. Carrot cruise

### `CruiseEcoControl`

Designed to encourage HEV EV-mode behavior, but the code has no vehicle-type restriction. When ego speed is more than 3 km/h below a set speed above 20 km/h, this value is temporarily added to the planner target. The correction ends after ego speed exceeds the original set speed.

For set speed 100, ego 96, and a value of 2, the temporary target is 102 km/h. Range is 0–10 km/h; zero disables it. This changes the target, not the maximum acceleration, so driving mode and the acceleration table still matter.

### Conditions for `CarrotCruiseDecel` and `CarrotCruiseAtcDecel`

These are currently implemented only in the **Hyundai/Kia controller** and require all of the following:

- Carrot cruise state active
- No driver accelerator override
- Not in soft-hold or stopping state
- Speed above 10 km/h

Button, LFA, or paddle settings can activate the state.

### `CarrotCruiseDecel`

Range -1–200, step 10. Non-negative values are scaled by `0.01 m/s²`.

| Value | Behavior |
|---:|---|
| `-1` | If either planned or current requested acceleration is not decelerating, temporarily release longitudinal command for coasting |
| `0` | Use zero as the ceiling and gradually remove positive acceleration |
| `50` | Require at least about -0.50 m/s² |
| `100` | Require at least about -1.00 m/s² |
| `200` | Require at least about -2.00 m/s² |

For a positive value, the stronger deceleration of the planner command and override is selected. Command change is rate-limited to about 1.0 m/s² per second. The `-1` description can look like a global cruise-off option, but it is specifically a coasting path inside Carrot cruise conditions. If both planned and requested values are already below about -0.1 m/s², normal planning remains active.

### `CarrotCruiseAtcDecel`

This override applies only when an ATC turn point is 0–500 m ahead. Range -1–200, step 10:

- `-1`: leave `CarrotCruiseDecel` unchanged.
- `0–200`: use the numerically larger deceleration magnitude of the base and ATC values.

For base 50 and ATC 100, the turn section uses about -1.00 m/s². For base 100 and ATC 50, it retains the stronger base value.

> [!WARNING]
> A large positive setting can force stronger deceleration than the planner originally requested. Test only in an approved environment and in small steps.

ATC distance is carried by the internal `carrotMan` service message. That is an internal service name and does not indicate support for the former CarrotMan app or CarrotLink; neither is currently supported.

## Quick diagnostic order

1. Confirm that openpilot actually controls acceleration and braking on the vehicle.
2. Disable automatic driving-mode switching and dynamic gap to establish a baseline.
3. Verify radar lead distance, speed, and acceleration.
4. Change only the one setting directly associated with the symptom.
5. Repeat under similar speed, gap personality, and lead conditions.
6. Restore the saved profile when the result is worse or unclear.

Related: [Understanding Settings](settings.md) · [Tuning introduction](https://github.com/ajouatom/openpilot/wiki/Guide-Tuning) · [Carrot Web](https://github.com/ajouatom/openpilot/wiki/Guide-Carrot-Web)

## Code references

- `openpilot/selfdrive/carrot_settings.json`
- `openpilot/selfdrive/carrot/carrot_functions.py`
- `openpilot/selfdrive/controls/lib/longitudinal_planner.py`
- `openpilot/selfdrive/controls/lib/longitudinal_mpc_lib/long_mpc.py`
- `openpilot/selfdrive/controls/lib/longcontrol.py`
- `openpilot/selfdrive/controls/radard.py`
- `opendbc_repo/opendbc/car/hyundai/carcontroller.py`

### Predicted departure of the lead vehicle

When a previously confirmed moving front vehicle is leaving your path and vision has switched to a farther vehicle, ACC can gradually reduce the following-distance demand for the future period after its predicted departure. The radar track and its measured distance and speed remain available.

The adjustment requires continuous measured outward motion and sufficient separation until the vehicle body clears the path. It affects only predictions after clearance plus 0.30 seconds, and is limited to half the selected time gap, at most 0.50 seconds or 8 m. Closer leadTwo vehicles, traffic stops, cruise limits, and the original collision-warning trajectory remain in the calculation. Loss of evidence cancels the adjustment; it does not operate during pedal override, in blended mode, or for stationary or strongly braking leads. See [radar behavior](radar.md).
