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

Catalog defaults and initial Params values currently differ for `CruiseMaxVals1` through `6`, `StopDistanceCarrot`, and `DynamicTFollowLC`. Use the value shown on your device as the baseline.

<a id="driving-mode"></a>
## 1. Driving mode

### `MyDrivingMode`

| Value | Mode | Max acceleration | `comfort_brake` | Time-gap term | Additional behavior |
|---:|---|---:|---:|---:|---|
| `1` | Eco | ×0.9 | ×0.9 | ×0.9, then clamped | Traffic-light detection retained |
| `2` | Safe | ×0.8 | ×0.8 | ×0.8, then clamped | Congestion state used by auto mode |
| `3` | Normal | ×1.0 | ×1.0 | ×1.0 | Baseline |
| `4` | High speed | ×1.2 | ×1.0 | ×1.0 | Traffic stop/go detection forced off |

A smaller `comfort_brake` increases the distance term calculated for stopping, while `t_follow` is reduced by 10% in Eco and 20% in Safe mode. “Safe” therefore does not simply mean a longer time gap; final distance depends on ego and lead speeds.

> [!WARNING]
> High-speed mode raises the acceleration ceiling by 20% and ignores traffic-light control.

### `MyDrivingModeAuto`

`0` uses the stored mode. `1` switches only between Safe and Normal according to traffic conditions; it never automatically selects Eco or High-speed mode.

The current code enters congestion after repeated observations of either:

- Lead distance at most 12 m and lead speed at most 2 km/h; or
- Lead speed below 5 km/h, lead acceleration below 0.2 m/s², ego speed above 1 km/h, and lead distance below 200 m.

It exits when lead acceleration exceeds 1.5 m/s², ego speed exceeds 35 km/h, or no lead is present within 200 m. The running code uses **35 km/h**, despite the setting description saying 20 km/h.

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
- Exactly `0`: use the vehicle's `CP.stopAccel` instead of the user value.

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

1. Select a base gap from the personality or speed table.
2. Apply the positive `EnableSpeedTF` low-speed reduction.
3. During deceleration, suspend that reduction and add `TFollowDecelBoost`.
4. Apply Eco/Safe driving-mode factors.
5. Clamp to the minimum and maximum of the four base values.
6. Apply `DynamicTFollowLC` during a lane change; otherwise apply `DynamicTFollow` when a lead exists.
7. Rate-limit increases so the gap does not jump suddenly.

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

Selecting following-distance level 1 with `LeadAccelResponse=4` or `5` while tracking a lead is an exception. `TFollowGap1` takes priority over positive or negative `EnableSpeedTF` adjustments and Eco/Safe gap factors only while a stable radar lead is accelerating positively and the gap is opening. When lead acceleration falls to `0.1 m/s²` or below, the exception is removed immediately and normal gap control—including the existing TF increase ramp—and braking behavior resume. It does not change the no-lead cruise target; `TFollowDecelBoost`, lane-change, and `DynamicTFollow` adjustments can still apply.

### `DynamicTFollow`

Range 0–100, step 1; zero disables it. It changes time gap from lead jerk `jLead`:

- Rapid change toward lead deceleration increases the gap.
- Rapid change toward lead acceleration decreases the gap.
- Around `jLead=-0.5` to `+0.5`, there is little adjustment.

A value of 50 permits up to about ±0.50 s under a strong change; 100 permits about ±1.00 s. The result is clamped to 0.3–2.0 s, and increases are smoothed.

### `DynamicTFollowLC`

For about 1.5 seconds after lane-change start, this percentage multiplies the time gap. Range 20–100, step 5: 100 retains the gap; 80 uses 80%; 20 is the code's minimum ratio.

The catalog default is currently 0 even though the range is 20–100 and initial Params value is 100. If zero reaches the running code, it is forced to 20%.

> [!WARNING]
> This multiplier is applied after the base time gap has been clamped to at least 0.3 s, and there is no second 0.3-second clamp. Applying 20% to 1.10 s can therefore produce about 0.22 s. Low values are very aggressive.

### `TFollowDecelBoost`

At ego acceleration around -0.2 m/s² or below, the code first prevents speed adjustment from reducing the target gap. This prevention works even when the setting is zero. The setting then adds gap based on deceleration strength.

At `TFollowDecelBoost=50`, the addition is approximately 0.03 s at -0.3 m/s², 0.125 s at -1.0 m/s², and a maximum around 0.25 s at -2.5 m/s². Range is 0–100 in steps of 10.

For a clean baseline, use `EnableSpeedTF=0`, `DynamicTFollow=0`, `DynamicTFollowLC=100`, `MyDrivingMode=3`, and `MyDrivingModeAuto=0`. If the result is still wrong, check the base gaps, stop distance, selected personality, and radar lead before adding dynamic features.

<a id="lead-response"></a>
## 6. Lead-vehicle response

| Setting | Range/scale | Role |
|---|---|---|
| `LeadAccelResponse` | 0–5, default 0 | Responsiveness to a lead starting or accelerating at following-distance level 1 |
| `RadarReactionFactor` | 0–200%, default 100% | How long measured lead acceleration persists into the future |
| `JLeadFactor3` | 0–100, ×0.01 | How much lead acceleration change enters future trajectory prediction |

### `LeadAccelResponse`

When the lead starts or accelerates and the gap begins to open, this setting reduces MPC's acceleration-change and jerk costs by level so it can select a faster new acceleration trajectory. It operates **only with following-distance level 1 (aggressive/TF1)**. On openpilot-longitudinal vehicles that cannot report `pcmCruiseGap`, it uses the selected level-1 personality instead. Levels 3–5 also operate when a stable radar lead remains but the current MPC source changes to `cruise`.

| Value | UI meaning | Active `aChangeCost` | Additional multiplier on existing jerk cost | Strong response ends |
|---:|---|---:|---:|---:|
| `0` | Disabled | `200` | `100%` | Not applicable |
| `1` | Weak | `170` | `95%` | Configured TF reached |
| `2` | Mild | `130` | `80%` | Configured TF reached |
| `3` | Brisk (recommended) | `80` | `60%` | Configured TF reached |
| `4` | Urgent follow | `36` | `35%` | Configured TF reached |
| `5` | Maximum follow (test) | `10` | `15%` | Configured TF reached |

Lower `aChangeCost` releases the solution from the previous MPC acceleration plan, while lower jerk cost permits a steeper transition to the new acceleration. Level 3 is the brisk everyday choice, level 4 is for an urgent driver, and level 5 is the maximum test level intended to feel distinctly strong. These values reduce the active-driving base cost of `200`; they do not change `AChangeCostStarting` or velocity-PID gains.

No positive acceleration is added after MPC. `vTargetNow` and `aTarget` therefore come from the same MPC velocity and acceleration trajectory. `CruiseMaxVals` remains MPC's hard acceleration ceiling, while curve, cut-in pre-deceleration, lead-obstacle, and danger-distance limits remain intact.

A nonzero value responds only when all of these common gates pass:

- The normal ACC planner is active, no stop is requested, and the driver is not pressing the accelerator.
- The same radar track has been observed for at least three consecutive updates. Levels 1–2 require a radar-lead MPC source; levels 3–5 may also operate with a `cruise` source.
- Every level requires measured lead acceleration above `0.1 m/s²`. With a radar-lead source, levels 1–4 additionally require relative lead acceleration above the `0.1 m/s²` deadband.
- Current relative speed plus predicted lead acceleration shows the lead pulling away while respecting the level-specific relative-speed floor.
- With a `cruise` source, set speed exceeds current speed by more than `1 km/h`.

Every level uses the strong cost reduction only while actual distance exceeds the configured TF target. At or inside that target, the reduction is removed immediately, the default `aChangeCost=200` and normal jerk cost return, and ordinary MPC safely maintains the gap. Level 5 still requires relative speed of at least `-0.2 m/s` and a gap predicted to open within 0.5 seconds. If the lead reaches zero acceleration or begins decelerating, the existing MPC lead prediction and deceleration preview continue unchanged.

Levels 1–3 do not change the target time gap. The level 4–5 exception that prioritizes the configured `TFollowGap1` as the base target applies only during positive lead acceleration; `DynamicTFollow` and lane-change corrections may still apply afterward. Lead-braking response and stopping behavior retain normal control at every level. Every level remains inactive with the experimental blended planner, a vision-only lead, and following-distance levels 2–4. Use level 5 only when you can verify that short-gap starts do not cause unwanted acceleration.

### `RadarReactionFactor`

Radar acceleration and jerk form `aLeadTau`, used by MPC to predict how long the lead's current acceleration or deceleration will continue.

- Lower values assume the measured change persists longer and respond more quickly.
- Higher values let it decay sooner and may respond more smoothly but later.
- Too low can react to radar noise; too high can respond slowly to real lead braking.

### `JLeadFactor3`

The code smooths `jLead` as 10% new and 90% previous, multiplies by this percentage, clamps to -1 through +1, and inserts it into the future lead trajectory. Zero excludes jerk; 50 uses half; 100 uses the full allowed value.

> [!NOTE]
> Even with `JLeadFactor3=0`, `DynamicTFollow` separately uses raw `jLead`. Check both settings when isolating jerk-related behavior.

For a baseline, set `DynamicTFollow=0`, `LeadAccelResponse=0`, `JLeadFactor3=0`, and `RadarReactionFactor=100`. If only TF1 response to a lead starting or accelerating is late, raise `LeadAccelResponse` from level 1 one step at a time. Change only one setting at once, and restore immediately if surging or unintended acceleration appears.

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
