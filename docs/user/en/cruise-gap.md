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
6. [Following responsiveness](#lead-response)
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

The related settings are `MyDrivingMode` and `MyDrivingModeAuto`.

### `MyDrivingMode`

Modes set maximum acceleration, braking allowance and base time gap, and cap the selected following responsiveness.

| Value | Mode | Max acceleration | `comfort_brake` | Time gap | Lead response cap |
|---:|---|---:|---:|---:|---:|
| 1 | Eco | ×0.9 | ×1.0 | ×1.1 | 2 |
| 2 | Safe | ×0.8 | ×0.9 | ×1.2 | 3 |
| 3 | Normal | ×1.0 | ×1.0 | ×1.0 | Selected value |
| 4 | High | ×1.2 | ×1.0 | ×1.0 | Selected value |

The gap-specific `LeadAccelResponseTF1`–`TF4` choice or common `LeadAccelResponse` is resolved before the mode cap. A selected 5 becomes 3 in Safe or 2 in Eco, while lower choices such as 1 and 0 are never raised. The same caps apply in manual modes, without changing stored settings.

A smaller `comfort_brake` increases the calculated stopping-distance term. Every response level retains user TF, `SpeedTFFactor` and the mode gap multiplier; temporary gap-recovery headroom and deceleration allowance remain separate.

The separate mode-specific jerk adjustment and former Safe level-4/5 acceleration taper are removed. Baseline jerk costs remain; the effective `LeadAccelResponse` controls cost changes during eligible lead response and gap recovery.

A decreasing mode gap multiplier releases at 0.05 per second: about four seconds from Safe to Normal or two from Eco to Normal. Separate deceleration allowance may remain in the total target. Increasing mode margins use the existing TF rise ramp; an explicit driver selection of a smaller gap is not separately delayed.

> [!WARNING]
> High mode raises the acceleration ceiling by 20% and disables traffic stop/go detection. Automatic selection never chooses High.

### `MyDrivingModeAuto`

`0` uses the stored mode, `1` selects Normal↔Safe, and `2` selects Eco↔Safe. Stopping approaches and sustained slow following select Safe; this detection does not select control leads or issue braking commands.

- **Stopping approach:** A lead at or below 5 km/h within the speed-dependent approach envelope for about 0.3 seconds selects Safe. The envelope is `ego speed² / (2 × 2.4) + 2 × ego speed` metres, clamped to 12–200 m; speeds in these formulas are in m/s.
- **Sustained slow following:** Ego at or below 35 km/h and a lead at or below 30 km/h within following range for eight seconds selects Safe. Following range is `12 + 3 × ego speed` metres, clamped to 30–80 m.
- **Lead acceleration:** Outside a stopping approach, lead acceleration above 1.5 m/s² for about 0.5 seconds restores Normal/Eco without waiting for six seconds of flow recovery.
- **Flow recovery:** Both vehicles at or above 35 km/h, or a lead at or above 15 km/h pulling away by at least 1 m/s with distance at least `8 + 1.8 × ego speed` metres, must persist for six seconds. Lead acceleration below -0.2 m/s² restarts recovery confirmation.
- **Clear road:** Valid observations of no lead while ego travels at least 15 km/h for four seconds restore the base mode. Losing a lead while stopped does not restore it.

Acceleration spikes shorter than about 0.5 seconds do not release Safe. A changed lead track restarts acceleration and flow confirmation; invalid or stale inputs and lead loss reset acceleration confirmation.

Manually changing the stored `MyDrivingMode` suspends automatic selection until the planner restarts. The displayed actual mode can differ from the stored choice during automatic operation.

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
| `VEgoStopping` | `50` → 0.50 m/s | Higher values enter stopping state at a higher planned speed |
| `AChangeCostStarting` | MPC acceleration-change cost | Higher values smooth initial acceleration changes |

### `StopDistanceCarrot`

Range 400–1000 cm, step 10 cm. The code divides by 100 and uses it as the fixed-distance term:

    ego braking distance + time gap × ego speed + StopDistance - lead braking distance

It is therefore not the actual moving following distance. Its direct effect is clearest near zero speed behind a stopped lead. When there is no active `leadOne` but the camera model consistently associates a stationary vehicle with the E2E stop endpoint, the planner first corrects that endpoint toward the inferred vehicle position and then applies this fixed clearance. No SCC/radar object is created. Although the catalog description says “stop position ×0.8,” the running code does not apply 0.8.

### `StoppingAccel` · Stopping acceleration

Default `-50`, range `-100 to -50`, step `10`. `-50` means `-0.50 m/s²`; `-100` means `-1.00 m/s²`. It is used across brands when openpilot controls longitudinal motion and does not directly control stock ACC longitudinal operation.

The original behavior enters stopping after `shouldStop` when actual acceleration exceeds the setting. Existing state-transition exceptions, including a nearby lead, remain. During stopping, requests weaker than the target become more negative at the vehicle's stopping rate; stronger requests are retained. Soft hold uses the vehicle-specific stationary-hold acceleration.

- **Raise it (toward −50):** requests weaker deceleration and delays the ordinary acceleration-based handover to stopping.
- **Lower it (toward −100):** requests stronger deceleration and advances that handover. It can increase stopping impact.

Changes apply within about one second, including while driving. Stronger output already reached during a stop is retained, so selecting a weaker target does not immediately reduce braking. Control clamps old or directly written out-of-range values to the range endpoints; unreadable values use the default.

<a id="canfd-stopping"></a>
### CANFD stopping and retry

Retry runs by default on Hyundai/Kia CANFD with openpilot longitudinal control. There is no separate setting, and the removed `CanfdStopRetry` value is not read. Conventional CAN vehicles and stock ACC longitudinal control are outside its scope.

- Stop intent and acceleration use the original control path. The additional one-second stop preview, forced convergence to -0.50 m/s² after StopReq, and two-frame soft-hold preparation are removed.
- While StopReq is active, aReqRaw follows control with `StoppingAccel`, and aReqValue uses normal packet limiting. InfoDisplay and byte7 remain zero; the lower band uses a fixed experimental value of 0.20 without copying stock SCC values.
- At low speed, elapsed time or distance alone does not release StopReq while deceleration continues. Acceleration rising from negative toward zero alone does not trigger retry either. Retry requires a sustained speed rebound with positive acceleration, or sustained loss of deceleration with insufficient speed reduction.
- Retry releases StopReq and requests the stronger deceleration of the existing request and -0.50 m/s², then reasserts once. Further failure retains negative acceleration requests without repeated toggling. This does not change the planner's departure decision or add reverse-direction detection.
- Accelerator input, cruise disengagement, and interlocks such as Auto Hold cancel it. Requests while the brake is pressed are allowed only for an armed soft hold with every speed input at or below 0.10 m/s.

> [!CAUTION]
> Retry does not guarantee complete stopping or collision prevention across vehicles. Any reduction in stopping impact also requires vehicle validation.

### `VEgoStopping`

Range 1–100, step 5. A value of 50 is 0.50 m/s (about 1.8 km/h). `shouldStop` becomes true when both the planner's target speeds at the control-delay horizon and one second later are below this threshold.

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

### Gap cycle levels

`CruiseGapLevels` sets how many levels the gap button cycles through. The minimum is 2; the maximum and default are the vehicle-supported count. Four-level vehicles offer 2–4, and three-level vehicles offer 2–3.

- 2 levels: TF1↔TF2.
- 3 levels: TF1→TF3→TF2→TF1.
- 4 levels: the existing TF1→TF4→TF3→TF2→TF1 sequence.

Changes apply on the next gap-button press. If the current level is outside the new range, the first press selects the configured maximum. Unused TF and gap-specific following responsiveness values are preserved. Holding the gap button still changes driving mode. This applies with openpilot longitudinal control; stock ACC retains its own levels.

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

1. Use the base TF of the cruise-gap level selected with the button.
2. Apply the speed TF multiplier and driving-mode adjustment. Lead response levels 4–5 use the same calculation.
3. Hold baseline TF against reduction during braking, then add `TFollowDecelBoost` once.
4. Apply TF increases gradually. Release extra deceleration margin at 0.10 seconds per second.

### `SpeedTFFactor` · Speed TF multiplier

Increases the selected base TF linearly with speed. Range `10–30`, default `10`, step `1`. The screen shows `1.0×–3.0×`.

- `10`: no speed adjustment.
- `20`: 1.0× when stopped, 1.5× at 50 km/h, 2.0× at 100 km/h and 3.0× at 200 km/h.
- Higher values give a longer TF at the same speed. Following responsiveness is unchanged.

With base TF 0.50 seconds and setting 20, TF is 0.50/0.75/1.00 seconds at 0/50/100 km/h. Driving-mode and deceleration adjustments follow. The speed-adjusted result is not clipped to the largest TF1–4 setting or the former two-second cap.

The former `EnableSpeedTF` setting is removed; its value is not converted to the new multiplier. The new setting starts with no speed adjustment. Existing base TF and common following responsiveness values are retained.

### Target following-distance marker

The bar on the driving path and its `25 m` label show the current following target. They include speed TF, relative-speed/stopping-distance adjustments, response-dependent extra headroom, and permitted lane-change/cutout relief.

The bar is hidden without a valid lead. With two leads, it shows the more restrictive following reference. It is not the measured lead distance or a guaranteed safety boundary. The selected button level does not change automatically.

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

For a clean baseline, use `SpeedTFFactor=10`, `DynamicTFollowLC=100`, `MyDrivingMode=3`, and `MyDrivingModeAuto=0`. If the result is still wrong, check the base gaps, stop distance, selected personality, and radar lead before adding dynamic features.

<a id="lead-response"></a>
## 6. Following responsiveness

Use `LeadAccelResponse` to adjust response to a lead starting, accelerating or being approached. Its range is 0–5; the default 0 disables acceleration boost and recovers extra TF most slowly.

### `LeadAccelResponseTF1`–`LeadAccelResponseTF4`

Assigns following responsiveness to each cruise-gap level. Range `-1–5`; the default `-1` is displayed as **Use common**.

Changing the cruise-gap level with the button while driving applies that gap's response. Switching gaps does not change the saved common value.

- `-1`: use the common `LeadAccelResponse` value.
- `0–5`: use this value at this gap. `0` is an explicit override, not inheritance.

For example, set TF1 and TF2 to 50 and their responses to 5 and 3. Both use the same base following time, while the button changes response. The selected response controls both acceleration boost and recovery of extra headroom. A response-level change reinitializes headroom under the new response. The speed multiplier applies to every gap.

### `LeadAccelResponse` · Common value

Sets lead-start and acceleration response for gaps configured to use the common value. Levels 1–3 soften small changes and response near the target gap; level 4 is quick and level 5 retains the immediate maximum response. The selected TF remains the reference; a separate lead-jerk adjustment no longer expands or shrinks TF.

The table below describes the **effective response level after the mode cap**, not necessarily the stored choice.

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

Every response level uses TF with speed and driving-mode adjustments. The former levels 4–5 speed-TF bypass is removed. `CruiseMaxVals`, curve, cut-in, lead-distance and danger-distance limits, and deceleration preview remain active. No acceleration is added after MPC. This setting does not change `AChangeCostStarting` or PID gains. Lower levels do not delay braking required by an urgent approach.

Deceleration preview operates independently of the response level. During active control, remaining correction releases progressively when relative acceleration eases or the lead switches between radar and vision or disappears. Accelerator or brake intervention and longitudinal control exit clear it immediately. Crossing zero relative acceleration does not remove the correction in one step. New hard-deceleration requests retain the existing preview attack rate and braking bounds.

Levels 0–4 initially capture half the excess over the base following distance when acquiring a radar lead. While the lead is pulling away, existing headroom is retained and a larger candidate replenishes it gradually. Headroom remains available even after lead acceleration ends if the gap is still opening, then recovers gradually as relative speed settles. Base TF plus extra TF is capped at 2.5 seconds without reducing a larger base TF. Two filter stages smooth the recovery rate, most gradually at level 0. MPC predicts future headroom with the same capture and recovery rules. A stopped lead retains it; a slow lead recovers it more slowly. Level 5 adds no extra TF. This replaces the previous relative-closing-speed distance allowance rather than stacking with it.

Headroom applies to a stable radar lead in normal ACC. Accelerator override, disengagement, forced deceleration and lane changes disable it. Unlike acceleration boost, it also applies at level 0 and during stopping. Target loss/replacement or a level change does not transfer the old allowance.

Acquisition allowance ramps in over 0.8 seconds with zero slope at both ends. Afterward, measured relative speed filtered with a 0.3-second time constant above 0.2m/s holds or replenishes headroom. The internal headroom target rises at no more than 0.5 TF seconds per second, and applied extra TF follows it smoothly. Candidates are absolute targets, not repeatedly added increments.

When relative speed settles or the gap closes, two filter stages recover the headroom. Applied TF retains its rate of change across hold, replenishment and recovery; ongoing replenishment can briefly continue raising it before it falls. A newly acquired slower or stopped lead can receive initial headroom without an opening gap. The stopped-lead hold rule is described below.

| Level | Base recovery time scale | Extra TF |
|---|---:|---|
| 0 | 5 seconds | Active |
| 1 | 4 seconds | Active |
| 2 | 3 seconds | Active |
| 3 | 2 seconds | Active |
| 4 | 1 second | Active |
| 5 | — | None |

Lead speed at or below 0.3m/s holds extra TF. From 0.3 to 5m/s, recovery strength increases linearly with lead speed; at 5m/s and above the table applies. Starting with equal recovery states and no new capture, roughly 41% remains after the listed time scale. Recovery begins with zero slope, builds gradually, then tapers. For a stopped lead, the extra-TF distance term shrinks with ego speed toward normal stopping clearance.

Capture uses actual distance minus the base target distance, with a 1m/s minimum divisor at low ego speed. Without excess over the base target including braking-distance terms, there is no new extra TF. Only the MPC comfort reference changes; physical lead positions, base TF, danger constraints and braking limits stay unchanged. Temporary TF does not guarantee a particular braking onset or ride quality. Ego-deceleration `TFollowDecelBoost` remains separate existing TF processing.

Lead response uses the final level after the mode cap. A selected 5 uses level 3 in Safe or level 2 in Eco, including that level’s gap-recovery headroom. No separate mode-specific jerk adjustment or former Safe-only acceleration taper is stacked on top. Level 5 in Normal and High retains maximum response.

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

### Cruise coasting margin (`CruiseCoastingPercent`)

Relaxes cruise braking that would bring a small overspeed back to the set speed. The default is **0%**, the range is **0–10%**, and the step is **1%**. **0% retains existing control.**

- Does not raise the set speed or MPC target, disable SCC, or add positive acceleration commands.
- For a 100km/h set speed and a 5% margin, braking relief applies between 100 and 105km/h. It does not accelerate the vehicle to 105km/h.
- Relief eases in over the first 10% of the band; normal braking returns over the final 40%. In this example, relief increases from 100 to 100.5km/h and braking returns from 103 to 105km/h. The ceiling is a brake-restoration threshold, not a guaranteed maximum actual speed.
- Applies only to ordinary cruise with openpilot longitudinal control, a reference above 10km/h, and the set speed, margin setting and eligibility unchanged for at least one second. The physical-speed reference is fixed using the conversion ratio at entry; later ratio changes neither restart the wait nor raise the reference.
- Leads, cut-in candidates, stopping, curve acceleration limiting, ATC, lane changes, and Experimental Mode prevent relief. A changed set speed or margin, or loss of eligibility, requires a new reference and another one-second wait.
- Navigation or other speed caps at or below the coasting ceiling prevent relief. External deceleration, pedal input, target changes, or invalid inputs give priority to normal control.
- Does not apply while `CruiseEcoControl` raises the target or the existing CarrotCruise acceleration-limiting mode is active. This setting is separate from `CarrotCruiseDecel`.

Adjust under Settings > Driving > Cruise & Gap > Carrot Cruise. Changes are read approximately once per second while running. Increase the margin to allow more overspeed before normal braking returns, or select 0% to restore existing control.

A zero SCC acceleration request does not guarantee zero regeneration or braking. Actual regeneration and ride comfort depend on the vehicle; driving validation has not yet been completed.

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
